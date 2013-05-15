/****************************************************************************
**
** Copyright (c) 2011 libmv authors.
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to
** deal in the Software without restriction, including without limitation the
** rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in
** all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
** IN THE SOFTWARE.
**
****************************************************************************/

#include "klt.h"

#include "libmv/numeric/numeric.h"
#include "libmv/image/image.h"
#include "libmv/image/convolve.h"
#include "libmv/image/sample.h"

namespace libmv {

Tracker::Tracker(const FloatImage &image1, float x, float y, int half_pattern_size, int search_width, int search_height, int num_levels) :
  half_pattern_size(half_pattern_size),
  search_width(search_width),
  search_height(search_height),
  num_levels(num_levels),
  max_iterations(16),
  tolerance(0.2),
  min_determinant(1e-6),
  min_update_squared_distance(1e-6),
  sigma(0.9),
  lambda(0.05),
  x1(x),
  y1(y) {
  MakePyramid(image1, pyramid1);
}

void Tracker::MakePyramid(const FloatImage &image, float **pyramid) const {
  FloatImage pyramids[8];
  BlurredImageAndDerivativesChannels(image, sigma, &pyramids[0]);
  FloatImage mipmap1,mipmap2;
  mipmap1 = image;
  for (int i = 1; i < num_levels; ++i) {
    DownsampleChannelsBy2(mipmap1, &mipmap2);
    mipmap1 = mipmap2;
    BlurredImageAndDerivativesChannels(mipmap1, sigma, &pyramids[i]);
  }
  int width = search_width, height = search_height;
  for (int i = 0; i < num_levels; i++, width /= 2, height /= 2) {
    pyramid[i] = (float*)malloc(sizeof(float)*width*height*3);
    memcpy(pyramid[i],pyramids[i].Data(),sizeof(float)*width*height*3);
  }
}

bool Tracker::Track(const FloatImage &image2, float *x2, float *y2) {
  // Create all the levels of the pyramid, since tracking has to happen from
  // the coarsest to finest levels, which means holding on to all levels of the
  // pyramid at once.
  float* pyramid2[8];
  MakePyramid(image2, pyramid2);

  // Shrink the guessed x and y location to match the coarsest level + 1 (which
  // when gets corrected in the loop).
  *x2 /= pow(2., num_levels);
  *y2 /= pow(2., num_levels);

  for (int i = num_levels - 1; i >= 0; --i) {
    // Position in the first image at pyramid level i.
    float xx = x1 / pow(2., i);
    float yy = y1 / pow(2., i);

    // Guess the new tracked position is where the last level tracked to.
    *x2 *= 2;
    *y2 *= 2;

    // Track the point on this level with the base tracker.
    bool succeeded = TrackImage(pyramid1[i], pyramid2[i], search_width >> i, half_pattern_size >> i, xx, yy, x2, y2);

    if (i == 0 && !succeeded) {
      // Only fail on the highest-resolution level, because a failure on a
      // coarse level does not mean failure at a lower level (consider
      // out-of-bounds conditions).
      // Adapt marker to new image
      for(int i = 0; i < num_levels; i++) {
        free(pyramid1[i]); // Free old image
        pyramid1[i] = pyramid2[i]; //Use new image
      }
      x1 = *x2;
      y1 = *y2;
      return false;
    }
  }

  // Adapt marker to new image
  for(int i = 0; i < num_levels; i++) {
    free(pyramid1[i]); // Free old image
    pyramid1[i] = pyramid2[i]; //Use new image
  }
  x1 = *x2;
  y1 = *y2;

  return true;
}

// An improved KLT algorithm that enforces that the tracking is time-reversible
// [1]. This is not the same as the "symmetric" KLT that is sometimes used.
// Anecdotally, this tracks much more consistently than vanilla KLT.
//
// [1] H. Wu, R. Chellappa, and A. Sankaranarayanan and S. Kevin Zhou. Robust
//     visual tracking using the time-reversibility constraint. International
//     Conference on Computer Vision (ICCV), Rio de Janeiro, October 2007.
bool Tracker::TrackImage(const float* image1,
                         const float* image2,
                         int size,
                         int half_pattern_size,
                         float  x1, float  y1,
                         float *x2, float *y2) const {
  for (int i = 0; i < max_iterations; ++i) {
    // Compute gradient matrix and error vector.
    Mat2f A, B, C;
    A = B = C = Mat2f::Zero();

    Vec2f R, S, V, W;
    R = S = V = W = Vec2f::Zero();

    // Prevent leaving search region
    int min = half_pattern_size, max = size-half_pattern_size-1;
    if(isnan(x1) || isnan(y1) || isnan(*x2) || isnan(*y2) ||
       x1 < min || y1 < min || *x2 < min || *y2 < min ||
       x1 >= max || y1 >= max || *x2 >= max || *y2 >= max) {
      return false;
    }

    // FIXME: don't resample source pattern
    const float* pattern1 = &image1[(int(y1)*size+int(x1))*3];
    const float* pattern2 = &image2[(int(*y2)*size+int(*x2))*3];
    float u1 = x1-int(x1), v1 = y1-int(y1);
    float u2 = *x2-int(*x2), v2 = *y2-int(*y2);

    for (int y = -half_pattern_size; y <= half_pattern_size; ++y) {
      for (int x = -half_pattern_size; x <= half_pattern_size; ++x) {
        const float* s1 = &pattern1[(y*size+x)*3];
        const float* s2 = &pattern2[(y*size+x)*3];

    #define sample(n,i) ((s##n[       i] * (1-u##n)  + s##n[       3+i] * u##n) * (1-v##n) \
                       + (s##n[size*3+i] * (1-u##n)  + s##n[size*3+3+i] * u##n) * v##n)

        float I = sample(1,0);
        float J = sample(2,0);

        Vec2f gI, gJ;
        gI << sample(1,1), sample(1,2);
        gJ << sample(2,1), sample(2,2);

        // Equation 15 from the paper.
        A += gI * gI.transpose();
        B += gI * gJ.transpose();
        C += gJ * gJ.transpose();
        R += I * gI;
        S += J * gI;
        V += I * gJ;
        W += J * gJ;
      }
    }

    // In the paper they show a D matrix, but it is just B transpose, so use that
    // instead of explicitly computing D.
    Mat2f Di = B.transpose().inverse();

    // Computes U and e from the Ud = e equation (number 14) from the paper.
    Mat2f U = A*Di*C + lambda*Di*C - 0.5*B;
    Vec2f e = (A + lambda*Mat2f::Identity())*Di*(V - W) + 0.5*(S - R);

    // Solve the linear system for the best update to x2 and y2.
    float det = U.determinant();
    if (det < min_determinant) {
      // The determinant, which indicates the trackiness of the point, is too
      // small, so fail out.
      return false;
    }
    Vec2f d = U.lu().solve(e);

    // Update the position with the solved displacement.
    *x2 += d[0];
    *y2 += d[1];

    // If the update is small, then we probably found the target.
    if (d.squaredNorm() < min_update_squared_distance) {
      return true;
    }
  }
  // Getting here means we hit max iterations, so tracking failed.
  return false;
}

}  // namespace libmv

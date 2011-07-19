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
#include <libmv/image/image_pyramid.h>

#include <QDebug>
#include <QImage>
#include <QDialog>
#include <QHBoxLayout>
#include <QLabel>

#include <Eigen/Core>
#include <Eigen/LU>
typedef Eigen::Vector2f Vec2f;
typedef Eigen::Matrix<float, 2, 2> Mat2f;

#include "klt.h"

namespace libmv {

void Debug(const T** images, int size, int count, QString text) {
  QDialog dialog;
  dialog.setWindowTitle(text);
  QHBoxLayout layout(&dialog);
  QLabel labels[count];
  for (int i = 0; i < count; i++) {
    QImage image(size,size,QImage::Format_Indexed8);
    ubyte* dst = image.bits();
    for(int y = 0; y < size; y++) for(int x = 0; x < size; x++) {
      dst[y*size+x] = abs(images[i][(y*size+x)*4]);
    }
    labels[i].setPixmap(QPixmap::fromImage(image));
    labels[i].setScaledContents(true);
    labels[i].setMinimumSize(size*4,size*4);
    layout.addWidget(&labels[i]);
  }
  dialog.exec();
}

Tracker::Tracker(ubyte* image, int half_pattern_size, int search_size, int pyramid_count)
  : half_pattern_size_(half_pattern_size), search_size_(search_size),
    pyramid_count_(pyramid_count), x_(search_size/2), y_(search_size/2) {
  int size = search_size_;
  for (int i = 0; i < pyramid_count_; i++, size/=2) {
    pyramid_[0][i] = new T[4*size*size];
    pyramid_[1][i] = new T[4*size*size];
  }
  NextImage(image);
}

Tracker::~Tracker() {
  for (int i = 0; i < pyramid_count_; i++) {
    delete pyramid_[0][i];
    delete pyramid_[1][i];
  }
}

/*static void ConvolveAndDownsample(ubyte* src, T* dst, ubyte* down, int size) {
  memset(dst,0,sizeof(T)*4*size*size); //DEBUG
  for(int y = 1; y < size-1; y++) {
    for(int x = 1; x < size-1; x++) {
      // TODO(MatthiasF): SIMD
      ubyte* s = &src[y*size+x];
      T* d = &dst[(y*size+x)*4];
      #define S(i,j) (float)s[j*size+i]
      // 3x3 Integer Gaussian
      d[0] = ( 1  * S(-1,-1) + 2  * S(0,-1) + 1  * S(1,-1) +
               2  * S(-1, 0) + 4  * S(0, 0) + 2  * S(1, 0) +
               1  * S(-1, 1) + 2  * S(0, 1) + 1  * S(1, 1) ) / 16 / 256;
      // 3x3 Sharr Horizontal
      d[2] = -( 3  * S(-1,-1) + 10 * S(0,-1) + 3  * S(1,-1) -

               3  * S(-1, 1) - 10 * S(0, 1) - 3  * S(1, 1) ) / 16 / 256;
      // 3x3 Sharr Vertical
      d[1] = -( 3  * S(-1,-1)                - 3  * S(1,-1) +
               10 * S(-1, 0)                - 10 * S(1, 0) +
               3  * S(-1, 1)                - 3  * S(1, 1) ) / 16 / 256;
      // 2x2 Box Downsample //FIXME: once every 4 pixel
      down[(y/2)*(size/2)+(x/2)] = ( S(0,0) + S(1,0) +
                                     S(0,1) + S(1,1) ) / 4;
      #undef S
    }
  }
}*/

void Tracker::NextImage(ubyte* src /*image*/) {
  // Swap buffers
  for (int i = 0; i < kMaxPyramidCount; i++) {
    T* swap    = pyramid_[0][i];
    pyramid_[0][i] = pyramid_[1][i];
    pyramid_[1][i] = swap;
  }
  /*// Filter next image
  ConvolveAndDownsample(image, pyramid_[1][0], image, search_size_);
  int size = search_size_ / 2;
  for (int i = 1; i < pyramid_count_; i++, size /= 2) {
    ConvolveAndDownsample(image, pyramid_[1][i], image, size);
  }*/
  libmv::FloatImage image;
  int size = search_size_;
  image.resize(size,size);
  for (int y = 0; y < size; y++) {
    for (int x = 0; x < size; x++) {
      image(y, x, 0) = *src++;
    }
  }
  ImagePyramid* pyramid = MakeImagePyramid(image,pyramid_count_,0.9);
  for (int i = 0; i < pyramid_count_; i++, size /= 2) {
    FloatImage image = pyramid->Level(i);
    for (int y = 0; y < size; y++) {
      for (int x = 0; x < size; x++) {
        pyramid_[1][i][(y*size+x)*4+0] = image(y, x, 0);
        pyramid_[1][i][(y*size+x)*4+1] = image(y, x, 1);
        pyramid_[1][i][(y*size+x)*4+2] = image(y, x, 2);
        pyramid_[1][i][(y*size+x)*4+3] = 0;
      }
    }
  }
  delete pyramid;
  //delete image;
}

#define PYRAMID 0
bool Tracker::Track(ubyte* image, float *x, float *y) {
  // Shift in the new image
  // FIXME(MatthiasF): Full adaptation require bigger pattern to avoid drifting
  NextImage(image);
#if RETRACK
  // Track forward, getting new x, y.
  if (!TrackPyramid(pyramid_[0], pyramid_[1], x_, y_, x, y)) {
    return false;
  }
  // Now track backward, to get x0 and y0 which, if the track is
  // good, should match x_ and y_ (but may not if the track is bad).
  float x0, y0;
  if (!TrackPyramid(pyramid_[1], pyramid_[0], *x, *y, &x0, &y0)) {
    return false;
  }
  float dx = x_ - x0;
  float dy = y_ - y0;
  const float kTolerance = 0.2;
  if (dx*dx + dy*dy < kTolerance*kTolerance) {
#elif PYRAMID
  if(TrackPyramid(pyramid_[0], pyramid_[1], x_, y_, x, y)) {
#else
  *x = x_;
  *y = y_;
  if(TrackImage(pyramid_[0][0], pyramid_[1][0], search_size_, x_, y_, x, y)) {
#endif
    // Save position as starting point for next frame
    x_ = *x; y_ = *y;
    return true;
  }
  return false;
}

#if PYRAMID
bool Tracker::TrackPyramid(T** pyramid1, T** pyramid2,
                           float x0, float y0, float *x1, float *y1) const {
    // Scale coordinates to match coarsest level
    *x1 = x0 / (1<<pyramid_count_);
    *y1 = y0 / (1<<pyramid_count_);

    for (int i = pyramid_count_ - 1; i >= 0; i--) {
      // Scale old coordinates
      float x  = x0 / (1<<i);
      float y  = y0 / (1<<i);
      // Scale new coordinates
      *x1 *= 2;
      *y1 *= 2;

      // Track the point on this level with the base tracker.
      if (i == 0 && !TrackImage(pyramid1[i], pyramid2[i], search_size_ >> i, x, y, x1, y1)) {
        // Only fail on the highest-resolution level, because a failure on a
        // coarse level does not mean failure at a lower level (consider
        // out-of-bounds conditions).
        return false;
      }
    }
    return true;
}
#endif

bool Tracker::TrackImage(const T* image0, const T* image1, int size,
                         float x0, float y0, float *x1, float *y1) const {
  const int kMaxIterations = 16;
  for (int i = 0; i < kMaxIterations; i++) {
    // Check if search size is big enough
    if(*x1 <= half_pattern_size_ || *x1 >= size-half_pattern_size_-1 ||
       *y1 <= half_pattern_size_ || *y1 >= size-half_pattern_size_-1) {
      return false;
    }

    // Compute gradient matrix and error vector.
    Mat2f A, B, C;
    A = B = C = Mat2f::Zero();
    Vec2f R, S, V, W;
    R = S = V = W = Vec2f::Zero();

    // Compute 8bit interpolation weights.
    int fx0 = (int)(x0*256), fy0 = (int)(y0*256);
    int ix0 = fx0 >> 8, iy0 = fy0 >> 8;
    int u0 = fx0 & 0xFF, v0 = fy0 & 0xFF;
    const T* pattern0 = &image0[(iy0*size+ix0)*4];

    int fx1 = (int)(*x1*256), fy1 = (int)(*y1*256);
    int ix1 = fx1 >> 8, iy1 = fy1 >> 8;
    int u1 = fx1 & 0xFF, v1 = fy1 & 0xFF;
    const T* pattern1 = &image1[(iy1*size+ix1)*4];

    /*const T* pattern0 = &image0[(int(y0)*size+int(x0))*4];
    const T* pattern1 = &image1[(int(*y1)*size+int(*x1))*4];*/

    for (int y = -half_pattern_size_; y <= half_pattern_size_; y++) {
      for (int x = -half_pattern_size_; x <= half_pattern_size_; x++) {
        // TODO(MatthiasF): SIMD
        // fixed point bilinear sampling
#define sample(n,i) (float)((s##n[       i] * (256-u##n)  + s##n[       4+i] * u##n) * (256-v##n) \
                          + (s##n[size*4+i] * (256-u##n)  + s##n[size*4+4+i] * u##n) * v##n ) / (256*256)

        const T* s0 = &pattern0[(y*size+x)*4];
        const T* s1 = &pattern1[(y*size+x)*4];
        float I = sample(0,0), J = sample(1,0);
        Vec2f gI,gJ;
        gI << sample(0,1), sample(0,2);
        gJ << sample(1,1), sample(1,2);

        /*float I = pattern0[(y*size+x)*4+0], J = pattern1[(y*size+x)*4+0];
        Vec2f gI,gJ;
        gI << pattern0[(y*size+x)*4+1], pattern0[(y*size+x)*4+2];
        gJ << pattern1[(y*size+x)*4+1], pattern1[(y*size+x)*4+2];*/

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

    // Equation 14 from the paper.
    const float kLambda = 0.05;
    Mat2f U = A*Di*C + kLambda*Di*C - 0.5*B;
    Vec2f e = (A + kLambda*Mat2f::Identity())*Di*(V - W) + 0.5*(S - R);

    // Solve the linear system for the best update to x2 and y2.
    float det = U.determinant();
    const float kMinDeterminant = 1e-6;
    if (det < kMinDeterminant) {
      return false;
    }
    Vec2f d = U.lu().solve(e);

    /*const T* images[6] = { image0, image0+1, image0+2, image1, image1+1, image1+2 };
    Debug(images,size,6,QString("det %1 dX %2 dY %3").arg(det).arg(d[0]).arg(d[1]));*/
    //Q_ASSERT( !isnan(det) );

    // Update the position with the solved displacement.
    *x1 += d[0];
    *y1 += d[1];

    // If the update is small, then we probably found the target.
    const float kMinUpdatedSquaredDistance = 1e-6;
    if (d.squaredNorm() < kMinUpdatedSquaredDistance) {
      return true;
    }
  }
  // Getting here means we hit max iterations, so tracking failed.
  return false;
}

}  // namespace libmv

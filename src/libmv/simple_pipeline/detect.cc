// Copyright (c) 2007, 2008 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include <cassert>

#include "libmv/simple_pipeline/detect.h"
#include "libmv/image/image.h"
#include "libmv/image/convolve.h"
#include "libmv/image/sample.h"

namespace libmv {

static void FindLocalMaxima(const FloatImage &trackness,
                            float min_trackness,
                            int min_distance,
                            std::vector<Feature> *features) {
  for (int i = min_distance; i < trackness.Height()-min_distance; ++i) {
    for (int j = min_distance; j < trackness.Width()-min_distance; ++j) {
      if (   trackness(i,j) >= min_trackness
          && trackness(i,j) >= trackness(i-1, j-1)
          && trackness(i,j) >= trackness(i-1, j  )
          && trackness(i,j) >= trackness(i-1, j+1)
          && trackness(i,j) >= trackness(i  , j-1)
          && trackness(i,j) >= trackness(i  , j+1)
          && trackness(i,j) >= trackness(i+1, j-1)
          && trackness(i,j) >= trackness(i+1, j  )
          && trackness(i,j) >= trackness(i+1, j+1)) {
        Feature p = { i, j, trackness(i,j) };
        features->push_back(p);
      }
    }
  }
}

// Compute the gradient matrix noted by Z in Good Features to Track.
//
//   Z = [gxx gxy; gxy gyy]
//
// This function computes the matrix for every pixel.
static void ComputeGradientMatrix(const Array3Df &image_and_gradients,
                                       int window_size,
                                       Array3Df *gradient_matrix) {
  Array3Df gradients;
  gradients.ResizeLike(image_and_gradients);
  for (int j = 0; j < image_and_gradients.Height(); ++j) {
    for (int i = 0; i < image_and_gradients.Width(); ++i) {
      float gx = image_and_gradients(j, i, 1);
      float gy = image_and_gradients(j, i, 2);
      gradients(j, i, 0) = gx * gx;
      gradients(j, i, 1) = gx * gy;
      gradients(j, i, 2) = gy * gy;
    }
  }
  // Sum the gradient matrix over tracking window for each pixel.
  BoxFilter(gradients, window_size, gradient_matrix);
}

// Given the three distinct elements of the symmetric 2x2 matrix
//
//                     [gxx gxy]
//                     [gxy gyy],
//
// return the minimum eigenvalue of the matrix.
// Borrowed from Stan Birchfield's KLT implementation.
static float MinEigenValue(float gxx, float gxy, float gyy) {
  return (gxx + gyy - sqrt((gxx - gyy) * (gxx - gyy) + 4 * gxy * gxy)) / 2.0f;
}

// Compute trackness of every pixel given the gradient matrix.
// This is done as described in the Good Features to Track paper.
static void ComputeTrackness(const Array3Df gradient_matrix,
                             Array3Df *trackness_pointer) {
  Array3Df &trackness = *trackness_pointer; // FIXME(MatthiasF)
  trackness.Resize(gradient_matrix.Height(), gradient_matrix.Width());
  for (int i = 0; i < trackness.Height(); ++i) {
    for (int j = 0; j < trackness.Width(); ++j) {
      double t = MinEigenValue(gradient_matrix(i, j, 0),
                               gradient_matrix(i, j, 1),
                               gradient_matrix(i, j, 2));
      trackness(i, j) = t;
    }
  }
}

// TODO(keir): Use Stan's neat trick of using a 'punch-out' array to detect
// too-closes features. This is O(n^2)!
static void RemoveTooCloseFeatures(std::vector<Feature> *features,
                                   double squared_min_distance) {
  // mark close features with least trackness as invalid
  for(size_t i = 0 ; i < features->size() ; ++i) {
    Feature& a = features->at(i);
    for(size_t j = 0 ; j < i ; j++) {  // compare each feature pair once
      Feature& b = features->at(j);
      if ( b.trackness != 0  // skip invalidated features
           && Square(a.x-b.x)+Square(a.y-b.y) < squared_min_distance ) {
        ((a.trackness < b.trackness) ? a : b).trackness=0;  // invalid feature
      }
    }
  }
  // compress feature array in place by removing invalid features
  size_t size = 0;
  for(size_t i = 0 ; i < features->size() ; ++i) {
    const Feature& a = features->at(i);
    if( a.trackness != 0 ) features->at(size++) = a;
  }
  features->resize(size);
}

void DetectGoodFeatures(const Array3Df &image_and_gradients,
                        int window_size,
                        double min_trackness,
                        double min_feature_distance,
                        std::vector<Feature> *features) {
  Array3Df gradient_matrix;
  ComputeGradientMatrix(image_and_gradients, window_size, &gradient_matrix);

  Array3Df trackness;
  ComputeTrackness(gradient_matrix, &trackness);

  FindLocalMaxima(trackness, min_trackness, min_feature_distance, features);

  RemoveTooCloseFeatures(features, min_feature_distance * min_feature_distance);
}

}  // namespace libmv

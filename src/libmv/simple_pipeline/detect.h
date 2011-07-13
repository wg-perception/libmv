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

#ifndef LIBMV_CORRESPONDENCE_KLT_H_
#define LIBMV_CORRESPONDENCE_KLT_H_

#include <cassert>
#include <vector>

#include "libmv/image/image.h"

namespace libmv {

struct Feature {
  float x, y, trackness;
};

// FIXME(MatthiasF): Remove Array3Df usage
void DetectGoodFeatures(const Array3Df &image_and_gradients,
                        int window_size,
                        double min_trackness,
                        double min_feature_distance,
                        std::vector<Feature> *features);

// for debugging purpose only
void ComputeGradientMatrix(const Array3Df &image_and_gradients,
                                       int window_size,
                                       Array3Df *gradient_matrix);
void ComputeTrackness(const Array3Df gradient_matrix,
                      Array3Df *trackness_pointer);

}  // namespace libmv

#endif  // LIBMV_CORRESPONDENCE_KLT_H_

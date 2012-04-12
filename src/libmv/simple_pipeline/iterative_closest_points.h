// Copyright (c) 2012 libmv authors.
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

#ifndef LIBMV_SIMPLE_PIPELINE_ITERATIVE_CLOSEST_POINTS_H_
#define LIBMV_SIMPLE_PIPELINE_ITERATIVE_CLOSEST_POINTS_H_

#include "libmv/base/vector.h"
#include "libmv/numeric/numeric.h"

namespace libmv {

/*!
    Orients two point clouds using iterative closest points algorithm:

    - Assumes that initial transformation is known and some refirement
      of it is needed
    - Iteration:
      * Find pairs of closest points
      * Calculate rigid transform between them
      * If change in error fells smaller than threshold
        or max iterations were exceeded, stop algorithm

    Return transformation from points to reference_points as
      - R for rotation matrix
      - S for per-component scale factor
      - t for translation
 */
void IterativeClosestPoints(const vector<Vec3> &reference_points,
                            const vector<Vec3> &points,
                            Mat3 &R,
                            Vec3 &S,
                            Vec3 &t,
                            int max_iterations = 10,
                            double threshold = 1e-3);

}  // namespace libmv

#endif  // LIBMV_SIMPLE_PIPELINE_ITERATIVE_CLOSEST_POINTS_H_

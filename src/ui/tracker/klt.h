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

#ifndef LIBMV_TRACKING_KLT_H_
#define LIBMV_TRACKING_KLT_H_

#include "libmv/image/image.h"

namespace libmv {

class Tracker {
public:
  Tracker() {}
  /*!
      Construct a tracker to track the pattern centered in \a image.

      \note the tracker takes ownership of \a image
  */
  Tracker(int half_pattern_size, int search_size, int num_levels);
  /*!
      Track pattern from last image to \a image.

      \note the tracker takes ownership of \a image
  */
  bool Track(const FloatImage &image1,
             const FloatImage &image2,
             float  x1, float  y1,
             float *x2, float *y2) const;

private:
  bool TrackPyramid(const FloatImage& image1, const FloatImage& image2,
                    float x1, float y1, float *x2, float *y2) const;
  bool TrackImage(const FloatImage& image1, const FloatImage& image2,
                  float x1, float y1, float *x2, float *y2) const;

  int half_pattern_size;
  int search_size;
  int num_levels;
  int max_iterations;
  float tolerance;
  float min_determinant;
  float min_update_squared_distance;
  float sigma;
  float lambda;
};

}  // namespace libmv

#endif  // LIBMV_TRACKING_KLT_H_

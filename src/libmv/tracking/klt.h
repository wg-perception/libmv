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

#ifdef __cplusplus

#include "libmv/image/image.h"

namespace libmv {

class Tracker {
public:
  Tracker() {}
  /*!
      Construct a tracker to track the pattern centered in \a image1.

      The tracked pattern is a \a half_pattern_size * 2 + 1 patch in the center of image1
      \a image1 should be a square patch of size \a search_size
      This tracker will use pyramid tracking using \a num_levels levels.
  */
  Tracker(const FloatImage &image1, float x, float y, int half_pattern_size,
          int search_width, int search_height, int num_levels);
  /*!
      Track a point from last image to \a image2.

      \a x2, \a y2 should start out as a best guess for the position in \a
      image2. If no guess is available, (\a x1, \a y1) is a good start. Returns
      true on success, false otherwise

      \a image2 become the "last image" of this tracker.
  */
  bool Track(const FloatImage &image2, float *x2, float *y2);

private:
  void MakePyramid(const FloatImage &image, float** pyramid) const;
  bool TrackImage(const float* image1, const float* image2, int size, int half_pattern_size,
                  float x1, float y1, float *x2, float *y2) const;

  int half_pattern_size;
  int search_width;
  int search_height;
  int num_levels;
  int max_iterations;
  float tolerance;
  float min_determinant;
  float min_update_squared_distance;
  float sigma;
  float lambda;
  float* pyramid1[8];
  float x1,y1;
};

}  // namespace libmv

#else

class Tracker;
Tracker* NewTracker(const float *image1, float x, float y, int half_pattern_size, int search_size, int num_levels);
bool Track(const float *image2, float *x2, float *y2);

#endif

#endif  // LIBMV_TRACKING_KLT_H_

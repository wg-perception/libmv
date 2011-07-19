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

namespace libmv {

typedef char byte;
typedef unsigned char ubyte;
typedef float T; //for debugging only, code is optimized for integers

class Tracker {
public:
  Tracker() {}
  /*!
      Construct a tracker to track the pattern centered in \a image.

      \note the tracker takes ownership of \a image
  */
  Tracker(ubyte* image, int half_pattern_size, int search_size, int pyramid_count);
  ~Tracker();
  /*!
      Track pattern from last image to \a image.

      \note the tracker takes ownership of \a image
  */
  bool Track(ubyte* image, float *x, float *y);

private:
  void NextImage(ubyte* image);
  bool TrackPyramid(T** pyramid1, T** pyramid2,
                    float x1, float y1, float *x2, float *y2) const;
  bool TrackImage(const T* image1, const T* image2, int size,
                  float x1, float y1, float *x2, float *y2) const;

  int half_pattern_size_;
  int search_size_;
  int pyramid_count_;
  float x_, y_;
  static const int kMaxPyramidCount = 4;
  T* pyramid_[2][kMaxPyramidCount];
};

}  // namespace libmv

#endif  // LIBMV_TRACKING_KLT_H_

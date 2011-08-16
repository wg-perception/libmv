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

#ifndef LIBMV_SIMPLE_PIPELINE_DETECT_H_
#define LIBMV_SIMPLE_PIPELINE_DETECT_H_

#ifdef __cplusplus
namespace libmv {
#endif

typedef unsigned char ubyte;

/*!
    A Corner is the 2D location of a detected feature in an image.

    \a x, \a y is the position of the feature center in pixels (from image top-left).
    \a score is an estimate of how well the feature will be tracked.
    \a size can be used as an initial pattern size to track the feature.

    \sa Detect
*/
struct Corner {
  /// Feature center position in pixels (from image top-left)
  /// \note libmv might eventually support subpixel precision.
  float x, y;
  /// Trackness of the feature
  float score;
  /// Size of the feature in pixels
  float size;
};

/*!
    Detect features in an image.

    \a image is a single channel 8-bit image of size \a width x \a height

    \a corners is an array with space to hold \a *count \a Corner.
    \a *count is the maximum feature count to detect on input and the actual
    detected count on output.
*/
void Detect(ubyte* image, int width, int height, Corner* corners, int* count);

#ifdef __cplusplus
}
#endif

#endif

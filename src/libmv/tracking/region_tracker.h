// Copyright (c) 2011 libmv authors.
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

#ifndef LIBMV_TRACKING_TRACKER_H_
#define LIBMV_TRACKING_TRACKER_H_

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace libmv {

class RegionTracker {
 public:
  RegionTracker() {}
  virtual ~RegionTracker() {}

  /*!
      Track a point from \a image1 to \a image2.

      \a x2, \a y2 should start out as a best guess for the position in \a
      image2. If no guess is available, (\a x1, \a y1) is a good start. Returns
      true on success, false otherwise
  */
  virtual bool Track(const cv::Mat_<float> &image1,
                     const cv::Mat_<float> &image2,
                     double  x1, double  y1,
                     double *x2, double *y2) const = 0;
};

// Compute the gaussian blur of an image and the derivatives of the blurred
// image, and store the results in three channels. Since the blurred value and
// gradients are closer in memory, this leads to better performance if all
// three values are needed at the same time.
inline void BlurredImageAndDerivativesChannels(const cv::Mat &in,
                                        cv::Mat_<cv::Vec3f> &blurred_and_gradxy) {
  CV_Assert(in.channels() == 1);

  std::vector<cv::Mat> channels(3);
  cv::GaussianBlur(in, channels[0], cv::Size(5,5), 0, 0);
  cv::Sobel(channels[0], channels[1], CV_32F, 1, 0, 3, 1.0/(1 << (3*2-1-0-2)));
  cv::Sobel(channels[0], channels[2], CV_32F, 0, 1, 3, 1.0/(1 << (3*2-0-1-2)));

  cv::merge(channels, blurred_and_gradxy);
}

}  // namespace libmv

#endif  // LIBMV_CORRESPONDENCE_TRACKER_H_

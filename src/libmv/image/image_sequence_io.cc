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

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#include "libmv/image/image_converter.h"
#include "libmv/image/image_sequence_io.h"
#include "libmv/image/cached_image_sequence.h"

namespace libmv {

// An image sequence loaded from disk with caching behaviour.
class LazyImageSequenceFromFiles : public CachedImageSequence {
 public:
  virtual ~LazyImageSequenceFromFiles() {}

  LazyImageSequenceFromFiles(const std::vector<std::string> &image_filenames,
                        ImageCache *cache)
      : CachedImageSequence(cache),
        filenames_(image_filenames) {}

  virtual int Length() {
    return filenames_.size();
  }

  virtual Image *LoadImage(int i) {
      cv::Mat image_cv = cv::imread(filenames_[i], -1), image_cv_float;
      if (image_cv.empty())
      {
        fprintf(stderr, "Failed loading image %d: %s\n", i, filenames_[i].c_str());
        return 0;
      }
      image_cv.convertTo(image_cv_float, CV_32F);
      Image *image = new Image(Mat2Image(image_cv_float));
      return image;
    }

 private:
  std::vector<std::string> filenames_;
};

ImageSequence *ImageSequenceFromFiles(const std::vector<std::string> &filenames,
                                      ImageCache *cache) {
  return new LazyImageSequenceFromFiles(filenames, cache);
}

}  // namespace libmv

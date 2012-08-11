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

#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include "libmv/image/cached_image_sequence.h"
#include "libmv/image/image_sequence_io.h"
#include "testing/testing.h"

using libmv::ImageCache;
using libmv::ImageSequence;
using libmv::ImageSequenceFromFiles;
using std::string;

namespace {

TEST(ImageSequenceIO, FromFiles) {
  cv::Mat_<unsigned char> image1(1, 2);
  cv::Mat_<unsigned char> image2(1, 2);
  image1(0,0) = 1;
  image1(0,1) = 0;
  image2(0,0) = 0;
  image2(0,1) = 1;

  string image1_fn = string(THIS_SOURCE_DIR) + "/1.pgm";
  string image2_fn = string(THIS_SOURCE_DIR) + "/2.pgm";
  cv::imwrite(image1_fn, image1);
  cv::imwrite(image2_fn, image2);

  std::vector<std::string> files;
  files.push_back(image1_fn);
  files.push_back(image2_fn);
  ImageCache cache;
  ImageSequence *sequence = ImageSequenceFromFiles(files, &cache);
  EXPECT_EQ(2, sequence->Length());

  cv::Mat image = sequence->GetImage(0);
  ASSERT_TRUE(!image.empty());
  EXPECT_EQ(2, image.cols);
  EXPECT_EQ(1, image.rows);
  EXPECT_EQ(1, image.channels());
  EXPECT_EQ(image.at<uchar>(0,0), 1.f);
  EXPECT_EQ(image.at<uchar>(0,1), 0.f);

  image = sequence->GetImage(1);
  ASSERT_TRUE(!image.empty());
  EXPECT_EQ(2, image.cols);
  EXPECT_EQ(1, image.rows);
  EXPECT_EQ(1, image.channels());
  EXPECT_EQ(image.at<uchar>(0,0), 0.f);
  EXPECT_EQ(image.at<uchar>(0,1), 1.f);

  sequence->Unpin(0);
  sequence->Unpin(1);

  unlink(image1_fn.c_str());
  unlink(image2_fn.c_str());
}

}  // namespace

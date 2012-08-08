// Copyright (c) 2007, 2008, 2009 libmv authors.
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

#include <algorithm>
#include <iostream>
#include <string>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "libmv/tools/tool.h"

using namespace libmv;
using namespace std;

void
usage()
{
  LOG(ERROR)<< " interest_points ImageNameIn.pgm ImageNameOut.pgm " <<std::endl
  << " ImageNameIn.pgm  : the input image on which surf features will be extrated, " << std::endl
  << " ImageNameOut.pgm : the surf keypoints will be displayed on it. " << std::endl
  << " INFO : work with pgm image only." << std::endl;
}

int
main(int argc, char **argv)
{
  libmv::Init("Extract surf feature from an image", &argc, &argv);

  if (argc != 3)
  {
    usage();
    LOG(ERROR)<< "Missing parameters or errors in the command line.";
    return 1;
  }

  // Parse input parameter
  const string sImageIn = argv[1];
  const string sImageOut = argv[2];

  cv::Mat imageIn = cv::imread(sImageIn, 0);
  if (imageIn.empty())
  {
    LOG(FATAL)<< "Failed loading image: " << sImageIn;
    return 0;
  }

  cv::SURF surf;
  std::vector<cv::KeyPoint> keypoints;
  surf(imageIn, cv::Mat(), keypoints);
  cv::Mat image;
  cv::drawKeypoints(imageIn, keypoints, image);

  cv::waitKey(0);
  cv::imwrite(sImageOut, image);

  return 0;
}

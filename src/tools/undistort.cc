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
//
// Undistort is a tool for undistorting images using known lens distortion 
// coefficients. It supports radial (up to 5 coef.) and tangential distortions
// (up to 2 coef).
// Undistorted images are saved in the same location as input images, with a
// suffix "_undist" placed at the end of the file name.
// Note: All images must have the same size.
// 
// We use the Brown's distortion model
// Variables:
//   (x,y): 2D point in the image (pixel)
//   (u,v): the undistorted 2D point (pixel)
//   radial_distortion (k1, k2, k3, ...): vector containing the
//   radial distortion
//   tangential_distortion_ (p1, p2): vector containing the
//                                    tangential distortion
//   (cx,cy): camera principal point
//
// Equation:
//  u = x + (x - cx) * (k1 * r^2 + k2 * r^4 +...)
//    + (p1(r^2 + 2(x-cx)^2) + 2p2(x-cx)(y-cy))(1 + p3*r^2 +...)
//  v = y + (y - cy) * (k1 * r^2 + k2 * r^4 +...)
//   + (p2(r^2 + 2(y-cy)^2) + 2p1(x-cx)(y-cy))(1 + p3*r^2 +...)

#include <algorithm>
#include <string>
#include <vector>

#include <opencv2/imgproc/imgproc.hpp>

#include "libmv/base/scoped_ptr.h"
#include "libmv/image/cached_image_sequence.h"
#include "libmv/image/image_converter.h"
#include "libmv/image/image_sequence_io.h"
#include "libmv/logging/logging.h"

DEFINE_double(k1, 0,  "Radial distortion coefficient k1 (Brown's model)");
DEFINE_double(k2, 0,  "Radial distortion coefficient k2 (Brown's model)");
DEFINE_double(k3, 0,  "Radial distortion coefficient k3 (Brown's model)");
DEFINE_double(k4, 0,  "Radial distortion coefficient k4 (Brown's model)");
DEFINE_double(k5, 0,  "Radial distortion coefficient k5 (Brown's model)");

DEFINE_double(p1, 0,  "Tangential distortion coefficient p1 (Brown's model)");
DEFINE_double(p2, 0,  "Tangential distortion coefficient p2 (Brown's model)");

DEFINE_double(fx, 0,  "Focal x (in px)");
DEFINE_double(fy, 0,  "Focal y (in px, default: fx)");
DEFINE_double(u0, 0,  "Principal Point u0 (in px, default: width/2)");
DEFINE_double(v0, 0,  "Principal Point v0 (in px, default: height/2)");
DEFINE_double(sk, 0,  "Skew factor");

DEFINE_string(of, "",         "Output folder.");
DEFINE_string(os, "_undist",  "Output file suffix.");

using namespace libmv;

/// TODO(julien) Put this somewhere else...
std::string ReplaceFolder(const std::string &s,
                          const std::string &new_folder) {
  std::string so = s;
  std::string nf = new_folder;
  if (new_folder == "")
    return so;
  
#ifdef WIN32
  size_t n = so.rfind("\\");
  if (n == std::string::npos)
    n = so.rfind("/");
  if (nf.rfind("\\") != nf.size()-1)
    nf.append("\\");
#else
  size_t n = so.rfind("/");
  if (nf.rfind("/") != nf.size()-1)
    nf.append("/");
#endif
    
  if (n != std::string::npos) {
    so.replace(0, n+1, nf);
  } else {
    so = nf; 
    so.append(s);
  }
  return so;
}

int main(int argc, char **argv) {
  std::string usage ="Undistort images using know distortion parameters.\n";
  usage += "Usage: " + std::string(argv[0]) + " IMAGE1 [IMAGE2 ... IMAGEN] ";
  usage += "[-k1 DOUBLE [-k2 DOUBLE] ... [-p1 DOUBLE [-p2 DOUBLE]]] ";
  usage += "-fx DOUBLE -fy DOUBLE [-u0 DOUBLE -v0 DOUBLE] [-sk DOUBLE]]\n";
  usage += "\t * IMAGEX is an image {PNG, PNM, JPEG}\n";
  google::SetUsageMessage(usage);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // This is not the place for this. I am experimenting with what sort of API
  // will be convenient for the tracking base classes.
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    files.push_back(argv[i]);
  }
  
  std::vector<float> distortion_parameters(8);
  distortion_parameters[0] = FLAGS_k1;
  distortion_parameters[1] = FLAGS_k2;
  distortion_parameters[2] = FLAGS_p1;
  distortion_parameters[3] = FLAGS_p2;
  distortion_parameters[4] = FLAGS_k3;
  distortion_parameters[5] = FLAGS_k4;
  distortion_parameters[6] = FLAGS_k5;

  cv::Mat image;
  cv::Mat image_out;
  ImageCache cache;

  scoped_ptr<ImageSequence> source(ImageSequenceFromFiles(files, &cache));
  for (size_t i = 0; i < files.size(); ++i) {
    Image2Mat(source->GetFloatImage(i), image);
    if (!image.empty())
    {
      cv::Mat_<float> K;
      if (FLAGS_u0 == 0)
        FLAGS_u0 = image.cols / 2 - 0.5;
      if (FLAGS_v0 == 0)
        FLAGS_v0 = image.rows / 2 - 0.5;
      if (FLAGS_fy == 0)
        FLAGS_fy = FLAGS_fx;
      K = (cv::Mat_<float>(3, 3) << FLAGS_fx, FLAGS_sk, FLAGS_u0, 0, FLAGS_fy, FLAGS_v0, 0, 0, 1);
      cv::undistort(image, image_out, K, distortion_parameters);
      VLOG(0) << "Undistorting image " << i << "...[DONE]." << std::endl;
    }
    source->Unpin(i);
    // Write the output image
    VLOG(0) << "Saving undistorted image." << std::endl;
    std::stringstream s;
    s << ReplaceFolder(files[i].substr(0, files[i].rfind(".")), FLAGS_of);
    s << FLAGS_os;
    s << files[i].substr(files[i].rfind("."), files[i].size());
    cv::imwrite(s.str(), image_out);
  }
  return 0;
}

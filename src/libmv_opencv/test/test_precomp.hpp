#ifdef __GNUC__
#  pragma GCC diagnostic ignored "-Wmissing-declarations"
#endif

#ifndef __OPENCV_TEST_PRECOMP_HPP__
#define __OPENCV_TEST_PRECOMP_HPP__

#include <opencv2/sfm/sfm.hpp>
#include <opencv2/ts/ts.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace cv;
using namespace std;

/*Read test data from YAML file*/
void
readtestdata(string filename, int nviews, int npts, std::vector<std::vector<cv::Point2d> > &points2d);

#endif

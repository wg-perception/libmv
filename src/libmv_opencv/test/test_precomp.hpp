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

/*Read 2d point data from YAML file*/
void
readtestdata(string filename, int nviews, int npts, std::vector<std::vector<cv::Point2d> > &points2d);

/*Read projection matrix data from YAML file*/
void
readtestdata(string filename, int nviews, std::vector<cv::Mat> &projection_matrices);

#endif

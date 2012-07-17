#ifdef __GNUC__
#  pragma GCC diagnostic ignored "-Wmissing-declarations"
#endif

#ifndef __OPENCV_TEST_PRECOMP_HPP__
#define __OPENCV_TEST_PRECOMP_HPP__

#include <opencv2/sfm/sfm.hpp>
#include <opencv2/ts/ts.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

#include "scene.h"

#define OPEN_TESTFILE(FNAME,FS)  \
      FS.open(FNAME, FileStorage::READ); \
    if (!FS.isOpened())\
    {\
        std::cerr << "Cannot find file: " << FNAME << std::endl;\
        return;\
    }

namespace cvtest
{

  /*Read 2d point data from YAML file*/
  void
  readtestdata(string filename, int nviews, int npts, std::vector<cv::Mat> &points2d);

  /*Read projection matrix data from YAML file*/
  void
  readtestdata(std::string filename, int nviews, std::vector<cv::Mat> &projection_matrices);

  /*Read 3D point data from YAML file*/
  void
  readtestdata(string filename, cv::Mat &points3d);

}

/* namespace cvtest */

#endif

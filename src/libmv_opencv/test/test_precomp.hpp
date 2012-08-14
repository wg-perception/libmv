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

#define EXPECT_MATRIX_NEAR(a, b, tolerance) \
do { \
  bool dims_match = (a.rows == b.rows) && (a.cols == b.cols); \
  EXPECT_EQ(a.rows, b.rows) << "Matrix rows don't match."; \
  EXPECT_EQ(a.cols, b.cols) << "Matrix cols don't match."; \
  if (dims_match) { \
    for (int r = 0; r < a.rows; ++r) { \
      for (int c = 0; c < a.cols; ++c) { \
        EXPECT_NEAR(a(r, c), b(r, c), tolerance) \
          << "r=" << r << ", c=" << c << "."; \
      } \
    } \
  } \
} while(false);

// Unless we have a smarter way to do this we can have different
// macros for different Mat types e.g. CV_64F, CV_64F3, etc as
// needed.
#define EXPECT_MATRIX_CV_64F_NEAR(a, b, tolerance) \
do { \
  EXPECT_EQ(a.type(), CV_64F); \
  EXPECT_EQ(b.type(), CV_64F); \
  bool dims_match = (a.rows == b.rows) && (a.cols == b.cols); \
  EXPECT_EQ(a.rows, b.rows) << "Matrix rows don't match."; \
  EXPECT_EQ(a.cols, b.cols) << "Matrix cols don't match."; \
  if (dims_match) { \
    for (int r = 0; r < a.rows; ++r) { \
      for (int c = 0; c < a.cols; ++c) { \
        EXPECT_NEAR(a.at<double>(r, c), b.at<double>(r, c), tolerance) \
          << "r=" << r << ", c=" << c << "."; \
      } \
    } \
  } \
} while(false);

namespace cvtest
{

  struct TwoViewDataSet
  {
    Mat K1, K2; // Internal parameters
    Mat R1, R2; // Rotation
    Mat t1, t2; // Translation
    Mat P1, P2; // Projection matrix, P = K(R|t)
    Mat F; // Fundamental matrix
    Mat X; // 3D points
    Mat x1, x2; // Projected points
  };

  void
  generateTwoViewRandomScene(TwoViewDataSet &data, int depth = CV_64F);

  /**
   * 2D tracked points
   * -----------------
   *
   * The format is:
   *
   * row1 : x1 y1 x2 y2 ... x36 y36 for track 1
   * row2 : x1 y1 x2 y2 ... x36 y36 for track 2
   * etc
   *
   * i.e. a row gives the 2D measured position of a point as it is tracked
   * through frames 1 to 36.  If there is no match found in a view then x
   * and y are -1.
   *
   * Each row corresponds to a different point.
   *
   */
  void
  parser_2D_tracks(const string &filename, libmv::Tracks &tracks);

} // namespace cvtest

#endif

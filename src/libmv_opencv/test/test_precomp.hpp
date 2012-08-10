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
  parser_2D_tracks( const string &filename, libmv::Tracks &tracks );

} // namespace cvtest

#endif

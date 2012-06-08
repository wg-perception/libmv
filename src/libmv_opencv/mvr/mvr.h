/*M///////////////////////////////////////////////////////////////////////////////////////
 //
 //  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
 //
 //  By downloading, copying, installing or using the software you agree to this license.
 //  If you do not agree to this license, do not download, install,
 //  copy or use the software.
 //
 //
 //                           License Agreement
 //                For Open Source Computer Vision Library
 //
 // Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
 // Copyright (C) 2009-2010, Willow Garage Inc., all rights reserved.
 // Third party copyrights are property of their respective owners.
 //
 // Redistribution and use in source and binary forms, with or without modification,
 // are permitted provided that the following conditions are met:
 //
 //   * Redistribution's of source code must retain the above copyright notice,
 //     this list of conditions and the following disclaimer.
 //
 //   * Redistribution's in binary form must reproduce the above copyright notice,
 //     this list of conditions and the following disclaimer in the documentation
 //     and/or other materials provided with the distribution.
 //
 //   * The name of the copyright holders may not be used to endorse or promote products
 //     derived from this software without specific prior written permission.
 //
 // This software is provided by the copyright holders and contributors "as is" and
 // any express or implied warranties, including, but not limited to, the implied
 // warranties of merchantability and fitness for a particular purpose are disclaimed.
 // In no event shall the Intel Corporation or contributors be liable for any direct,
 // indirect, incidental, special, exemplary, or consequential damages
 // (including, but not limited to, procurement of substitute goods or services;
 // loss of use, data, or profits; or business interruption) however caused
 // and on any theory of liability, whether in contract, strict liability,
 // or tort (including negligence or otherwise) arising in any way out of
 // the use of this software, even if advised of the possibility of such damage.
 //
 //M*/

#ifndef MVR_H
#define MVR_H

//    Multiview Reconstruction Functions
//
//    WIP
//    Should be significantly cleaned up and packaged later
//
// Related to Vincents email thread ...
//    - get a noiseless, outlier free, no missing entry function for multiview orthographic
//    and projective. For orthographic, Tomasi-Kanade has to be implemented (if not in libmv)
//    , and for projective, start by counting the number of points/views (for below 2, libmv
//    has stuff, not sure of what they implement for the rest)
//
//
//TODO: Check idea/realisic funcs - may have mixed up
//TODO: Complete overall skeleton
//TODO: Convert data types to opencv
//Combine nview and 2 view to triangulatePoints() - unit tests - visualise???

//  Triangulate known 2D points and camera matrices from several views
//  to obtain the 3D points
//
// void
// triangulatePoints(InputArrayOfArrays projMatrices, InputArrayOfArrays points,
//    OutputArray points, bool isIdeal)
//
//    TODO: Obtain fundamental matrices from data points -- libmv??

//Todo: Do something about relative paths later
#include "../../libmv/logging/logging.h"
#include "../../libmv/base/vector.h"
#include "../../libmv/multiview/fundamental.h"
#include "../../libmv/multiview/projection.h"
#include "../../libmv/multiview/test_data_sets.h"
#include "../../libmv/numeric/numeric.h"
#include "../../libmv/multiview/twoviewtriangulation.h"
#include "../../libmv/multiview/nviewtriangulation.h"

//remove later
#include<iostream>

namespace libmv_opencv
{

  using namespace libmv;

  // TODO:
//  reconstruct3d(InputArrayOfArrays points, OutputArray points, bool isProj)
//  libmv funcs:
//  ../src/libmv/simple_pipeline/initialize_reconstruction.cc
//  ../src/libmv/simple_pipeline/uncalibrated_reconstructor.cc
//  ../src/libmv/simple_pipeline/reconstruction.cc
//  ../src/libmv/reconstruction/reconstruction_test.cc
//  ../src/libmv/reconstruction/projective_reconstruction.cc
//  ../src/libmv/reconstruction/euclidean_reconstruction.cc
//  ../src/libmv/reconstruction/reconstruction.cc
//  ../src/libmv/reconstruction/euclidean_reconstruction_test.cc
  void
  reconstruct3d(bool isProj, bool isOrtho)
  {

    // Projective reconstruction - Uncalibrated Cameras
    if (isProj)
      {
        // 2 view - libmv code exists - no need?
        // n view - libmv code exists
      }

    // Eucledian reconstruction - Calibrated Cameras
    else
      {
        // 2 view - libmv code exists - no need?
        // n view
      }

    // Orthographic cameras ?
  }

  // TODO: Test this
  // Triangulates a single 3D using 2D points from 2 or more views
  // Returns 3D point in euclidean coordinates
  template<typename T>
    void
    triangulatePoints(const Matrix<T, 2, Dynamic> &xs,
        const vector<Matrix<T, 3, 4> > &Ps, Matrix<T, 3, 1> *X, bool isIdeal)
    {
      Vec4 X_homogenous;
      triangulatePoints(xs, Ps, &X_homogenous, isIdeal);
      HomogeneousToEuclidean(X_homogenous, &X);
    }

//
// Triangulates a single 3D using 2D points from 2 or more views
// Returns 3D point in homogeneous coordinates
//
//  libmv functions:
//  ../src/libmv/multiview/triangulation.cc             -- DLT Triang. // HZ 12.2 pag.312
//  ../src/libmv/multiview/nviewtriangulation.cc        -- nview triang (DONE)
//  ../src/libmv/multiview/twoviewtriangulation.cc      -- 2 view - ideal and by planes??
  template<typename T>
    void
    triangulatePoints(const Matrix<T, 2, Dynamic> &xs,
        const vector<Matrix<T, 3, 4> > &Ps, Matrix<T, 4, 1> *X, bool isIdeal)
    {

      // Get number of views
      int nviews = xs.cols();

      // Should have a 'P' for each view
      assert(Ps.size()==nviews);

      //Need at least 2 views
      assert(nviews>2);

      //
      //  Two view case
      //

      if (0)
//  TODO:- no need?
//  if (nviews == 2)
//  OK This is a bit tricky because the libmv Two View Functions
//  expect the essential matrix between the two cameras.
//  This pattern is different from the nview case which does not
//  expect this.
//  What do we do?? Do we need this here at all???

        {

          if (isIdeal)
          //
          // Ideal points
          //

            {
              // TODO:
//              void TwoViewTriangulationIdeal(const Vec3 &x1, const Vec3 &x2,
//              const Mat34 &P, const Mat3 &E,
//              Vec4 *X){X);
            }

          else

          // Realistic points

            {
              // TODO:
              //          void
              //          TwoViewTriangulationByPlanes(const Vec3 &x1, const Vec3 &x2,
              //              const Mat34 &P, const Mat3 &E, Vec4 *X);
            }
        }

      else

      //
      //  Multi view (>2) case
      //

        {

          // Ideal points
          if (isIdeal)
            {
              //TODO:
            }

          // Realistic points
          else
            {
              NViewTriangulate(xs, Ps, X);

              // Do we need this one?
              //  void NViewTriangulateAlgebraic(const Matrix<T, 2, Dynamic> &x,
              //                                 const vector<Matrix<T, 3, 4> > &Ps,
              //                                 Matrix<T, 4, 1> *X)
              //        What about this method?? No unit test in libmv
              //        TEST(NViewTriangulateAlgebraic, FiveViews) {
              //            uses     NViewTriangulate(xs, Ps, &X);
              //             and not NViewTriangulateAlgebraic()????
            }
        }
    } // triangulatePoints

}
#endif  // MVR_H

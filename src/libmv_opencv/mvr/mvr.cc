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
#include "mvr.h"
namespace libmv_opencv
{
  void test()
  {

  }

//  Triangulate known 2D points and camera matrices from several views
//  to obtain the 3D points
//
// void
// triangulatePoints(InputArrayOfArrays projMatrices, InputArrayOfArrays points,
//    OutputArray points, bool isIdeal)
//
//Only 2 view case directly using libmv function and eigen structs
// Todo: Convert to opencv values and handle Vec3 data
  void
  triangulatePoints(const Vec2 &x1, const Vec2 &x2, const Mat34 &P,
      const Mat3 &E, Vec3 *X, bool isIdeal)
  {
//  Two view
    if (1)
      {
        // Ideal points
        if (isIdeal)
          {
            /**
             * The same algorithm as above generalized for ideal points,
             * e.i. where x1*E*x2' = 0. This will not work if the points are
             * not ideal. In the case of measured image points it is best to
             * either use the TwoViewTriangulationByPlanes function or correct
             * the points so that they lay on the corresponding epipolar lines.
             *
             * \param x1 The normalized image point in the first camera
             *          (inv(K1)*x1_image)
             * \param x2 The normalized image point in the second camera
             *           (inv(K2)*x2_image)
             * \param P  The second camera matrix in the form [R|t]
             * \param E  The essential matrix between the two cameras
             * \param X  The 3D homogeneous point
             */

            //TODO: Workout Get P and E - or just take as args to this func?
            //TODO: handle both 3 vecs
            TwoViewTriangulationIdeal(x1, x2, P, E, X);
//void          TwoViewTriangulationIdeal(const Vec2 &x1, const Vec2 &x2,
//              const Mat34 &P, const Mat3 &E, Vec3 *X);

          }

        // Measured points
        else
          {
            /**
             * Two view triangulation for cameras in canonical form,
             * where the reference camera is in the form [I|0] and P is in
             * the form [R|t]. The algorithm minimizes the re-projection error
             * in the first image only, i.e. the error in the second image is 0
             * while the point in the first image is the point lying on the
             * epipolar line that is closest to x1.
             *
             * \param x1 The normalized image point in the first camera
             *          (inv(K1)*x1_image)
             * \param x2 The normalized image point in the second camera
             *           (inv(K2)*x2_image)
             * \param P  The second camera matrix in the form [R|t]
             * \param E  The essential matrix between the two cameras
             * \param X  The 3D homogeneous point
             *
             * This is the algorithm described in Appendix A in:
             * "An efficient solution to the five-point relative pose problem",
             * by D. Nist\'er, IEEE PAMI, vol. 26
             */
//          void
//          TwoViewTriangulationByPlanes(const Vec3 &x1, const Vec3 &x2,
//              const Mat34 &P, const Mat3 &E, Vec4 *X);
//          void
//          TwoViewTriangulationByPlanes(const Vec2 &x1, const Vec2 &x2,
//              const Mat34 &P, const Mat3 &E, Vec3 *X);
          }
      }

//  More than two views
    else
      {
        // TODO: this doesnt look right?
//      void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
//                          const Mat34 &P2, const Vec2 &x2,
//                          Vec4 *X_homogeneous);
//
//      void TriangulateDLT(const Mat34 &P1, const Vec2 &x1,
//                          const Mat34 &P2, const Vec2 &x2,
//                          Vec3 *X_euclidean);
      }
  }

// TODO:
#if 0
//
// The above masking outliers
//
void
triangulatePointsMask(InputArrayOfArrays projMatrices, InputArrayOfArrays masks,
    InputArrayOfArrays points, OutputArray points);

//
// Reconstructs 3d points from a series of 2d points from multiple views
//
void
reconstruct3d(InputArrayOfArrays points, OutputArray points, bool isProj,
    bool isCalib)

  {

//  get camera matrices
//  triangulate points

//  Projective camera
    if (isProj)
      {

        // Calibrated
        if (isCalib)
          {
            // TODO
          }

        // Uncalibrated
        else
          {

            //  Two views only
            if (1)
              {
//ReconstructFromTwoUncalibratedViews            (const Matches &matches,
//                CameraID image_id1,
//                CameraID image_id2,
//                Matches *matches_inliers,
//                Reconstruction *reconstruction);
              }

            //  More than two views
            else
              {
                // TODO

              }

          }

      }

//  Orthographic camera
    else
      {
        // Calibrated
        if (isCalib)
          {
            // TODO
          }

        // Uncalibrated
        else
          {

            //  Two views only
            if (1)
              {

//              ReconstructFromTwoUncalibratedViews(const Matches &matches,
//                CameraID image_id1,
//                CameraID image_id2,
//                Matches *matches_inliers,
//                Reconstruction *reconstruction);
              }

            //  More than two views
            else
              {
                // TODO

              }

          }
      }

// TODO: libmv has euclidean_reconstruction.cc --- do we need this?
  }
#endif

}

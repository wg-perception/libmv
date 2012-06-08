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
}

#endif


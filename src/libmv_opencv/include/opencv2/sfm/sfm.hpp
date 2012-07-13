/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __OPENCV_SFM_HPP__
#define __OPENCV_SFM_HPP__

#ifdef __cplusplus

#include <opencv2/core/core.hpp>

namespace cv
{

  /** Triangulates enum */
  enum
  {
      CV_TRIANG_DLT = 0,         /*!< HZ 12.2 pag.312 */
      CV_TRIANG_ALGEBRAIC = 1,   /*!< ... */
      CV_TRIANG_BY_PLANE = 2,
  };

  
  /** Triangulates the 3d position of 2d correspondences between several images
   * @param points2d a vector of vectors of 2d points (the inner vector is per image)
   * @param projection_matrices The 3 x 4 projections matrices of each image
   * @param points3d the 3d points
   * @param has_outliers if true, the correspondences are not trusted
   */
  CV_EXPORTS
  void
  triangulatePoints(InputArrayOfArrays points2d, InputArrayOfArrays projection_matrices,
                    OutputArray points3d, int method = CV_TRIANG_DLT);

  /** Reconstruct 3d points from 2d correspondences without performing autocalibration.
   * @param points2d a vector of vectors of 2d points (the inner vector is per image)
   * @param projection_matrices The 3 x 4 projections matrices of each image
   * @param points3d the 3d points
   * @param is_projective if true, the cameras are supposed to be projective
   * @param has_outliers if true, the correspondences are not trusted
   * @param is_sequence if true, the data is assumed to be from tracking in a video sequence
   */
  CV_EXPORTS
  void
  reconstruct(InputArrayOfArrays points2d, OutputArrayOfArrays projection_matrices, OutputArray points3d,
              bool is_projective = false, bool has_outliers = false, bool is_sequence = false);

  /** Reconstruct 3d points from 2d correspondences while performing autocalibration.
   * @param points2d a vector of vectors of 2d points (the inner vector is per image)
   * @param Rs The 3 x 3 rotations of the camera
   * @param Ts The 3 x 1 translations of the camera
   * @param K The intrinsic parameters of the camera
   * @param points3d the 3d points
   * @param is_projective if true, the cameras are supposed to be projective
   * @param has_outliers if true, the correspondences are not trusted
   * @param is_sequence if true, the data is assumed to be from tracking in a video sequence
   */
  CV_EXPORTS
  void
  reconstruct(InputArrayOfArrays points2d, OutputArrayOfArrays Rs, OutputArrayOfArrays Ts, OutputArray K,
              OutputArray points3d, bool is_projective = false, bool has_outliers = false, bool is_sequence =
                  false);

  /** Converts point coordinates from homogeneous to euclidean pixel coordinates. E.g., ((x,y,z)->(x/z, y/z))
   * @param src Input vector of N-dimensional points
   * @param dst Output vector of N-1-dimensional points.
   */
  CV_EXPORTS
  void
  HomogeneousToEuclidean(InputArray src, OutputArray dst);

  /** Converts points from Euclidean to homogeneous space. E.g., ((x,y)->(x,y,1))
   * @param src Input vector of N-dimensional points
   * @param dst Output vector of N+1-dimensional points.
   */
  CV_EXPORTS
  void
  EuclideanToHomogeneous(InputArray src, OutputArray dst);
  
  /** This function normalizes points as done in the eight point algorithm
   * @param X Input vector of N-dimensional points
   * @param x Output vector of the same N-dimensional points but with mean 0 and average norm sqrt(2)
   * @param T Output transform matrix such that x = T*X
   */
  CV_EXPORTS
  void
  IsotropicScaling(InputArray X, OutputArray x, OutputArray T);
} /* namespace cv */

#endif /* __cplusplus */

#endif

/* End of file. */

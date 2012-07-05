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

#include "test_precomp.hpp"

TEST(Sfm_reconstruct, twoViewProjective)
{

  std::vector<std::vector<cv::Point2d> > points2d;
  std::vector<cv::Point3d> points3d;
  std::vector<cv::Mat> projection_matrices;
  std::vector<cv::Mat> estimated_projection_matrices;

  string filename(cvtest::TS::ptr()->get_data_path() + "sfm/rnd_N10_F3.yml");
  readtestdata(filename, 2, 10, points2d);
  readtestdata(filename,2,projection_matrices);

  cv::reconstruct(points2d, estimated_projection_matrices, points3d, true);

  cout << projection_matrices[0] << endl;
  cout << projection_matrices[1] << endl;

}

//  std::vector<cv::Point2d> pts;
//  pts = points2d[0];
//  cout << pts << endl;

//  OutputArray projection_matrix_outarray(projection_matrices);
//
//  assert(projection_matrix_outarray.kind() == _OutputArray::STD_VECTOR_MAT);
//
//
//cv::reconstruct(points2d, projection_matrix_outarray, points3d, true);


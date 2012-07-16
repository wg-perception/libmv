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

using namespace cv;
using namespace std;
using namespace cvtest;

/* Check projection errors */
void
check_projection_errors(Mat& points3d_estimated, vector<Mat>& projection_matrices_estimated, vector<Mat>& points2d)
{
  for (int m = 0; m < points2d.size(); ++m)
  {

    Mat X, x;
    EuclideanToHomogeneous(points3d_estimated, X); // 3D point
    HomogeneousToEuclidean(projection_matrices_estimated[m] * X, x); // 2d projection
    Mat projerr = points2d[m] - x;
//    cout << projerr << endl;
    for (int n = 0; n < 10; ++n)
    {
      double d = sqrt(pow(projerr.at<double>(0, n), 2) + pow(projerr.at<double>(1, n), 2));
      EXPECT_NEAR(0, d, 1e-4);
    }
  }
}

TEST(Sfm_reconstruct, twoViewProjective)
{
  Mat points3d;
  Mat points3d_estimated;
  vector<Mat> projection_matrices;
  vector<Mat> projection_matrices_estimated;
  vector<Mat> points2d;

  string filename(cvtest::TS::ptr()->get_data_path() + "sfm/rnd_N10_F3_Proj1.yml");
//  cout << "Test data: " << filename << endl;

  readtestdata(filename, 2, 10, points2d);
  readtestdata(filename, 2, projection_matrices);
  readtestdata(filename, points3d);
  CV_Assert(points3d.cols == 10);

  reconstruct(points2d, projection_matrices_estimated, points3d_estimated, true);

  /*  Check projection errors on GT*/
  check_projection_errors(points3d, projection_matrices, points2d);

  /*  Check projection errors on estimates*/
  // should this work for the  projective case??
  check_projection_errors(points3d_estimated, projection_matrices_estimated, points2d);
}

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

TEST(Sfm_reconstruct, twoViewProjective)
{
  vector<Point3d> points3d;
  vector<Point3d> points3d_estimated;
  vector<Mat> projection_matrices;
  vector<Mat> projection_matrices_estimated;
  vector<vector<Point2d> > points2d;

  string filename(cvtest::TS::ptr()->get_data_path() + "sfm/rnd_N10_F3.yml");

  cout << "Test data: " << filename << endl;
  readtestdata(filename, 2, 10, points2d);
  readtestdata(filename, 2, projection_matrices);
  readtestdata(filename, points3d);
  CV_Assert(points3d.size()==10);

  cout << "Ground truth 3D Points:" << endl;
  for (int n = 0; n < points3d.size(); ++n)
    cout << points3d[n] << endl;

  reconstruct(points2d, projection_matrices_estimated, points3d_estimated, true);

  /*
  cout << "Groundtruth Projection Matrices:" << endl;
  cout << projection_matrices[0] << endl;
  cout << projection_matrices[1] << endl;
  cout << "Estimate Projection Matrices :" << endl;
  cout << projection_matrices_estimated[0] << endl;
  cout << projection_matrices_estimated[1] << endl;
  cout << "Not necessarily equal.Better to check diff in x=PX" << endl;
  cout << "Ground truth 3D Points:" << endl;
  for (int n = 0; n < points3d.size(); ++n)
    cout << points3d[n] << endl;

  cout << "Estimted 3D Points:" << endl;
  for (int n = 0; n < points3d.size(); ++n)
    cout << points3d_estimated[n] << endl;
*/

  /* Check x=PX -- improve */
/*  vector<Point2d> v1pts2d = points2d[0];
  for (int n = 0; n < points3d.size(); ++n)
  {
    cout << "View 1Projection error for pt " << n << ": "
         << v1pts2d[n] - projection_matrices_estimated[0] * points3d_estimated[n];
  }*/

}

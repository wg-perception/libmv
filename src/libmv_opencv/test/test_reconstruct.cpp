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
check_projection_errors(const Mat& points3d_estimated, const vector<Mat>& projection_matrices_estimated,
                        const vector<Mat>& points2d)
{
  Mat X;
  EuclideanToHomogeneous(points3d_estimated, X); // 3D point
  for (int m = 0; m < points2d.size(); ++m)
  {
    Mat x;
    HomogeneousToEuclidean(projection_matrices_estimated[m] * X, x); // 2d projection
    Mat projerr = points2d[m] - x;
//    cout << projerr << endl;
    for (int n = 0; n < projerr.cols; ++n)
    {
      double d = cv::norm(projerr.col(n));
      EXPECT_NEAR(0, d, 1e-4);
    }
  }
}

TEST(Sfm_reconstruct, twoViewProjective)
{
  int nviews = 2;
  int npoints = 50;
  bool is_projective = true;

  for(unsigned iter =0;iter<2;++iter)
  {
    int depth;
    float err_max2d, err_max3d;
    if (iter==0)
    {
      depth = CV_32F;
      err_max2d = 1e-5;
      err_max3d = 1e-9;
    }
    else
    {
      depth = CV_64F;
      err_max2d = 1e-7;
      err_max3d = 1e-9;
    }

    cv::Mat K;
    std::vector<cv::Mat> Rs;
    std::vector<cv::Mat> ts;
    std::vector<cv::Mat> Ps;
    cv::Mat points3d;
    std::vector<cv::Mat> points2d;
    generateScene(nviews, npoints, is_projective, depth, K, Rs, ts, Ps, points3d, points2d);

    Mat points3d_estimated;
    vector<Mat> Ps_estimated;

    reconstruct(points2d, Ps_estimated, points3d_estimated, is_projective);

    /*  Check projection errors on GT*/
    check_projection_errors(points3d, Ps, points2d);

    /*  Check projection errors on estimates*/
    // should this work for the  projective case??
    check_projection_errors(points3d_estimated, Ps_estimated, points2d);
  }
}

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

#include "libmv/multiview/test_data_sets.h"
#include "libmv/numeric/numeric.h"

#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace std;
using namespace cvtest;


TEST(Sfm_triangulate, TriangulateDLT) {
    libmv::TwoViewDataSet d = libmv::TwoRealisticCameras();

    Mat x1, x2;
    Mat P1, P2;

    eigen2cv<double, 2, Eigen::Dynamic>(d.x1, x1);
    eigen2cv<double, 2, Eigen::Dynamic>(d.x2, x2);
    eigen2cv<double,3,4>(d.P1, P1);
    eigen2cv<double,3,4>(d.P2, P2);

    // build x
    vector<Mat> x;
    x.push_back(x1);
    x.push_back(x2);

    // build P
    vector<Mat> P;
    P.push_back(P1);
    P.push_back(P2);

    // get 3d points
    Mat X_estimated;
    triangulatePoints(x, P, X_estimated);

    // check
    for (int i = 0; i < d.X.cols(); ++i)
    {
        libmv::Vec3 X_est, X_gt;

        // get current columns
        libmv::MatrixColumn(d.X, i, &X_gt);
        cv2eigen<double,3,1>(X_estimated.col(i), X_est);

        // Check: || X_est - X_gt ||_{inf} < 1e-8
        EXPECT_NEAR(0, libmv::DistanceLInfinity(X_est, X_gt), 1e-8);
    }
}

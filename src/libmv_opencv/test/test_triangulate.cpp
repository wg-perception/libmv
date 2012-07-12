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

    for (int i = 0; i < d.X.cols(); ++i)
    {
        libmv::Vec2 x1, x2;
        libmv::MatrixColumn(d.x1, i, &x1);
        libmv::MatrixColumn(d.x2, i, &x2);
        libmv::Vec3 X_estimated, X_gt;
        libmv::MatrixColumn(d.X, i, &X_gt);

        // build x
        vector<Mat> x;

        Mat tmp;
        eigen2cv<double,2,1>(x1, tmp);
        x.push_back( tmp.clone() );

        eigen2cv<double,2,1>(x2, tmp);
        x.push_back( tmp.clone() );

        // build P
        vector<Mat> P;
        eigen2cv<double,3,4>(d.P1, tmp);
        P.push_back( tmp.clone() );

        eigen2cv<double,3,4>(d.P2, tmp);
        P.push_back( tmp.clone() );

        // get 3d points
        Mat X_estimated_cv;
        triangulatePoints(x, P, X_estimated_cv);
        cv2eigen<double,3,1>(X_estimated_cv, X_estimated);

        // check
        EXPECT_NEAR(0, libmv::DistanceLInfinity(X_estimated, X_gt), 1e-8);
    }
}

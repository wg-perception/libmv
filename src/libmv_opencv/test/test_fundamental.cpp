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

TEST(Sfm_fundamental, fundamentalFromProjections)
{
    Mat_<double> P1_gt(3,4), P2_gt(3,4);
    P1_gt << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0;

    P2_gt << 1, 1, 1, 3,
             0, 2, 0, 3,
             0, 1, 1, 0;

    Mat F_gt;
    fundamentalFromProjections(P1_gt, P2_gt, F_gt);

    Mat_<double> P1(3,4), P2(3,4);
    projectionsFromFundamental(F_gt, P1, P2);

    Mat F;
    fundamentalFromProjections(P1, P2, F);

    EXPECT_LE( norm(F_gt, F), 1e-6 );
}

TEST(Sfm_fundamental, normalizedEightPointSolver) {
    int nviews = 2;
    int npoints = 8;
    bool is_projective = true;
    int depth = CV_64F;

    Mat K;
    vector<Mat> Rs;
    vector<Mat> ts;
    vector<Mat> Ps;
    Mat points3d;
    vector<Mat> points2d;

    generateScene(nviews, npoints, is_projective, depth, K, Rs, ts, Ps, points3d, points2d);

    Mat F;
    normalizedEightPointSolver(points2d[0], points2d[1], F);

    for(int i=0; i<npoints; ++i)
    {
        Mat x1, x2;

        euclideanToHomogeneous(points2d[0].col(i), x1);
        euclideanToHomogeneous(points2d[1].col(i), x2);

        Mat_<double> value = (x2.t() * F * x1);
        // cout << "x2' * F * x1 = " << value(0,0) << endl;
        EXPECT_LE( value(0,0), 1e-9 );
    }
}

TEST(Sfm_fundamental8Point, correctness)
{

}

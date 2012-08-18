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
using namespace cvtest;
using namespace std;

TEST(Sfm_fundamental, fundamentalFromProjections)
{
    Mat_<double> P1_gt(3, 4), P2_gt(3, 4);
    P1_gt << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;

    P2_gt << 1, 1, 1, 3, 0, 2, 0, 3, 0, 1, 1, 0;

    Mat F_gt;
    fundamentalFromProjections(P1_gt, P2_gt, F_gt);

    Mat_<double> P1(3, 4), P2(3, 4);
    projectionsFromFundamental(F_gt, P1, P2);

    Mat F;
    fundamentalFromProjections(P1, P2, F);

    EXPECT_LE( norm(F_gt, F), 1e-6);
}

TEST(Sfm_fundamental, normalizedEightPointSolver)
{
    cvtest::TwoViewDataSet d;
    generateTwoViewRandomScene<double>( d );

    Mat F;
    normalizedEightPointSolver( d.x1, d.x2, F );
    expectFundamentalProperties<double>( F, d.x1, d.x2 );
}

TEST(Sfm_fundamental, motionFromEssential)
{

    cvtest::TwoViewDataSet d;
    double tol = 1e-9;

//	   generateTwoViewRandomScene( d, CV_32F );
    generateTwoViewRandomScene(d, CV_64F);

    Mat E(3, 3, CV_64F);
//Todo: change this to EssentialFromRt - any diff???
    essentialFromFundamental(d.F, d.K1, d.K2, E);
//     essentialFromRt(d.R1, d.t1, d.R2, d.t2, E);

    Mat R,t;
    relativeCameraMotion( d.R1, d.t1, d.R2, d.t2, R, t );
    cv::normalize(t,t);

    vector<Mat> Rs, ts;
    motionFromEssential(E, Rs, ts);
    bool one_solution_is_correct = false;
    for( int i=0;i<Rs.size(); ++i)
    {
        if((norm(Rs[i],R)<tol) && (norm(ts[i],t)<tol))
        {
            one_solution_is_correct=true;
            break;
        }
    }
    EXPECT_TRUE(one_solution_is_correct);

}

// Combined test for fundamentalFromEssential and essentialFromFundamental
TEST(Sfm_fundamental, fundamentalToAndFromEssential)
{
    cvtest::TwoViewDataSet data;
    double tol = 1e-4;

    //	   generateTwoViewRandomScene( data, CV_32F );
    generateTwoViewRandomScene(data, CV_64F);

    Mat F(3,3,CV_64F), E(3,3,CV_64F);
    essentialFromFundamental(data.F, data.K1, data.K2, E);
    fundamentalFromEssential(E, data.K1, data.K2, F);
    EXPECT_MATRIX_NEAR<double>(data.F, F, tol);
}

TEST(Sfm_fundamental, essentialFromFundamental) 
{
    cvtest::TwoViewDataSet d;
    generateTwoViewRandomScene<double>(d);

    Mat_<double> E_from_Rt(3,3);
    essentialFromRt(d.R1, d.t1, d.R2, d.t2, E_from_Rt);

    Mat_<double> E_from_F(3,3);
    essentialFromFundamental(d.F, d.K1, d.K2, E_from_F);

    EXPECT_MATRIX_PROP(E_from_Rt, E_from_F, 1e-6);
}

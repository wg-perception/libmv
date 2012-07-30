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

#include "opencv2/sfm/fundamental.hpp"

#include "libmv/multiview/robust_fundamental.h"
#include "libmv/multiview/fundamental.h"
#include <opencv2/core/eigen.hpp>

#include <iostream>

namespace cv
{

template<typename T>
void
projectionsFromFundamental( const Mat &_F, Mat &_P1, Mat &_P2 )
{
    libmv::Mat3 F;
    libmv::Mat34 P1, P2;

    cv2eigen( _F, F );

    libmv::ProjectionsFromFundamental( F, &P1, &P2 );

    eigen2cv( P1, _P1 );
    eigen2cv( P2, _P2 );
}

void
projectionsFromFundamental( const Mat &F, Mat &P1, Mat &P2 )
{
    int depth = F.depth();

    if( depth == CV_32F )
    {
//         projectionsFromFundamental<float>( F, P1, P2 );
        std::cerr << "Function projectionsFromFundamental not handled for float" <<std::endl;
    }
    else
    {
        projectionsFromFundamental<double>( F, P1, P2 );
    }
}

template<typename T>
void
fundamentalFromProjections( const Mat &_P1,
                            const Mat &_P2,
                            Mat &_F )
{
    libmv::Mat34 P1, P2;
    libmv::Mat3 F;

    cv2eigen( _P1, P1 );
    cv2eigen( _P2, P2 );

    libmv::FundamentalFromProjections( P1, P2, &F );

    eigen2cv( F, _F );
}

void
fundamentalFromProjections( const Mat &P1,
                            const Mat &P2,
                            Mat &F )
{
    int depth = P1.depth();
    CV_Assert( depth == P2.depth() );

    if( depth == CV_32F )
    {
        // fundamentalFromProjections<float>( P1, P2, F );
        std::cerr << "Function fundamentalFromProjections not handled for float" << std::endl;
    }
    else
    {
        fundamentalFromProjections<double>( P1, P2, F );
    }
}

template<typename T>
void
normalizedEightPointSolver( const Mat &_x1,
                            const Mat &_x2,
                            Mat &_F )
{
    libmv::Mat x1, x2;
    libmv::Mat3 F;

    cv2eigen( _x1, x1 );
    cv2eigen( _x2, x2 );

    libmv::NormalizedEightPointSolver( x1, x2, &F );

    eigen2cv( F, _F );
}

void
normalizedEightPointSolver(const Mat &x1,
                           const Mat &x2,
                           Mat &F)
{
    int depth = x1.depth();
    CV_Assert( depth == x2.depth() );

    if( depth == CV_32F )
    {
//         normalizedEightPointSolver<float>( x1, x2, F );
        std::cerr << "Function normalizedEightPointSolver not handled for float" <<std::endl;
    }
    else
    {
        normalizedEightPointSolver<double>( x1, x2, F );
    }
}



void
fundamental8Point(InputArray _x1, InputArray _x2, OutputArray _F, bool has_outliers)
{
    double max_error = 0.1;

    cv::Mat F(3, 3, CV_64F), T1(3, 3, CV_64F), T2(3, 3, CV_64F);
    cv::Mat x1, x2;
    x1 = _x1.getMat();
    x2 = _x2.getMat();

    // Need at least 8 pts
    CV_Assert(x1.cols >= 8 && x1.cols == x2.cols);
    CV_Assert(x1.depth() == x2.depth());
    int depth = x1.depth();

    // Normalize points
//     normalizeIsotropicPoints(x1, x1, T1);
//     normalizeIsotropicPoints(x2, x2, T2);

    // Compute fundamental matrix
    libmv::vector<int> inliers;
    libmv::Mat x1_, x2_;
    libmv::Mat3 F_;
    cv2eigen(x1, x1_);
    cv2eigen(x2, x2_);

    if (has_outliers)
      FundamentalFromCorrespondences8PointRobust(x1_, x2_, max_error, &F_, &inliers);
    else
      libmv::NormalizedEightPointSolver(x1_, x2_, &F_);
    eigen2cv(F_, F);

    // Denormalized
//     F = T2.t() * F * T1;

    // Pack output
    F.convertTo(_F.getMatRef(), depth);
}

} /* namespace cv */

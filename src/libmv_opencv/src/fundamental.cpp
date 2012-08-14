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

using namespace std;

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
        cerr << "Function projectionsFromFundamental not handled for float" <<endl;
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
        cerr << "Function fundamentalFromProjections not handled for float" << endl;
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
        cerr << "Function normalizedEightPointSolver not handled for float" <<endl;
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


template<typename T>
void
relativeCameraMotion( const Mat &_R1,
                      const Mat &_t1,
                      const Mat &_R2,
                      const Mat &_t2,
                      Mat &_R,
                      Mat &_t )
{
    libmv::Mat3 R1, R2, R;
    libmv::Vec3 t1, t2, t;

    cv2eigen( _R1, R1 );
    cv2eigen( _t1, t1 );
    cv2eigen( _R2, R2 );
    cv2eigen( _t2, t2 );

    libmv::RelativeCameraMotion( R1, t1, R2, t2, &R, &t );

    eigen2cv( R, _R );
    eigen2cv( t, _t );
}

void
relativeCameraMotion( const Mat &R1,
                      const Mat &t1,
                      const Mat &R2,
                      const Mat &t2,
                      Mat &R,
                      Mat &t )
{
    int depth = R1.depth();
    CV_Assert( depth == t1.depth() && depth == R2.depth() && depth == t2.depth() );

    if( depth == CV_32F )
    {
        // relativeCameraMotion<float>( R1, t1, R2, t2, R, t );
        cerr << "Function relativeCameraMotion not handled for float" << endl;
    }
    else
    {
        relativeCameraMotion<double>( R1, t1, R2, t2, R, t );
    }
}


// MotionFromEssential
template<typename T>
void
motionFromEssential( const Mat &_E, vector<Mat> &_Rs, vector<Mat> &_ts )
{
    libmv::Mat3 E;
    vector<libmv::Mat3> Rs;
    vector<libmv::Vec3> ts;

    cv2eigen( _E, E );

    libmv::MotionFromEssential( E, &Rs, &ts );

    _Rs.clear();
    _ts.clear();

    int n = Rs.size();
    CV_Assert( ts.size() == n );
    Mat R_temp, t_temp;

    for ( int i=0; i < n; ++i )
    {
        eigen2cv( Rs[i], R_temp );
        _Rs.push_back( R_temp );

        eigen2cv( ts[i], t_temp );
        _ts.push_back( t_temp );
    }
}

void
motionFromEssential( const Mat &E, vector<Mat> &Rs, vector<Mat> &ts )
{
    int depth = E.depth();
    if( depth == CV_32F )
    {
        // motionFromEssential<float>( E, Rs, ts );
        cerr << "Function motionFromEssential not handled for float" << endl;
    }
    else
    {
        motionFromEssential<double>(E, Rs, ts);
    }
}

// fundamentalFromEssential
template<typename T>
void
fundamentalFromEssential( const Mat &_E, const Mat &_K1, const Mat &_K2, Mat &_F )
{
    libmv::Mat3 E, K1, K2;
    libmv::Mat3 F;

    cv2eigen( _E, E );
    cv2eigen( _K1, K1 );
    cv2eigen( _K2, K2 );

    libmv::FundamentalFromEssential( E, K1, K2, &F );

    eigen2cv( F, _F );
}

void
fundamentalFromEssential( const Mat &E, const Mat &K1, const Mat &K2, Mat &F )
{
    int depth = F.depth();
    CV_Assert( depth == K1.depth() && depth == K2.depth() );

    if( depth == CV_32F )
    {
        // fundamentalFromEssential<float>( E, K1, K2, F );
        cerr << "Function fundamentalFromEssential not handled for float" << endl;
    }
    else
    {
        fundamentalFromEssential<double>( E, K1, K2, F );
    }
}

// essentialFromFundamental
template<typename T>
void
essentialFromFundamental( const Mat &_F, const Mat &_K1, const Mat &_K2, Mat &_E )
{
    libmv::Mat3 F, K1, K2;
    libmv::Mat3 E;

    cv2eigen( _F, F );
    cv2eigen( _K1, K1 );
    cv2eigen( _K2, K2 );

    libmv::EssentialFromFundamental( F, K1, K2, &E );

    eigen2cv( E, _E );
}

void
essentialFromFundamental( const Mat &F, const Mat &K1, const Mat &K2, Mat &E )
{
    int depth = F.depth();
    CV_Assert( depth == K1.depth() && depth == K2.depth() );

    if( depth == CV_32F )
    {
        // essentialFromFundamental<float>( F, K1, K2, E );
        std::cerr << "Function essentialFromFundamental not handled for float" << std::endl;
    }
    else
    {
        essentialFromFundamental<double>( F, K1, K2, E );
    }
}

} /* namespace cv */

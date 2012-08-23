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
#include "opencv2/sfm/numeric.hpp"

#include "libmv/multiview/robust_fundamental.h"
#include "libmv/multiview/fundamental.h"
#include <opencv2/core/eigen.hpp>

#include <iostream>

using namespace std;

namespace cv
{

    template<typename T>
    void
    projectionsFromFundamental( const Mat &F, Mat &_P1, Mat &_P2 )
    {
        Mat_<T> P1(3,4), P2(3,4);
        P1 << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1, 0;

        Mat_<T> e2(3,1);
        cv::SVD::solveZ(F.t(), e2);

        P2(Range(0,3), Range(0, 3)) = skewMat(e2) * F;
        P2(0,3) = e2(0);
        P2(1,3) = e2(1);
        P2(2,3) = e2(2);

        P1.copyTo(_P1);
        P2.copyTo(_P2);
    }

    void
    projectionsFromFundamental( const Mat &F, Mat &P1, Mat &P2 )
    {
        int depth = F.depth();

        if ( depth == CV_32F )
        {
            projectionsFromFundamental<float>(F, P1, P2);
        }
        else
        {
            projectionsFromFundamental<double>(F, P1, P2);
        }
    }

    template<typename T>
    void
    fundamentalFromProjections( const Mat &P1, const Mat &P2, Mat &_F )
    {
        Mat X[3];
        vconcat( P1.row(1), P1.row(2), X[0] );
        vconcat( P1.row(2), P1.row(0), X[1] );
        vconcat( P1.row(0), P1.row(1), X[2] );

        Mat Y[3];
        vconcat( P2.row(1), P2.row(2), Y[0] );
        vconcat( P2.row(2), P2.row(0), Y[1] );
        vconcat( P2.row(0), P2.row(1), Y[2] );

        Mat_<T> F(3,3), XY;
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                vconcat(X[j], Y[i], XY);
                F(i, j) = determinant(XY);
            }
        }
        F.copyTo(_F);
    }

    void
    fundamentalFromProjections( const Mat &P1, const Mat &P2, Mat &F )
    {
        int depth = P1.depth();
        CV_Assert(depth == P2.depth());

        if ( depth == CV_32F )
        {
            fundamentalFromProjections<float>(P1, P2, F);
        }
        else
        {
            fundamentalFromProjections<double>(P1, P2, F);
        }
    }

    template<typename T>
    void
    normalizedEightPointSolver( const Mat &_x1, const Mat &_x2, Mat &_F )
    {
        libmv::Mat x1, x2;
        libmv::Mat3 F;

        cv2eigen(_x1, x1);
        cv2eigen(_x2, x2);

        libmv::NormalizedEightPointSolver(x1, x2, &F);

        eigen2cv(F, _F);
    }

    void
    normalizedEightPointSolver( const Mat &x1, const Mat &x2, Mat &F )
    {
        int depth = x1.depth();
        CV_Assert(depth == x2.depth());

        if ( depth == CV_32F )
        {
//         normalizedEightPointSolver<float>( x1, x2, F );
            cerr << "Function normalizedEightPointSolver not handled for float"
            << endl;
        }
        else
        {
            normalizedEightPointSolver < double >(x1, x2, F);
        }
    }

    template<typename T>
    void
    relativeCameraMotion( const Mat_<T> &R1, const Mat_<T> &t1, const Mat_<T> &R2,
                          const Mat_<T> &t2, Mat &R, Mat &t )
    {
        Mat(R2 * R1.t()).copyTo(R);
        Mat(t2 - R * t1).copyTo(t);
    }

    void
    relativeCameraMotion( const Mat &R1, const Mat &t1, const Mat &R2,
                          const Mat &t2, Mat &R, Mat &t )
    {
        int depth = R1.depth();
        CV_Assert(depth == t1.depth() && depth == R2.depth() && depth == t2.depth());

        // check dimensions
        CV_Assert( R1.rows == 3 && R1.cols == 3 );
        CV_Assert( R2.rows == 3 && R2.cols == 3 );
        CV_Assert( t1.rows == 3 && t1.cols == 1 );
        CV_Assert( t2.rows == 3 && t2.cols == 1 );

        if ( depth == CV_32F )
        {
            relativeCameraMotion < float >(R1, t1, R2, t2, R, t);
        }
        else
        {
            relativeCameraMotion < double >(R1, t1, R2, t2, R, t);
        }
    }

// MotionFromEssential
    template<typename T>
    void
    motionFromEssential( const Mat &_E, vector < Mat > &_Rs,
                         vector < Mat > &_ts )
    {
        libmv::Mat3 E;
        vector < libmv::Mat3 > Rs;
        vector < libmv::Vec3 > ts;

        cv2eigen(_E, E);

        libmv::MotionFromEssential(E, &Rs, &ts);

        _Rs.clear();
        _ts.clear();

        int n = Rs.size();
        CV_Assert(ts.size() == n);

        for ( int i = 0; i < n; ++i )
        {
            Mat R_temp, t_temp;

            eigen2cv(Rs[i], R_temp);
            _Rs.push_back(R_temp);

            eigen2cv(ts[i], t_temp);
            _ts.push_back(t_temp);
        }
    }

    void
    motionFromEssential( const Mat &E, vector < Mat > &Rs, vector < Mat > &ts )
    {
        int depth = E.depth();
        if ( depth == CV_32F )
        {
            // motionFromEssential<float>( E, Rs, ts );
            cerr << "Function motionFromEssential not handled for float"
            << endl;
        }
        else
        {
            motionFromEssential < double >(E, Rs, ts);
        }
    }

// fundamentalFromEssential
    void
    fundamentalFromEssential( const Mat &E, const Mat &K1, const Mat &K2,
                              Mat &F )
    {
        int depth = F.depth();
        CV_Assert(depth == K1.depth() && depth == K2.depth());

        Mat(K2.inv().t() * E * K1.inv()).copyTo(F);
    }

// essentialFromFundamental
    void
    essentialFromFundamental( const Mat &F, const Mat &K1, const Mat &K2,
                              Mat &E )
    {
        int depth = F.depth();
        CV_Assert(depth == K1.depth() && depth == K2.depth());

        Mat(K2.t() * F * K1).copyTo(E);
    }

    template<typename T>
    void
    essentialFromRt( const Mat &_R1,
                    const Mat &_t1,
                    const Mat &_R2,
                    const Mat &_t2,
                    Mat &_E )
    {
        libmv::Mat3 E;
        libmv::Mat3 R1, R2;
        libmv::Vec3 t1, t2;

        cv2eigen( _R1, R1 );
        cv2eigen( _t1, t1 );
        cv2eigen( _R2, R2 );
        cv2eigen( _t2, t2 );

        libmv::EssentialFromRt( R1, t1, R2, t2, &E );

        eigen2cv( E, _E );
    }

    void
    essentialFromRt( const Mat &R1,
                    const Mat &t1,
                    const Mat &R2,
                    const Mat &t2,
                    Mat &E )
    {
        int depth = R1.depth();
        CV_Assert( depth == t1.depth() && depth == R2.depth() && depth == t2.depth() );

        if( depth == CV_32F )
        {
            // essentialFromRt<float>( R1, t1, R2, t2, E );
            std::cerr << "Function essentialFromRt not handled for float" << std::endl;
        }
        else
        {
            essentialFromRt<double>( R1, t1, R2, t2, E );
        }
    }

    // normalizeFundamental
    template<typename T>
    void
    normalizeFundamental( const Mat_<T> &F, Mat &F_normalized )
    {
        Mat( F/norm(F,NORM_L2) ).copyTo( F_normalized );  // Frobenius Norm

        if ( F_normalized.at<T>(2,2) < 0 )
        {
            F_normalized *= -1;
        }
    }

    void
    normalizeFundamental( const Mat &F, Mat &F_normalized )
    {
        int depth = F.depth();
        if( depth == CV_32F )
        {
            normalizeFundamental<float>( F, F_normalized );
        }
        else
        {
            normalizeFundamental<double>( F, F_normalized );
        }
    }


} /* namespace cv */

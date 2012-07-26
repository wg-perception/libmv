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

#include <opencv2/sfm/conditioning.hpp>

#include "libmv/multiview/conditioning.h"
#include <opencv2/core/eigen.hpp>

#include <iostream>

namespace cv
{

template<typename T>
void
preconditionerFromPoints( const Mat &_points,
                          Mat &_Tr )
{
    libmv::Mat points;
    libmv::Mat3 Tr;

    cv2eigen( _points, points );

    libmv::PreconditionerFromPoints( points, &Tr );

    eigen2cv( Tr, _Tr );
}

void
preconditionerFromPoints( const Mat &points,
                          Mat &T )
{
    int depth = points.depth();
    if( depth == CV_32F )
    {
        // preconditionerFromPoints<float>( points, T );
        std::cerr << "Function preconditionerFromPoints not handled for float" << std::endl;
    }
    else
    {
        preconditionerFromPoints<double>( points, T );
    }
}


template<typename T>
void
applyTransformationToPoints( const Mat &_points,
                             const Mat &_T,
                             Mat &_transformed_points )
{
    libmv::Mat points, transformed_points;
    libmv::Mat3 Tr;

    cv2eigen( _points, points );
    cv2eigen( _T, Tr );

    libmv::ApplyTransformationToPoints( points, Tr, &transformed_points );

    eigen2cv( transformed_points, _transformed_points );
}

void
applyTransformationToPoints( const Mat &points,
                             const Mat &T,
                             Mat &transformed_points )
{
    int depth = points.depth();
    CV_Assert( depth == T.depth() );

    if( depth == CV_32F )
    {
        // applyTransformationToPoints<float>( points, T, transformed_points );
        std::cerr << "Function applyTransformationToPoints not handled for float" << std::endl;
    }
    else
    {
        applyTransformationToPoints<double>( points, T, transformed_points );
    }
}


void
IsotropicScaling(const InputArray _X, OutputArray _x, OutputArray _T)
{
    // input
    libmv::Mat X;
    cv2eigen<double>(_X.getMat(), X);

    // normalization
    libmv::Mat x;
    libmv::Mat3 T;
    libmv::NormalizeIsotropicPoints(X, &x, &T);

    // output
    cv::Mat Xcv = _X.getMat();
    _x.create(Xcv.rows, Xcv.cols, CV_64F);
    cv::Mat x_dst = _x.getMat();
    eigen2cv<double>(x, x_dst);

    _T.create(Xcv.rows + 1, Xcv.rows + 1, CV_64F);
    cv::Mat T_dst = _T.getMat();
    eigen2cv<double, 3, 3>(T, T_dst);
}

} /* namespace cv */

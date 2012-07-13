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

#include "libmv/multiview/triangulation.h"
#include "libmv/multiview/twoviewtriangulation.h"
#include "libmv/multiview/fundamental.h"

#include "opencv2/sfm/sfm.hpp"
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace std;

namespace cv
{

template<typename T>
void
triangulateDLT( const Mat &_xl, const Mat &_xr, const Mat &_Pl, const Mat &_Pr, Mat &points3d)
// triangulateDLT( const Matx<T,2,1> &_xl, const Matx<T,2,1> &_xr, const Matx<T,3,4> &_Pl, const Matx<T,3,4> &_Pr, Matx<T,3,1> &points3d)
// triangulateDLT( InputArray _xl, InputArray _xr, InputArray _Pl, InputArray _Pr, OutputArray points3d)
{
    // src
    Eigen::Matrix<T, 2, 1> xl, xr;
    Eigen::Matrix<T, 3, 4> Pl, Pr;

    // dst
    Eigen::Matrix<T, 3, 1> XEuclidean;
    Eigen::Matrix<T, 4, 1> XHomogeneous;

    // convert to Eigen types
    cv2eigen<T, 2, 1>(_xl, xl);
    cv2eigen<T, 2, 1>(_xr, xr);
    cv2eigen<T, 3, 4>(_Pl, Pl);
    cv2eigen<T, 3, 4>(_Pr, Pr);

    // libmv implementation
    if( points3d.rows == 3 )
    {
        libmv::TriangulateDLT(Pl, xl, Pr, xr, &XEuclidean);
        eigen2cv<T, 3, 1>(XEuclidean, points3d);
    }
    else
    {
        libmv::TriangulateDLT(Pl, xl, Pr, xr, &XHomogeneous);
        eigen2cv<T, 4, 1>(XHomogeneous, points3d);
    }
}

template<typename T>
void
triangulatePoints_(unsigned nviews, const vector<cv::Mat> & points2d, const vector<cv::Mat> & projection_matrices,
                   cv::Mat & points3d, int method)
{
    // Two view
    if( nviews == 2 )
    {
        Mat xl = points2d.at(0);               // left points
        Mat xr = points2d.at(1);               // right points
        Mat Pl = projection_matrices.at(0);    // left matrix projection
        Mat Pr = projection_matrices.at(1);    // right matrix projection

        // number of points
        unsigned npoints = xl.cols;
        CV_Assert( xr.cols == npoints );

        // pre-allocation
        if( points3d.rows == 3 )
        {
            points3d.create( 3, npoints, xl.type() );
        }
        else
        {
            points3d.create( 4, npoints, xl.type() );
        }

        // triangulate
        for( unsigned i = 0; i < npoints; ++i )
        {
            Mat current_points3d = points3d.col(i);
            triangulateDLT<T>( xl.col(i), xr.col(i), Pl, Pr, current_points3d );
        }


//         if( method == CV_TRIANG_DLT )
//         {
//             triangulateDLT<T>( xl, xr, Pl, Pr, points3d );
//         }
//         else if( method == CV_TRIANG_BY_PLANE )
//         {
//             // Fundamental matrix
//             libmv::Mat3 F;
//             libmv::NormalizedEightPointSolver(xl, xr, &F);
// 
//             // Essential matrix
//             libmv::Mat3 E;
// //             libmv::EssentialFromFundamental(F, K1, K2, &E);
// 
//             libmv::TwoViewTriangulationByPlanes(xl, xr, Pr, E, &XEuclidean);
//         }

    }
    else
    {
    }
}


void
triangulatePoints(InputArrayOfArrays _points2d, InputArrayOfArrays _projection_matrices,
                  OutputArray _points3d, int method)
{
    unsigned nviews = (unsigned) _points2d.total();
    CV_Assert(nviews >= 2 && nviews == _projection_matrices.total());

    vector<Mat> points2d;
    _points2d.getMatVector(points2d);

    vector<Mat> projection_matrices;
    _projection_matrices.getMatVector(projection_matrices);

    cv::Mat points3d = _points3d.getMat();

    // ToDo (pablo): libmv only has a version for 'double'
//     if( _points2d.getMat(0).depth() == CV_32F )
//     {
//         triangulatePoints_<float>(nviews, points2d, projection_matrices, points3d, method);
//     }
//     else
//     {
        triangulatePoints_<double>(nviews, points2d, projection_matrices, points3d, method);
//     }

    points3d.copyTo(_points3d);
}

} /* namespace cv */

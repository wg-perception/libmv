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

#include "opencv2/sfm/sfm.hpp"
#include "libmv/multiview/twoviewtriangulation.h"
#include "libmv/multiview/fundamental.h"

#include <opencv2/core/eigen.hpp>


using namespace cv;
using namespace std;

namespace cv
{

// HZ 12.2 pag.312
template<typename T>
void
triangulateDLT( const Mat_<T> &xl, const Mat_<T> &xr, const Mat_<T> &Pl, const Mat_<T> &Pr, Mat &points3d)
{
    Matx<T, 4, 4> design;
    for (int i = 0; i < 4; ++i)
    {
        design(0,i) = xl(0) * Pl(2,i) - Pl(0,i);
        design(1,i) = xl(1) * Pl(2,i) - Pl(1,i);
        design(2,i) = xr(0) * Pr(2,i) - Pr(0,i);
        design(3,i) = xr(1) * Pr(2,i) - Pr(1,i);
    }

    Matx<T, 4, 1> XHomogeneous;
    cv::SVD::solveZ(design, XHomogeneous);

    homogeneousToEuclidean(XHomogeneous, points3d);
}

// x's are 2D coordinates (x,y,1) in each image; Ps are projective cameras.
// The output, X, is a 3D vector.
// Algorithm is the standard DLT; for derivation see appendix of Keir's thesis.
template<typename T>
void
NViewTriangulate(const Mat &x, const vector<Mat> &Ps, Mat &X)
{
    unsigned nviews = x.cols;
    CV_Assert(nviews == Ps.size());

    cv::Mat_<T> design = cv::Mat_<T>::zeros(3*nviews, 4 + nviews);
    for (unsigned i=0; i < nviews; ++i) {
        design(Range(3*i,3*i+3), Range(0, 4)) = -Ps[i];
        design(3*i + 0, 4 + i) = x.at<T>(0, i);
        design(3*i + 1, 4 + i) = x.at<T>(1, i);
        design(3*i + 2, 4 + i) = 1.0;
    }

    Mat_<T> X_and_alphas;
    cv::SVD::solveZ(design, X_and_alphas);
    homogeneousToEuclidean(X_and_alphas.rowRange(0, 4), X);
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
        points3d.create( 3, npoints, xl.type() );

        // triangulate
        for( unsigned i = 0; i < npoints; ++i )
        {
            Mat current_points3d = points3d.col(i);
            triangulateDLT<T>( xl.col(i), xr.col(i), Pl, Pr, current_points3d );
        }
    }
    else if( nviews > 2 )
    {
        // defs
        unsigned npoints = points2d.at(0).cols;
        int type = points2d.at(0).type();

        // check dimensions
        for( unsigned k=1; k < nviews; ++k )
            CV_Assert( points2d.at(k).cols == npoints );

        // pre-allocation
        points3d.create( 3, npoints, type );

        // triangulate
        for( unsigned i=0; i < npoints; ++i )
        {
            // build x matrix (one point per view)
            Mat x( 2, nviews, type );
            for( unsigned k=0; k < nviews; ++k )
            {
                points2d.at(k).col(i).copyTo( x.col(k) );
            }

            Mat current_points3d = points3d.col(i);
            NViewTriangulate<T>( x, projection_matrices, current_points3d );
        }
    }
}

template<typename T>
void
triangulatePoints_(unsigned nviews, const vector<Mat> & points2d, const vector<Mat> & K,
                   const vector<Mat> & R, const vector<Mat> & t,
                   Mat & points3d, int method)
{
    if( method == CV_TRIANG_DLT)
    {
        // Compute projection matrices
        std::vector<cv::Mat> P;
        P.resize(nviews);
        for (unsigned i = 0; i < nviews; ++i)
        {
            P[i].create(3, 4, points2d.at(0).depth() );
            cv::Mat(K[i] * R[i]).copyTo(P[i].colRange(0, 3));
            cv::Mat(K[i] * t[i]).copyTo(P[i].col(3));
        }

        triangulatePoints_<T>(nviews, points2d, P, points3d, method);
    }
    else if( method == CV_TRIANG_BY_PLANE )
    {
        // Two view
        if( nviews == 2 )
        {
            libmv::Vec3 xl, xr;
            cv2eigen( points2d.at(0), xl );
            cv2eigen( points2d.at(1), xr );

            // Fundamental matrix
            libmv::Mat3 F;
            libmv::NormalizedEightPointSolver(xl, xr, &F);

            libmv::Mat K1, K2;
            cv2eigen( K.at(0), K1 );
            cv2eigen( K.at(1), K2 );

            // Essential matrix
            libmv::Mat3 E;
            libmv::EssentialFromFundamental(F, K1, K2, &E);

            // Mat Pl; // [ I | 0 ]
            Mat Pr;
            cv::Mat(K[1] * R[1]).copyTo(Pr.colRange(0, 3));
            cv::Mat(K[1] * t[1]).copyTo(Pr.col(3));
            libmv::Mat34 P2;
            cv2eigen( Pr, P2 );

            // triangulation by planes
            libmv::Vec4 XEuclidean;
            libmv::TwoViewTriangulationByPlanes(xl, xr, P2, E, &XEuclidean);

            eigen2cv( XEuclidean, points3d );
        }
        else if( nviews > 2 )
        {
//             CV_ERROR( CV_StsBadArg, "Invalid number of views" );
        }
    }
    else
    {
//         CV_ERROR( CV_StsBadArg, "Invalid method" );
    }
}

void
triangulatePoints(InputArrayOfArrays _points2d, InputArrayOfArrays _projection_matrices,
                  OutputArray _points3d, int method)
{
    // check
    unsigned nviews = (unsigned) _points2d.total();
    CV_Assert(nviews >= 2 && nviews == _projection_matrices.total());

    // inputs
    vector<Mat> points2d;
    _points2d.getMatVector(points2d);

    vector<Mat> projection_matrices;
    _projection_matrices.getMatVector(projection_matrices);

    // output
    cv::Mat points3d = _points3d.getMat();

    // type: float or double
    if( _points2d.getMat(0).depth() == CV_32F )
    {
        triangulatePoints_<float>(nviews, points2d, projection_matrices, points3d, method);
    }
    else
    {
        triangulatePoints_<double>(nviews, points2d, projection_matrices, points3d, method);
    }

    points3d.copyTo(_points3d);
}


void
triangulatePoints(InputArrayOfArrays _points2d, InputArrayOfArrays _K,
                  InputArrayOfArrays _R, InputArrayOfArrays _t,
                  OutputArray _points3d, int method)
{
    // check
    unsigned nviews = (unsigned) _points2d.total();
    CV_Assert(nviews >= 2 && nviews == _R.total() && nviews == _t.total());

    // inputs
    vector<Mat> points2d, K, R, t;
    _points2d.getMatVector(points2d);
    _K.getMatVector(K);
    _R.getMatVector(R);
    _t.getMatVector(t);

    // output
    cv::Mat points3d = _points3d.getMat();

    // type: float or double
    if( _points2d.getMat(0).depth() == CV_32F )
    {
        triangulatePoints_<float>(nviews, points2d, K, R, t, points3d, method);
    }
    else
    {
        triangulatePoints_<double>(nviews, points2d, K, R, t, points3d, method);
    }

    points3d.copyTo(_points3d);
}


} /* namespace cv */

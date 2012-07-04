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
#include "libmv/multiview/triangulation.h"
#include <opencv2/core/eigen.hpp>

using namespace cv;
using namespace std;

template<typename T>
void
triangulatePoints_(unsigned nviews, const vector<cv::Mat> & points2d, const vector<cv::Mat> & projection_matrices,
                  cv::Mat & points3d, int method)
{
    // Two view
    if (nviews == 2 && method == CV_TRIANG_DLT)
    {
        Eigen::Matrix<T, 2, 1> x1, x2;
        Eigen::Matrix<T, 3, 4> P1, P2;
        Eigen::Matrix<T, 3, 1> X_euclidean;

        // ToDo (pablo): Check dimensions
        
        cv2eigen<T, 2, 1>(points2d.at(0),x1);
        cv2eigen<T, 2, 1>(points2d.at(1),x2);
        cv2eigen<T, 3, 4>(projection_matrices.at(0),P1);
        cv2eigen<T, 3, 4>(projection_matrices.at(1),P2);

        libmv::TriangulateDLT(P1, x1, P2, x2, &X_euclidean);

        eigen2cv<T, 3, 1>(X_euclidean,points3d);
    }
    else
    {
    }
}

void
triangulatePoints(const InputArrayOfArrays _points2d, const InputArrayOfArrays _projection_matrices,
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
}

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

// Eigen
#include <Eigen/Core>

// Open CV
#include <opencv2/sfm/sfm.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

// libmv headers
#include <libmv/reconstruction/reconstruction.h>
#include <libmv/reconstruction/projective_reconstruction.h>
#include <libmv/reconstruction/projective_reconstruction.h>

#include <iostream>

using namespace cv;
using namespace libmv;
using namespace std;

// Temp debug macro
#define BR exit(1);

namespace cv
{

  //  Reconstruction function for API
  void
  reconstruct(const InputArrayOfArrays points2d, OutputArrayOfArrays projection_matrices, OutputArray points3d,
              bool is_projective, bool has_outliers, bool is_sequence)
  {

    int nviews = points2d.total();
    cv::Mat F;

    // OpenCV data types
    std::vector<cv::Mat> pts2d;
    points2d.getMatVector(pts2d);
    int depth = pts2d[0].depth();

    // Projective reconstruction

    if (is_projective)
    {

      // Two view reconstruction

      if (nviews == 2)
      {

        // Get fundamental matrix
        fundamental8Point(pts2d[0], pts2d[1], F, has_outliers);

        // Get Projection matrices
        cv::Mat P, Pp;
        projectionsFromFundamental(F, P, Pp);
        projection_matrices.create(2, 1, depth);
        P.copyTo(projection_matrices.getMatRef(0));
        Pp.copyTo(projection_matrices.getMatRef(1));

        //  Triangulate and find 3D points using inliers
        triangulatePoints(points2d, projection_matrices, points3d);
      }
    }

    // Affine reconstruction

    else
    {

      // Two view reconstruction

      if (nviews == 2)
      {

      }
      else
      {

      }

    }

  }

} // namespace cv

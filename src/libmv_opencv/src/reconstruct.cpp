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

/* Eigen */
#include <Eigen/Core>

/* Open CV */
#include <opencv2/sfm/sfm.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d/calib3d.hpp>

/* libmv headers */
#include <libmv/reconstruction/reconstruction.h>
#include <libmv/reconstruction/projective_reconstruction.h>

#include <iostream>

using namespace cv;
using namespace libmv;
using namespace std;

/* Temp debug macro */
#define BR exit(1);

namespace cv
{
#if 0
  void
  ptvec2mat(std::vector<cv::Point2d> &pvec, cv::Mat &pmat)
  {
    pmat = Mat(2, pvec.size(), CV_64F);

    for (int i = 0; i < pvec.size(); ++i)
    {
      pmat.at<double>(0, i) = pvec[i].x;
      pmat.at<double>(1, i) = pvec[i].y;
    }
  }

  void
  iparr_2_ptvec(const InputArrayOfArrays points2d, std::vector<std::vector<Point2d> >& points2dvec)
  {
    int nviews = (int) points2d.total();
    for (int m = 0; m < nviews; ++m)
    {
      std::vector<Point2d> imgpts;
      imgpts = points2d.getMat(m);
      points2dvec.push_back(imgpts);
    }
  }

  void
  ptvec_2_matches(std::vector<std::vector<Point2d> >& points2d, libmv::Matches& matches)
  {
    int nviews = points2d.size();

    for (int m = 0; m < nviews; ++m)
    {
      std::vector<Point2d> imgpts;
      imgpts = points2d[m];

      for (int n = 0; n < imgpts.size(); ++n)
      {
        PointFeature * feature = new PointFeature(imgpts[n].x, imgpts[n].y);
        matches.Insert(m, n, feature);
        feature = (PointFeature *) matches.Get(m, n);
      }
    }

  }

  /* Converts 2d vector of points to 2xN Mat */
  void
  ptvec2d_2_mat(std::vector<cv::Point2d> ptvec2d, cv::Mat& mat)
  {
    mat = Mat::zeros(2, ptvec2d.size(), CV_64F);
    for (int n = 0; n < ptvec2d.size(); ++n)
    {
      mat.at<double>(0, n) = ptvec2d[n].x;
      mat.at<double>(1, n) = ptvec2d[n].y;
    }
  }
#endif

  /*  Build libmv matches from input points2d*/
  void
  points2d_2_matches(const std::vector<cv::Mat> & pts2d, libmv::Matches& matches)
  {
    for (int m = 0; m < pts2d.size(); ++m)
    {
      for (int n = 0; n < pts2d[m].cols; ++n)
      {
        PointFeature * feature;
        if (pts2d[m].depth() == CV_32F)
          feature = new PointFeature(pts2d[m].at<float >(0, n), pts2d[m].at<float>(0, n));
        else
          feature = new PointFeature(pts2d[m].at<double>(0, n), pts2d[m].at<double>(0, n));
        matches.Insert(m, n, feature);
      }
    }

  }

  /*  Builds projection matrix array from libmv Reconstruction*/
  void
  recon_2_projmatvec(libmv::Reconstruction& recon, OutputArrayOfArrays Pv, int depth)
  {
    libmv::PinholeCamera *cam;

    for (int m = 0; m < recon.GetNumberCameras(); ++m)
    {
      cam = dynamic_cast<PinholeCamera *>(recon.GetCamera(m));
      cv::Mat P;
      eigen2cv(cam->projection_matrix(), P);
      P.convertTo(Pv.getMatRef(m), depth);
    }
  }

  /* reconstruction function for API */
  void
  reconstruct(const InputArrayOfArrays points2d, OutputArrayOfArrays projection_matrices, OutputArray points3d,
              bool is_projective, bool has_outliers, bool is_sequence)
  {

    int nviews = points2d.total();

    /* OpenCV data types */
    bool result = false;
    std::vector<std::vector<Point2d> > _points2d;
    std::vector<cv::Mat> pts2d;
    points2d.getMatVector(pts2d);
    int depth = pts2d[0].depth();

    /* Data types needed by libmv functions */
    Matches matches;
    Matches matches_inliers;
    Reconstruction recon;

    /* Convert OpenCV types to libmv types */
    points2d_2_matches(pts2d, matches);

    /* Projective reconstruction*/

    if (is_projective)
    {

      /* Two view reconstruction */

      if (nviews == 2)
      {
        result = ReconstructFromTwoUncalibratedViews(matches, 0, 1, &matches_inliers, &recon);

        /* Get projection matrices */

        CV_Assert(recon.GetNumberCameras() == nviews);
        projection_matrices.create(1, nviews, 0, -1, true, 0);
        recon_2_projmatvec(recon, projection_matrices, depth);

        /* Triangulate and find 3D points */

        triangulatePoints(points2d, projection_matrices, points3d);
      }

    }

    /* Euclidian reconstruction*/

    else
    {

      /* Two view reconstruction */

      if (nviews == 2)
      {
        result = ReconstructFromTwoUncalibratedViews(matches, 0, 1, &matches_inliers, &recon);

        /* Get projection matrices */

        CV_Assert(recon.GetNumberCameras() == nviews);
        projection_matrices.create(1, nviews, 0 /*type*/, -1, true, 0);
        recon_2_projmatvec(recon, projection_matrices, depth);

        /* Triangulate and find 3D points */

        triangulatePoints(points2d, projection_matrices, points3d);
      }

    }

    /* Assert if reconstruction succeeded */
    CV_Assert(result == true);
  }

}

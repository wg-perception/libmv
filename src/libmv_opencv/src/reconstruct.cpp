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

namespace cv
{
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
      std::vector < Point2d > imgpts;
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
      std::vector < Point2d > imgpts;
      imgpts = points2d[m];

      for (int n = 0; n < imgpts.size(); ++n)
      {
//        cout << imgpts[n] << endl;
        PointFeature * feature = new PointFeature(imgpts[n].x, imgpts[n].y);
        matches.Insert(m, n, feature);
        feature = (PointFeature *) matches.Get(m, n);
//        cout << feature->coords << endl;
      }
    }

  }

  void
  recon_2_projmatvec(libmv::Reconstruction& recon, std::vector<cv::Mat>& Pv)
  {
    libmv::PinholeCamera *cam;

    for (int m = 0; m < recon.GetNumberCameras(); ++m)
    {
      cam = (PinholeCamera *) recon.GetCamera(m);
      cv::Mat P;
      eigen2cv(cam->GetPoseMatrix(), P);
//      cout << P << endl;
      Pv.push_back(P);
    }
  }

  void
  projmatvec_2_oparr(std::vector<cv::Mat>& P, OutputArrayOfArrays projection_matrices)
  {
//    cout << P.size() << endl;
    projection_matrices.create(P.size(), 1, CV_64FC3);

    for (int m = 0; m < P.size(); ++m)
    {
//      cout << P[m] << endl;
      projection_matrices.create(P[m].rows, P[m].cols, CV_64F, m, true);
      memcpy(projection_matrices.getMat(m).data, P[m].ptr<double>(0), P[m].rows * P[m].cols * sizeof(double));
    }
  }

  void
  reconstruct(const InputArrayOfArrays points2d, OutputArrayOfArrays projection_matrices, OutputArray points3d,
              bool is_projective, bool has_outliers, bool is_sequence)
  {

    /* OpenCV data types */
    bool result = false;
    std::vector < std::vector<Point2d> > _points2d;
    std::vector < cv::Mat > P;

    /* Data types needed by libmv functions */
    Matches matches;
    Matches matches_inliers;
    Reconstruction recon;

    /* Convert OpenCV types to libmv types */
    iparr_2_ptvec(points2d, _points2d);
    ptvec_2_matches(_points2d, matches);

    int nviews = _points2d.size();
    // cout << nviews << endl;

    /* Projective reconstruction*/

    if (is_projective)
    {

      /* Two view reconstruction */

      if (nviews == 2)
        result = ReconstructFromTwoUncalibratedViews(matches, 0, 1, &matches_inliers, &recon);

      /* Get projection matrices */
      CV_Assert(recon.GetNumberCameras() == nviews);
      recon_2_projmatvec(recon, P);
      projmatvec_2_oparr(P, projection_matrices);
      cout << projection_matrices.getMat(0) << endl;
      cout << projection_matrices.getMat(1) << endl;


      /*  Triangulate and find  3D points*/
      /*      cv::Mat pt4d(4, 10, CV_64F);
       cv::Mat pt2d[2];
       ptvec2mat(_points2d[0], pt2d[0]);
       ptvec2mat(_points2d[1], pt2d[1]);
       CV_Assert(pt2d[0].cols == pt2d[1].cols);
       cout << pt2d[0] << endl;
       cout << pt2d[1] << endl;
       cout << Pcv[0] << endl;
       cout << Pcv[1] << endl;*/

      // This gives seg fault??
//      cv::triangulatePoints(Pcv[0], Pcv[1], pt2d[0], pt2d[1], pt4d);
    }
    /* Euclidian reconstruction*/
    else
    {

    }

    /* Give error if reconstruction failed */
//    CV_Assert(result == true);
  }

}

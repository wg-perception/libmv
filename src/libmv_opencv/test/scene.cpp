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

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>

#include "test_precomp.hpp"

template<typename T>
cv::Mat_<T>
randomK(bool is_projective)
{
  static cv::RNG rng;

  cv::Mat_<T> K = cv::Mat_ < T > ::zeros(3, 3);
  K(0, 0) = rng.uniform(100, 1000);
  K(1, 1) = rng.uniform(100, 1000);
  if (is_projective)
  {
    K(0, 2) = rng.uniform(-100, 100);
    K(1, 2) = rng.uniform(-100, 100);
  }
  K(2, 2) = 1.0;

  return K;
}

/** Generate a set of random views of a rigid scene for testing purposes
 * @param n_points
 * @param K
 * @param R
 * @param T
 * @param P
 * @param points3d
 * @param points2d
 */
template<typename T>
void
generateScene(size_t n_views, size_t n_points, bool is_projective, cv::Mat & K_out, std::vector<cv::Mat> & R_out,
              std::vector<cv::Mat> & t_out, cv::Mat & points3d_out)
{
  cv::RNG rng;

  // Generate a bunch of random 3d points in a 0, 1 cube
  cv::Mat_<T> points3d(3, n_points);
  rng.fill(points3d, cv::RNG::UNIFORM, 0, 1);

  // Generate random intrinsics
  cv::Mat_<T> K = randomK<T>(is_projective);

  // Generate random camera poses
  // TODO deal with smooth camera poses (e.g. from a video sequence)
  std::vector<cv::Mat_<T> > R(n_views);
  std::vector<cv::Vec<T, 3> > t(n_views);
  for (size_t i = 0; i < n_views; ++i)
  {
    // Get a random rotation axis
    cv::Vec<T, 3> vec;
    rng.fill(vec, cv::RNG::UNIFORM, 0, 1);
    // Give a random angle to the rotation vector
    vec = vec / cv::norm(vec) * rng.uniform(0.0f, float(2 * CV_PI));
    cv::Rodrigues(vec, R[i]);
    // Create a random translation
    t[i] = cv::Vec<T, 3>(rng.uniform(-0.5f, 0.5f), rng.uniform(-0.5f, 0.5f), rng.uniform(1.0f, 2.0f));
    // Make sure the shape is in front of the camera
    cv::Mat_<T> points3d_transformed = R[i] * points3d + cv::Mat(t[i]) * cv::Mat_ < T > ::ones(1, n_points);
    double min_dist, max_dist;
    cv::minMaxIdx(points3d_transformed.row(2), &min_dist, &max_dist);
    if (min_dist < 0)
      t[i][2] = t[i][2] - min_dist + 1.0;
  }

  // Copy the data back
  K.copyTo(K_out);
  R_out.resize(n_views);
  t_out.resize(n_views);
  for (size_t i = 0; i < n_views; ++i)
  {
    R[i].copyTo(R_out[i]);
    cv::Mat(t[i]).copyTo(t_out[i]);
  }
  points3d.copyTo(points3d_out);
}

void
generateScene(size_t n_views, size_t n_points, bool is_projective, int depth, cv::Mat & K, std::vector<cv::Mat> & R,
              std::vector<cv::Mat> & t, std::vector<cv::Mat> & P, cv::Mat & points3d, std::vector<cv::Mat> & points2d)
{
  CV_Assert(depth == CV_32F || depth == CV_64F);

  R.resize(n_views);
  t.resize(n_views);

  if (depth == CV_32F)
    generateScene<float>(n_views, n_points, is_projective, K, R, t, points3d);
  else
    generateScene<double>(n_views, n_points, is_projective, K, R, t, points3d);

  // Compute projection matrices
  P.resize(n_views);
  for (size_t i = 0; i < n_views; ++i)
  {
    P[i].create(3, 4, depth);
    cv::Mat(K * R[i]).copyTo(P[i].colRange(0, 3));
    cv::Mat(K * t[i]).copyTo(P[i].col(3));
  }

  // Compute homogeneous 3d points
  cv::Mat points3d_homogeneous(4, n_points, depth);
  points3d.copyTo(points3d_homogeneous.rowRange(0, 3));
  points3d_homogeneous.row(3).setTo(1);
  // Project those points for every view
  points2d.resize(n_views);
  for (size_t i = 0; i < n_views; ++i)
  {
    cv::Mat points2d_tmp = P[i] * points3d_homogeneous;
    points2d[i].create(2, n_points, depth);
    for (unsigned char j = 0; j < 2; ++j)
      cv::Mat(points2d_tmp.row(j) / points2d_tmp.row(2)).copyTo(points2d[i].row(j));
  }

// TODO: remove a certain number of points per view
// TODO: add a certain number of outliers per view

}

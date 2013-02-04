// Copyright (c) 2012 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include "ceres/ceres.h"
#include "libmv/logging/logging.h"
#include "libmv/multiview/homography.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/simple_pipeline/keyframe_selection.h"

namespace libmv {

namespace {

Vec2 NorrmalizedToPixelSpace(Vec2 vec, CameraIntrinsics &intrinsics) {
  Vec2 result;

  result(0) = vec(0) * intrinsics.focal_length_x() + intrinsics.principal_point_x();
  result(1) = vec(1) * intrinsics.focal_length_y() + intrinsics.principal_point_y();

  return result;
}

Mat3 IntrinsicsNormalizationMatrix(CameraIntrinsics &intrinsics) {
  Mat3 T = Mat3::Identity(), S = Mat3::Identity();

  T(0, 2) = -intrinsics.principal_point_x();
  T(1, 2) = -intrinsics.principal_point_y();

  S(0, 0) /= intrinsics.focal_length_x();
  S(1, 1) /= intrinsics.focal_length_y();

  return S * T;
}

class HomographySymmetricGeometricCostFunctor {
 public:
  HomographySymmetricGeometricCostFunctor(Vec2 x, Vec2 y)
      : x_(x),
        y_(y) {
  }

  template<typename T>
  bool operator()(const T *homography_parameters, T *residuals) const {
    typedef Eigen::Matrix<T, 3, 3> Mat3;
    typedef Eigen::Matrix<T, 3, 1> Vec3;

    Mat3 H(homography_parameters);

    Vec3 x(T(x_(0)), T(x_(1)), T(1.0));
    Vec3 y(T(y_(0)), T(y_(1)), T(1.0));

    Vec3 H_x = H * x;
    Vec3 Hinv_y = H.inverse() * y;

    H_x /= H_x(2);
    Hinv_y /= Hinv_y(2);

    residuals[0] = H_x(0) - T(y_(0));
    residuals[1] = H_x(1) - T(y_(1));

    residuals[2] = Hinv_y(0) - T(x_(0));
    residuals[3] = Hinv_y(1) - T(x_(1));

    return true;
  }

  const Vec2 x_;
  const Vec2 y_;
};

void ComputeHomographyFromCorrespondences(Mat &x1, Mat &x2, CameraIntrinsics &intrinsics, Mat3 *H) {
  // Algebraic homography estimation, happens with normalized coordinates
  Homography2DFromCorrespondencesLinear(x1, x2, H, 1e-12);

  // Convert homography to original pixel space
  Mat3 N = IntrinsicsNormalizationMatrix(intrinsics);
  *H = N.inverse() * (*H) * N;

  // Refine matrix using Ceres minimizer, it'll be in pixel space
  ceres::Problem problem;

  for (int i = 0; i < x1.cols(); i++) {
    Vec2 pixel_space_x1 = NorrmalizedToPixelSpace(x1.col(i), intrinsics),
         pixel_space_x2 = NorrmalizedToPixelSpace(x2.col(i), intrinsics);

    HomographySymmetricGeometricCostFunctor *homography_symmetric_geometric_cost_function =
        new HomographySymmetricGeometricCostFunctor(pixel_space_x1, pixel_space_x2);

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<
            HomographySymmetricGeometricCostFunctor,
            4, /* num_residuals */
            9>(homography_symmetric_geometric_cost_function),
        NULL,
        H->data());
  }

  // Configure the solve.
  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  solver_options.max_num_iterations = 50;
  solver_options.update_state_every_iteration = true;
  solver_options.parameter_tolerance = 1e-16;
  solver_options.function_tolerance = 1e-16;

  // Run the solve.
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  VLOG(1) << "Summary:\n" << summary.FullReport();
}

class FundamentalSymmetricEpipolarCostFunctor {
 public:
  FundamentalSymmetricEpipolarCostFunctor(Vec2 x, Vec2 y)
      : x_(x),
        y_(y) {
  }

  template<typename T>
  bool operator()(const T *fundamental_parameters, T *residuals) const {
    typedef Eigen::Matrix<T, 3, 3> Mat3;
    typedef Eigen::Matrix<T, 3, 1> Vec3;

    Mat3 F(fundamental_parameters);

    Vec3 x(T(x_(0)), T(x_(1)), T(1.0));
    Vec3 y(T(y_(0)), T(y_(1)), T(1.0));

    Vec3 F_x = F * x;
    Vec3 Ft_y = F.transpose() * y;
    T y_F_x = y.dot(F_x);

    residuals[0] = y_F_x * T(1) / F_x.head(2).norm();
    residuals[1] = y_F_x * T(1) / Ft_y.head(2).norm();

    return true;
  }

  const Mat x_;
  const Mat y_;
};

void ComputeFundamentalFromCorrespondences(Mat &x1, Mat &x2, CameraIntrinsics &intrinsics, Mat3 *F)
{
  // Algebraic fundamental estimation, happens with normalized coordinates
  NormalizedEightPointSolver(x1, x2, F);

  // Convert fundamental to original pixel space
  Mat3 N = IntrinsicsNormalizationMatrix(intrinsics);
  *F = N.inverse() * (*F) * N;

  // Refine matrix using Ceres minimizer, it'll be in pixel space
  ceres::Problem problem;

  for (int i = 0; i < x1.cols(); i++) {
    Vec2 pixel_space_x1 = NorrmalizedToPixelSpace(x1.col(i), intrinsics),
         pixel_space_x2 = NorrmalizedToPixelSpace(x2.col(i), intrinsics);

    FundamentalSymmetricEpipolarCostFunctor *fundamental_symmetric_epipolar_cost_function =
        new FundamentalSymmetricEpipolarCostFunctor(pixel_space_x1, pixel_space_x2);

    problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<
            FundamentalSymmetricEpipolarCostFunctor,
            2, /* num_residuals */
            9>(fundamental_symmetric_epipolar_cost_function),
        NULL,
        F->data());
  }

  // Configure the solve.
  ceres::Solver::Options solver_options;
  solver_options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
  solver_options.max_num_iterations = 50;
  solver_options.update_state_every_iteration = true;
  solver_options.parameter_tolerance = 1e-16;
  solver_options.function_tolerance = 1e-16;

  // Run the solve.
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

  VLOG(1) << "Summary:\n" << summary.FullReport();
}

// P.H.S. Torr
// Geometric Motion Segmentation and Model Selection
//
// http://reference.kfupm.edu.sa/content/g/e/geometric_motion_segmentation_and_model__126445.pdf
//
// d is the number of dimensions modeled (d = 3 for a fundamental matrix or 2 for a homography)
// k is the number of degrees of freedom in the model (k = 7 for a fundamental matrix or 8 for a homography)
// r is the dimension of the data (r = 4 for 2D correspondences between two frames)
double GRIC(Vec &e, int d, int k, int r) {
  int n = e.rows();
  double lambda1 = log((double) r);
  double lambda2 = log((double) r * n);

  // lambda3 limits the residual error, and this paper
  // http://elvera.nue.tu-berlin.de/files/0990Knorr2006.pdf
  // suggests using lambda3 of 2
  // same value is used in Torr's Problem of degeneracy in structure and motion recovery
  // from uncalibrated image sequences
  // http://www.robots.ox.ac.uk/~vgg/publications/papers/torr99.ps.gz
  double lambda3 = 2.0;

  // measurement error of tracker
  double sigma2 = 0.01;

  // Actual GRIC computation
  double gric_result = 0.0;

  for (int i = 0; i < n; i++) {
    double rho = std::min(e(i) * e(i) / sigma2, lambda3 * (r - d));
    gric_result += rho;
  }

  gric_result += lambda1 * d * n;
  gric_result += lambda2 * k;

  return gric_result;
}

} // namespace

void SelectkeyframesBasedOnGRIC(Tracks &tracks,
                                CameraIntrinsics &intrinsics,
                                vector<int> &keyframes) {
  // Mirza Tahir Ahmed, Matthew N. Dailey
  // Robust key frame extraction for 3D reconstruction from video streams
  //
  // http://www.cs.ait.ac.th/~mdailey/papers/Tahir-KeyFrame.pdf

  int max_image = tracks.MaxImage();
  int next_keyframe = 1;
  int number_keyframes = 0;

  // Limit correspondence ratio from both sides.
  // On the one hand if number of correspondent features is too low,
  // triangulation will suffer.
  // On the other hand high correspondence likely means short baseline.
  // which also will affect om accuracy
  const double Tmin = 0.8;
  const double Tmax = 1.0;

  while (next_keyframe != -1) {
    int current_keyframe = next_keyframe;

    LG << "Found keyframe " << next_keyframe;

    keyframes.push_back(next_keyframe);
    number_keyframes++;
    next_keyframe = -1;

    for (int candidate_image = current_keyframe + 1;
         candidate_image <= max_image;
         candidate_image++)
    {
      // Conjunction of all markers from both keyframes
      vector<Marker> all_markers = tracks.MarkersInBothImages(current_keyframe, candidate_image);

      // Match keypoints between frames current_keyframe and candidate_image
      vector<Marker> tracked_markers = tracks.MarkersForTracksInBothImages(current_keyframe, candidate_image);

      // Correspondences in normalized space
      Mat x1, x2;
      CoordinatesForMarkersInImage(tracked_markers, current_keyframe, &x1);
      CoordinatesForMarkersInImage(tracked_markers, candidate_image, &x2);

      LG << "Found " << x1.cols() << " correspondences between " << current_keyframe
         << " and " << candidate_image;

      // Not enough points to construct fundamental matrix
      if (x1.cols() < 8 || x2.cols() < 8)
        continue;

      // Correspondence ratio constraint
      int Tc = tracked_markers.size();
      int Tf = all_markers.size();
      double Rc = (double) Tc / (double) Tf;

      LG << "Correspondence between " << current_keyframe << " and " << candidate_image
         << ": " << Rc;

      if (Rc < Tmin || Rc > Tmax)
        continue;

      Mat3 H, F;
      ComputeHomographyFromCorrespondences(x1, x2, intrinsics, &H);
      ComputeFundamentalFromCorrespondences(x1, x2, intrinsics, &F);

      // TODO(sergey): Discard outlier matches

      // Compute error values for homography and fundamental matrices
      Vec H_e, F_e;
      H_e.resize(x1.cols());
      F_e.resize(x1.cols());
      for (int i = 0; i < x1.cols(); i++) {
        Vec2 current_x1 = NorrmalizedToPixelSpace(Vec2(x1(0, i), x1(1, i)), intrinsics);
        Vec2 current_x2 = NorrmalizedToPixelSpace(Vec2(x2(0, i), x2(1, i)), intrinsics);

        H_e(i) = SymmetricGeometricDistance(H, current_x1, current_x2);
        F_e(i) = SymmetricEpipolarDistance(F, current_x1, current_x2);
      }

      LG << "H_e: " << H_e.transpose();
      LG << "F_e: " << F_e.transpose();

      // Degeneracy constraint
      double GRIC_H = GRIC(H_e, 2, 8, 4);
      double GRIC_F = GRIC(F_e, 3, 7, 4);

      LG << "GRIC values for frames " << current_keyframe << " and " << candidate_image
         << ", H-GRIC: " << GRIC_H << ", F-GRIC: " << GRIC_F;

      if (GRIC_H <= GRIC_F)
        continue;

      // TODO(sergey): PELC criterion

      next_keyframe = candidate_image;
    }

    // This is a bit arbitrary and main reason of having this is to deal better
    // with situations when there's no keyframes were found for current current
    // keyframe this could happen when there's no so much parallax in the beginning
    // of image sequence and then most of features are getting occluded.
    // In this case there could be good keyframe pain in the middle of the sequence
    //
    // However, it's just quick hack and smarter way to do this would be nice
    //
    // Perhaps we shouldn't put all the keyframes inti the same plain list and
    // use some kind of sliced list instead
    if (next_keyframe == -1) {
      if (number_keyframes == 1) {
        LG << "Removing previous candidate keyframe because no other keyframe could be used with it";
        keyframes.pop_back();
      }

      next_keyframe = current_keyframe + 10;
      number_keyframes = 0;

      if (next_keyframe >= max_image)
        break;

      LG << "Starting searching for keyframes starting from " << next_keyframe;
    }
  }

  //for (int i =  0; i < keyframes.size(); i++)
  //  LG << keyframes[i];
}

} // namespace libmv

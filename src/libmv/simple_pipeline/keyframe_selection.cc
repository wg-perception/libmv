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
template<typename T>
T SymmetricGeometricDistance(const Eigen::Matrix<T, 3, 3> &H,
                                  const Eigen::Matrix<T, 2, 1> &x1,
                                  const Eigen::Matrix<T, 2, 1> &x2) {
  Eigen::Matrix<T, 3, 1> x(x1(0), x1(1), T(1.0));
  Eigen::Matrix<T, 3, 1> y(x2(0), x2(1), T(1.0));

  Eigen::Matrix<T, 3, 1> H_x = H * x;
  Eigen::Matrix<T, 3, 1> Hinv_y = H.inverse() * y;

  H_x /= H_x(2);
  Hinv_y /= Hinv_y(2);

  return (H_x.head(2) - y.head(2)).squaredNorm() +
         (Hinv_y.head(2) - x.head(2)).squaredNorm();
}

class HomographySymmetricGeometricCostFunctor {
 public:
  HomographySymmetricGeometricCostFunctor(Mat &x1, Mat &x2)
      : x1_(x1),
        x2_(x2) {
  }

  template<typename T>
  bool operator()(const T *homography_parameters, T *residuals) const {
    typedef Eigen::Matrix<T, 3, 3> Mat3;
    typedef Eigen::Matrix<T, 2, 1> Vec2;

    Mat3 H(homography_parameters);

    T symmetric_error = T(0.0);

    for (int i = 0; i < x1_.cols(); i++) {
      Vec2 x1(T(x1_(0, i)), T(x1_(1, i)));
      Vec2 x2(T(x2_(0, i)), T(x2_(1, i)));

      symmetric_error += SymmetricGeometricDistance(H, x1, x2);
    }

    residuals[0] = symmetric_error;

    return true;
  }

  const Mat &x1_;
  const Mat &x2_;
};

void ComputeHomographyFromCorrespondences(Mat &x1, Mat &x2, Mat3 *H) {
  // Algebraic homography estimation
  Homography2DFromCorrespondencesLinear(x1, x2, H);

  // Refine matrix using Ceres minimizer
  ceres::Problem problem;

  HomographySymmetricGeometricCostFunctor *homography_symmetric_geometric_cost_function =
       new HomographySymmetricGeometricCostFunctor(x1, x2);

  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<
          HomographySymmetricGeometricCostFunctor,
          1, /* num_residuals */
          9>(homography_symmetric_geometric_cost_function),
      NULL,
      H->data());

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
  FundamentalSymmetricEpipolarCostFunctor(Mat &x1, Mat &x2)
      : x1_(x1),
        x2_(x2) {
  }

  template<typename T>
  bool operator()(const T *fundamental_parameters, T *residuals) const {
    typedef Eigen::Matrix<T, 3, 3> Mat3;
    typedef Eigen::Matrix<T, 2, 1> Vec2;

    Mat3 F(fundamental_parameters);

    T error = T(0.0);

    for (int i = 0; i < x1_.cols(); i++) {
      Vec2 x1(T(x1_(0, i)), T(x1_(1, i)));
      Vec2 x2(T(x2_(0, i)), T(x2_(1, i)));

      error += SymmetricEpipolarDistance(F, x1, x2);
    }

    residuals[0] = error;

    return true;
  }

  const Mat &x1_;
  const Mat &x2_;
};

void ComputeFundamentalFromCorrespondences(Mat &x1, Mat &x2, Mat3 *F)
{
  // Algebraic fundamental estimation
  NormalizedEightPointSolver(x1, x2, F);

  // Refine matrix using Ceres minimizer
  ceres::Problem problem;

  FundamentalSymmetricEpipolarCostFunctor *fundamental_symmetric_epipolar_cost_function =
       new FundamentalSymmetricEpipolarCostFunctor(x1, x2);

  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<
          FundamentalSymmetricEpipolarCostFunctor,
          1, /* num_residuals */
          9>(fundamental_symmetric_epipolar_cost_function),
      NULL,
      F->data());

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
  double lambda1 = log10((double) r);
  double lambda2 = log10((double) r * n);

  // lambda3 limits the residual error, and this paper
  // http://elvera.nue.tu-berlin.de/files/0990Knorr2006.pdf
  // suggests using lambda3 of 2
  double lambda3 = 2.0;

  // Compute squared standard deviation sigma2 of the error
  double mean_value = 0;
  for (int i = 0; i < n; i++)
    mean_value += e(i);
  mean_value /= (double) n;

  double sigma2 = 0.0;
  for (int i = 0; i < n; i++)
    sigma2 += Square(e(i) - mean_value);
  sigma2 /= n;

  // pre-compute for faster calculation of cycle body
  double rho_max = lambda3 * (r - d);

  // Actual GRIC computation
  double gric_result = 0.0;

  for (int i = 0; i < n; i++) {
    double rho = std::min(e(i) * e(i) / sigma2, rho_max);

    gric_result += rho;
  }

  gric_result += lambda1 * d * n;
  gric_result += lambda2 * k;

  return gric_result;
}

} // namespace

void SelectkeyframesBasedOnGRIC(Tracks &tracks, vector<int> &keyframes) {
  // Mirza Tahir Ahmed, Matthew N. Dailey
  // Robust key frame extraction for 3D reconstruction from video streams
  //
  // http://www.cs.ait.ac.th/~mdailey/papers/Tahir-KeyFrame.pdf

  int max_image = tracks.MaxImage();
  int image = 1, next_keyframe = 1;

  // Limit correspondence ratio from both sides.
  // On the one hand if number of correspondent features is too low,
  // triangulation will suffer.
  // On the other hand high correspondence likely means short baseline.
  // which also will affect om accuracy
  const double Tmin = 0.9;
  const double Tmax = 1.0;

  while (next_keyframe != -1) {
    int current_keyframe = next_keyframe;

    LG << "Found keyframe " << next_keyframe;

    keyframes.push_back(next_keyframe);
    image++;
    next_keyframe = -1;

    for (int candidate_image = current_keyframe + 1;
         candidate_image <= max_image;
         candidate_image++)
    {
      // Conjunction of all markers from both keyframes
      vector<Marker> all_markers = tracks.MarkersInBothImages(current_keyframe, candidate_image);

      // Match keypoints between frames current_keyframe and candidate_image
      vector<Marker> tracked_markers = tracks.MarkersForTracksInBothImages(current_keyframe, candidate_image);

      Mat x1, x2;
      CoordinatesForMarkersInImage(tracked_markers, current_keyframe, &x1);
      CoordinatesForMarkersInImage(tracked_markers, candidate_image, &x2);

      // Not enough points to construct fundamental matrix
      if (x1.cols() < 8 || x2.cols() < 8)
        continue;

      Mat3 H, F;
      ComputeHomographyFromCorrespondences(x1, x2, &H);
      ComputeFundamentalFromCorrespondences(x1, x2, &F);

      // TODO(sergey): Discard outlier matches

      // Correspondence ratio constraint
      int Tc = tracked_markers.size();
      int Tf = all_markers.size();
      double Rc = (double) Tc / (double) Tf;

      LG << "Correspondence between " << current_keyframe << " and " << candidate_image
         << ": " << Rc;

      if (Rc < Tmin || Rc > Tmax)
        continue;

      // Compute error values for homography and fundamental matrices
      Vec H_e, F_e;
      H_e.resize(x1.cols());
      F_e.resize(x1.cols());
      for (int i = 0; i < x1.cols(); i++) {
        Vec2 current_x1(x1(0, i), x1(1, i));
        Vec2 current_x2(x2(0, i), x2(1, i));

        H_e(i) = SymmetricGeometricDistance(H, current_x1, current_x2);
        F_e(i) = SymmetricEpipolarDistance(F, current_x1, current_x2);
      }

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
  }
}

} // namespace libmv

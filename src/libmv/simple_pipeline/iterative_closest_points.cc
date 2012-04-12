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

#include "libmv/logging/logging.h"
#include "libmv/simple_pipeline/iterative_closest_points.h"
#include "libmv/simple_pipeline/rigid_registration.h"

namespace libmv {

// creats two vectors of points in a way that points with euql indices
// are belong to the same pair (closest to each other)
static void FindClosestPoints(const vector<Vec3> &points_a,
                              const vector<Vec3> &points_b,
                              vector<Vec3> &closest_points_a,
                              vector<Vec3> &closest_points_b) {
  if (points_b.size() < points_a.size()) {
    FindClosestPoints(points_b, points_a, closest_points_b, closest_points_a);
    return;
  }

  for (int i = 0; i < points_a.size(); i++) {
    int other_index = -1;
    double distance;

    closest_points_a.push_back(points_a[i]);

    // TODO(sergey): optimize using AABB or K-D trees
    for (int j = 0; j < points_b.size(); j++) {
      double current_distance = (points_b[j] - points_a[i]).norm();

      if (other_index < 0 || current_distance < distance) {
        other_index = j;
        distance = current_distance;
      }
    }

    closest_points_b.push_back(points_b[other_index]);
  }
}

static void TransformPointCloud(const vector<Vec3> &points,
                                vector<Vec3> &transformed_points,
                                const Mat3 &R,
                                const Mat3 &S,
                                const Vec3 &t) {
  for (int i = 0; i < points.size(); i++) {
    Vec3 X = R * S * points[i] + t;
    transformed_points.push_back(X);
  }
}

void IterativeClosestPoints(const vector<Vec3> &reference_points,
                            const vector<Vec3> &points,
                            Mat3 &R,
                            Vec3 &S,
                            Vec3 &t,
                            int max_iterations,
                            double threshold) {
  // Store scale as matrix for easier math, convert to scale vector at the end
  Mat3 SMat;

  R.setIdentity();
  S.setIdentity();
  t.setZero();
  SMat.setIdentity();

  LG << "iteractive closest points on maximum iterations " << max_iterations << ", threshold " << threshold;

  double previous_error = 0;

  for (int it = 0; it < max_iterations; it++) {
    vector<Vec3> closest_reference_points, closest_points;
    vector<Vec3> transformed_points;

    // Step 0: first transform point cloud with currently calculated transform
    TransformPointCloud(points, transformed_points, R, SMat, t);

    // Step 1: find pairs of points which are closest to each other
    FindClosestPoints(reference_points, transformed_points,
                      closest_reference_points,
                      closest_points);

    if (closest_points.size() == 0) {
      // No points to calculate transformation between, stop iterations
      break;
    }

    Mat3 current_R;
    Vec3 current_t;
    Vec3 current_S;
    Mat3 current_SMat;

    LG << "iteractive closest points: iteration " << (it + 1) <<
      " for " << closest_points.size() << " points";

    // Step 2: find current rigid transformation and apply it
    double error = RigidRegistration(closest_reference_points,
                                     closest_points,
                                     current_R,
                                     current_S,
                                     current_t);

    // Convert scale vector to scale matrix
    current_SMat.setIdentity();
    current_SMat(0, 0) = current_S(0);
    current_SMat(1, 1) = current_S(1);
    current_SMat(2, 2) = current_S(2);

    // Update final transformation
    R *= current_R;
    SMat *= current_SMat;
    t += current_t;

    LG << "Average error for iteration is " << error;

    // Step 3: check threshold
    if (fabs(previous_error - error) < threshold) {
      LG << "iterative closest points: average error is below threshold, finishing";
      break;
    }

    previous_error = error;
  }

  // Convert scale matrix to vector of per-component scale values
  S(0) = SMat(0, 0);
  S(1) = SMat(1, 1);
  S(2) = SMat(2, 2);

  LG << "iterative closest points: final transformation is " << std::endl << R <<
    std::endl << S << std::endl << t;
}

}  // namespace libmv

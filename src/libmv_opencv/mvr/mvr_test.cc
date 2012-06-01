// Copyright (c) 2007, 2008 libmv authors.
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

//Todo: Do something about relative paths later
#include "../../libmv/logging/logging.h"
#include "../../libmv/multiview/fundamental.h"
#include "../../libmv/multiview/projection.h"
#include "../../libmv/multiview/test_data_sets.h"
#include "../../libmv/numeric/numeric.h"
#include "../../libmv/multiview/twoviewtriangulation.h"
#include "testing/testing.h"

#include "mvr.h"

//namespace {
using namespace libmv;
using namespace libmv_opencv;

// TODO: modify this to test mvr.cc stuff

TEST(Mvr, TwoViewTriangulationIdeal) {
  TwoViewDataSet d = TwoRealisticCameras();

  // Compute essential matrix.
  Mat3 E;
  EssentialFromFundamental(d.F, d.K1, d.K2, &E);
  Mat3 K1_inverse = d.K1.inverse();
  Mat3 K2_inverse = d.K2.inverse();

  //Transform the system so that camera 1 is in its canonical form [I|0]
  Eigen::Transform< double, 3, Eigen::Affine > Hcanonical =
    Eigen::Translation3d(d.t1)*d.R1;
  Hcanonical = Hcanonical.inverse();

  Mat34 P2;
  P2.block<3,3>(0,0) = d.R2;
  P2.block<3,1>(0,3) = d.t2;

  P2 = P2*Hcanonical.matrix();

  for (int i = 0; i < d.X.cols(); ++i) {

    Vec2 x1, x2;
    MatrixColumn(d.x1, i, &x1);
    MatrixColumn(d.x2, i, &x2);
    x1 = ImageToNormImageCoordinates(K1_inverse,x1);
    x2 = ImageToNormImageCoordinates(K2_inverse,x2);

    Vec3 X_estimated, X_gt;
    MatrixColumn(d.X, i, &X_gt);
//    TwoViewTriangulationIdeal(x1, x2, P2, E, &X_estimated);

    //Todo: Fix undefined ref error
//    mvr_test.cc:(.text+0x3d1): undefined reference to `libmv_opencv::triangulatePoints(Eigen::Matrix<double, 2, 1, 0, 2, 1> const&, Eigen::Matrix<double, 2, 1, 0, 2, 1> const&, Eigen::Matrix<double, 3, 4, 0, 3, 4> const&, Eigen::Matrix<double, 3, 3, 0, 3, 3> const&, Eigen::Matrix<double, 3, 1, 0, 3, 1>*, bool)'
//  triangulatePoints(x1, x2, P2, E, &X_estimated, true);
    test(); // mvr.h is not included? complains undefined func?

    X_estimated = Hcanonical*X_estimated;
    EXPECT_NEAR(0, DistanceLInfinity(X_estimated, X_gt), 1e-8);
  }
}


//} // namespace


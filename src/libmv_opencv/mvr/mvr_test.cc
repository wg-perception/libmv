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

//#include "mvr.h"
//
//// libmv includes
////Todo: Do something about relative paths later
#include "../../libmv/logging/logging.h"
#include "../../libmv/multiview/fundamental.h"
#include "../../libmv/multiview/projection.h"
#include "../../libmv/multiview/test_data_sets.h"
#include "../../libmv/numeric/numeric.h"
#include "../../libmv/multiview/twoviewtriangulation.h"
#include "../../libmv/reconstruction/reconstruction.h"
#include "../../libmv/reconstruction/euclidean_reconstruction.h"

#include "../../libmv/correspondence/matches.h"
#include "../../libmv/image/image_converter.h"

#include <Eigen/Core>
#include "libmv/multiview/random_sample.h"
#include <list>
#include "libmv/logging/logging.h"
#include "libmv/multiview/projection.h"
#include "libmv/multiview/random_sample.h"
#include "libmv/multiview/test_data_sets.h"
#include "libmv/numeric/numeric.h"
#include "libmv/reconstruction/reconstruction.h"
#include "libmv/reconstruction/euclidean_reconstruction.h"
#include "libmv/reconstruction/mapping.h"
#include "libmv/reconstruction/optimization.h"
#include "libmv/reconstruction/projective_reconstruction.h"

// gtest headers
#include "testing/testing.h"

#include <opencv2/core/core.hpp>
#include <iostream>
#include <string>
using namespace cv;
using namespace std;

using namespace libmv;
using namespace Eigen;
using namespace cv;

//using namespace libmv_opencv;

#if 1
TEST(Mvr, TestYAML)
{
  std::vector<cv::Point2d> imgpts, imgpts2;
  imgpts.push_back(Point2d(1.0, 2.0));
  imgpts.push_back(Point2d(2.0, 3.0));
  imgpts.push_back(Point2d(3.0, 2.0));
  { //write
    cv::Mat R = Mat_<uchar>::eye(3, 3), T = Mat_<double>::zeros(3, 1);
    FileStorage fs("opencv.yml", FileStorage::WRITE);

    fs << "R" << R; // cv::Mat
    fs << "T" << T;

    fs << "imgpts" << imgpts;

    fs.release(); // explicit close
    cout << "Write Done." << endl;
  }

  { //read
    string filename = "opencv.yml";
    cout << endl << "Reading: " << endl;
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
//    Mat R, T;
//
//    fs["R"] >> R; // Read cv::Mat
//    fs["T"] >> T;
//
//    cout << endl << "R = " << R << endl;
//    cout << "T = " << T << endl;

    fs["imgpts"] >> imgpts2;
    cout << "imgpts = " << imgpts2[0] << endl;
    cout << "imgpts = " << imgpts2[1] << endl;

  }
  { //test
    string filename = "rnd_N10_F3.yml";
    int nviews = 3;
    int npts = 10;

    cout << endl << "Reading: " << endl;
    FileStorage fs;
    fs.open(filename, FileStorage::READ);
    cv::Mat S, W1, W2, P1, P2;
    fs["S"] >> S;
    fs["W1"] >> W1;
    fs["W2"] >> W2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;

    cv::Mat W[3];
    fs["W1"] >> W[0];
    fs["W2"] >> W[1];
    fs["W2"] >> W[2];

    cout << endl << "P1 = " << P1 << endl;
    cout << endl << "P2 = " << P2 << endl;
    cout << endl << "W1 = " << W1 << endl;
    cout << endl << "W2 = " << W2 << endl;
    cout << endl << "S = " << S << endl;

    // libmv api

    // Matching pts
    Matches matches;
    for (int v = 0; v < nviews; ++v)
    {
      for (int p = 0; p < npts; ++p)
      {
        PointFeature * feature = new PointFeature(W[v].at<double>(0, p), W[v].at<double>(1, p));
        matches.Insert(v, p, feature);
      }
    }

    // Camera matrices
    Eigen::MatrixXd K1_libmv = Eigen::MatrixXd::Identity(3, 3);
    Eigen::MatrixXd K2_libmv = Eigen::MatrixXd::Identity(3, 3);

    // Image size -- has probs
    Vec2u image_size1;
    image_size1 << (unsigned int) 1, (unsigned int) 1; //this is an unsinged int???

    Reconstruction recons;
    InitialReconstructionTwoViews(matches, 0, 1, K1_libmv, K1_libmv, image_size1, image_size1, &recons);

    // iterate and read out 3D pts from recons;
    cout << endl << "recons.GetNumberStructures()=" << recons.GetNumberStructures() << endl;

    //gets 6 out of 10?
//		PointStructure *point= new PointStructure();
//		for(int p; p<recons.GetNumberStructures();++p)
//		{
//			point=recons.GetStructure(p);
////			point->coords()
//		}
  }
}
#endif

#if 0
TEST(Mvr, TwoViewTriangulationIdealHomogenous)
{
  TwoViewDataSet d = TwoRealisticCameras();

// Compute essential matrix.
  Mat3 E;
  EssentialFromFundamental(d.F, d.K1, d.K2, &E);
  Mat3 K1_inverse = d.K1.inverse();
  Mat3 K2_inverse = d.K2.inverse();

//Transform the system so that camera 1 is in its canonical form [I|0]
  Eigen::Transform<double, 3, Eigen::Affine> Hcanonical = Eigen::Translation3d(
      d.t1) * d.R1;
  Hcanonical = Hcanonical.inverse();

  Mat34 P2;
  P2.block<3, 3>(0, 0) = d.R2;
  P2.block<3, 1>(0, 3) = d.t2;

  P2 = P2 * Hcanonical.matrix();

  for (int i = 0; i < d.X.cols(); ++i)
  {

    Vec2 x1, x2;
    MatrixColumn(d.x1, i, &x1);
    MatrixColumn(d.x2, i, &x2);
    x1 = ImageToNormImageCoordinates(K1_inverse, x1);
    x2 = ImageToNormImageCoordinates(K2_inverse, x2);

    Vec3 X_estimated, X_gt;
    MatrixColumn(d.X, i, &X_gt);

//          TwoViewTriangulationIdeal(x1, x2, P2, E, &X_estimated);

// Convert to general format
    int nviews = 2;
    Mat2X xs(2, nviews);
    xs.col(1) = x1;
    xs.col(2) = x2;
    vector<Mat34> Ps(nviews);

// Check the order here
    Ps[0] = d.

    X_estimated = Hcanonical * X_estimated;
    EXPECT_NEAR(0, DistanceLInfinity(X_estimated, X_gt), 1e-8);
  }
}

TEST(Mvr, TwoViewTriangulationIdealEucledian)
{
  TwoViewDataSet d = TwoRealisticCameras();

// Compute essential matrix.
  Mat3 E;
  EssentialFromFundamental(d.F, d.K1, d.K2, &E);
  Mat3 K1_inverse = d.K1.inverse();
  Mat3 K2_inverse = d.K2.inverse();

//Transform the system so that camera 1 is in its canonical form [I|0]
  Eigen::Transform<double, 3, Eigen::Affine> Hcanonical = Eigen::Translation3d(
      d.t1) * d.R1;
  Hcanonical = Hcanonical.inverse();

  Mat34 P2;
  P2.block<3, 3>(0, 0) = d.R2;
  P2.block<3, 1>(0, 3) = d.t2;

  P2 = P2 * Hcanonical.matrix();

  for (int i = 0; i < d.X.cols(); ++i)
  {

    Vec2 x1, x2;
    MatrixColumn(d.x1, i, &x1);
    MatrixColumn(d.x2, i, &x2);
    x1 = ImageToNormImageCoordinates(K1_inverse, x1);
    x2 = ImageToNormImageCoordinates(K2_inverse, x2);

    Vec3 X_estimated, X_gt;
    MatrixColumn(d.X, i, &X_gt);

    // Consilidate data to multiview format
    // used in triangulatePoints()
    int nviews=2;
    Mat2X xs(2, nviews);
    xs.col(1)=x1;
    xs.col(2)=x2;
    // Ok X_estimated is Vec3 but NViewTriangulate has Vec4 ... now what?

    triangulatePoints(x1, x2, P2, E, &X_estimated, true);
    triangulatePoints(xs, Ps, &X, false);

    X_estimated = Hcanonical * X_estimated;
    EXPECT_NEAR(0, DistanceLInfinity(X_estimated, X_gt), 1e-8);
  }
}

TEST(Mvr, FiveViewsHomogeneous)
{
  int nviews = 5;
  int npoints = 6;
  NViewDataSet d = NRealisticCamerasFull(nviews, npoints);

// Collect P matrices together.
  vector<Mat34> Ps(nviews);
  for (int j = 0; j < nviews; ++j)
  {
    Ps[j] = d.P(j);
  }

  for (int i = 0; i < npoints; ++i)
  {
    // Collect the image of point i in each frame.
    Mat2X xs(2, nviews);
    for (int j = 0; j < nviews; ++j)
    {
      xs.col(j) = d.x[j].col(i);
    }
    Vec4 X;
    triangulatePoints(xs, Ps, &X, false);

    // Check reprojection error. Should be nearly zero.
    for (int j = 0; j < nviews; ++j)
    {
      Vec3 x_reprojected = Ps[j] * X;
      x_reprojected /= x_reprojected(2);
      double error = (x_reprojected.head(2) - xs.col(j)).norm();
      EXPECT_NEAR(error, 0.0, 1e-9);
    }
  }
}

#endif


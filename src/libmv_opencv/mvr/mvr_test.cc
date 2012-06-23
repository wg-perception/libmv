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


void GenerateMatchesFromNViewDataSet(const NViewDataSet &d,
                                     int noutliers,
                                     Matches *matches,
                                     std::list<Feature *> *list_features) {
  Matches::TrackID track_id;
  libmv::vector<int> wrong_matches;
  for (size_t n = 0; n < d.n; ++n) {
    //std::cout << "n -> "<< d.x[n]<< std::endl;
    // Generates wrong matches
    UniformSample(noutliers, d.X.cols(), &wrong_matches);
    //std::cout << "Features :"<<d.x[n].transpose()<<"\n";
    for (size_t p = 0; p < d.x[n].cols(); ++p) {
      PointFeature * feature = new PointFeature(d.x[n](0, p), d.x[n](1, p));
      list_features->push_back(feature);
      track_id = p;
      if (p < noutliers) {
        track_id = wrong_matches[p];
      }
      matches->Insert(n, track_id, feature);
    }
  }
}

TEST(CalibratedReconstruction, TestSynthetic6FullViews) {
  // TODO(julien) maybe a better check is the relative motion
  int nviews = 6;
  int npoints = 100;
  int noutliers = 0.4*npoints;// 30% of outliers
  NViewDataSet d = NRealisticCamerasFull(nviews, npoints);

  Mat4X X;
  EuclideanToHomogeneous(d.X, &X);

  Reconstruction reconstruction;
  // Create the matches
  Matches matches;
  Matches matches_inliers;
  std::list<Feature *> list_features;
  GenerateMatchesFromNViewDataSet(d, noutliers, &matches, &list_features);

  // We fix the gauge by setting the pose of the initial camera to the true pose

  Vec2u image_size1;
  image_size1 << d.K[0](0, 2), d.K[0](1, 2);

  Vec2u image_size2;
  image_size2 << d.K[0](0, 2), d.K[0](1, 2);

  std::cout << "Proceed Initial Motion Estimation" << std::endl;
  // Proceed Initial Motion Estimation
  bool recons_ok = true;
  recons_ok = InitialReconstructionTwoViews(matches,
                                            0, 1,
                                            d.K[0], d.K[1],
                                            image_size1, image_size2,
                                            &reconstruction);
  PinholeCamera * camera = NULL;
  EXPECT_EQ(reconstruction.GetNumberCameras(), 2);
  camera = dynamic_cast<PinholeCamera *>(reconstruction.GetCamera(0));
  EXPECT_TRUE(camera != NULL);
  /*
  PinholeCamera * camera0 = new PinholeCamera(d.K[0], d.R[0], d.t[0]);
  // These are the expected precision of the results
  // Dont expect much since for now
  //  - this is an incremental approach
  //  - the 3D structure is not retriangulated when new views are estimated
  //  - there is no optimization!
  const double kPrecisionOrientationMatrix = 3e-2;
  const double kPrecisionPosition          = 3e-2;

  // TODO(julien) Check the reconstruction!
  EXPECT_MATRIX_PROP(camera->orientation_matrix(), d.R[0], 1e-8);
  EXPECT_MATRIX_PROP(camera->position(), d.t[0], 1e-8);

  double rms = RootMeanSquareError(d.x[0], X, camera->projection_matrix());
  std::cout << "RMSE Camera 0 = " << rms << std::endl;

  camera = dynamic_cast<PinholeCamera *>(reconstruction.GetCamera(1));
  EXPECT_TRUE(camera != NULL);

  // This is a monocular reconstruction
  // We fix the scale
  Mat3 dR = d.R[0].transpose()*d.R[1];
  Vec3 dt = d.R[0].transpose() * (d.t[1] - d.t[0]);
  double dt_norm_real = dt.norm();
  dt = camera0->orientation_matrix().transpose() *
    (camera->position() - camera0->position());
  dt *= dt_norm_real/dt.norm();
  camera->set_position(camera0->orientation_matrix() * dt
    + camera0->position());

  EXPECT_MATRIX_PROP(camera->orientation_matrix(), d.R[1],
                     kPrecisionOrientationMatrix);
  EXPECT_MATRIX_PROP(camera->position(), d.t[1], kPrecisionPosition);
  rms = RootMeanSquareError(d.x[1], X, camera->projection_matrix());
  std::cout << "RMSE Camera 1 = " << rms << std::endl;

  std::cout << "Proceed Initial Intersection" << std::endl;
  // Proceed Initial Intersection
  uint nInliers_added = 0;
  size_t minimum_num_views_batch = 2;
  nInliers_added = PointStructureTriangulationCalibrated(matches_inliers, 1,
                                                        minimum_num_views_batch,
                                                        &reconstruction);
  ASSERT_NEAR(nInliers_added, npoints - noutliers, 1);
  // TODO(julien) check imqges sizes, etc.
  size_t minimum_num_views_incremental = 3;
  Mat3 R;
  Vec3 t;
  // Checks the incremental reconstruction
  for (int i = 2; i < nviews; ++i) {
    std::cout << "Proceed Incremental Resection" << std::endl;
    // Proceed Incremental Resection
    CalibratedCameraResection(matches, i, d.K[i],
                              &matches_inliers, &reconstruction);

    EXPECT_EQ(reconstruction.GetNumberCameras(), i+1);
    camera = dynamic_cast<PinholeCamera *>(reconstruction.GetCamera(i));
    EXPECT_TRUE(camera != NULL);
    EXPECT_MATRIX_PROP(camera->orientation_matrix(), d.R[i],
                       kPrecisionOrientationMatrix);
    EXPECT_MATRIX_PROP(camera->position(), d.t[i], kPrecisionPosition);

    std::cout << "Proceed Incremental Intersection" << std::endl;
    // Proceed Incremental Intersection
    nInliers_added = PointStructureTriangulationCalibrated(
     matches_inliers, i,
     minimum_num_views_incremental,
     &reconstruction);
    ASSERT_NEAR(nInliers_added, 0, 1);

    // TODO(julien) Check the rms with the reconstructed structure
    rms = RootMeanSquareError(d.x[i], X, camera->projection_matrix());
    std::cout << "RMSE Camera " << i << " = " << rms << std::endl;
    // TODO(julien) Check the 3D structure coordinates and inliers
  }
  // Performs bundle adjustment
  rms = MetricBundleAdjust(matches_inliers, &reconstruction);
  std::cout << " Final RMSE = " << rms << std::endl;
  // TODO(julien) Check the results of BA*/
  // clear the cameras, structures and features
  reconstruction.ClearCamerasMap();
  reconstruction.ClearStructuresMap();
  std::list<Feature *>::iterator features_iter = list_features.begin();
  for (; features_iter != list_features.end(); ++features_iter)
    delete *features_iter;
  list_features.clear();
}

#if 0
TEST(Mvr, TestYAML) {
//	{ //write
//		Mat R = Mat_ < uchar > ::eye(3, 3), T = Mat_<double>::zeros(3, 1);
//		FileStorage fs("opencv.yml", FileStorage::WRITE);
//
//		fs << "R" << R; // cv::Mat
//		fs << "T" << T;
//
//		fs.release(); // explicit close
//		cout << "Write Done." << endl;
//	}
//
//	{ //read
//		string filename = "opencv.yml";
//		cout << endl << "Reading: " << endl;
//		FileStorage fs;
//		fs.open(filename, FileStorage::READ);
//		Mat R, T;
//
//		fs["R"] >> R; // Read cv::Mat
//		fs["T"] >> T;
//
//		cout << endl << "R = " << R << endl;
//		cout << "T = " << T << endl << endl;
//	}
	{ //test
		string filename = "rnd_N5_F3.yml";
		int nviews = 3;
		int npts = 5;

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
		for (int v = 0; v < nviews; ++v) {
			for (int p = 0; p < npts; ++p) {
				PointFeature * feature = new PointFeature(W[v].at<double>(0, p),
						W[v].at<double>(0, p));
				matches.Insert(v, p, feature);
			}
		}

		// Camera matrices
		Eigen::MatrixXf K1_libmv = Eigen::MatrixXd::Identity(3, 3);
		Eigen::MatrixXf K2_libmv = Eigen::MatrixXd::Identity(3, 3);

		// Image size -- has probs
		Vec2u image_size1;
		image_size1 << (unsigned int) 1, (unsigned int) 1; //this is an unsinged int???

		Reconstruction recons;
		InitialReconstructionTwoViews(matches, 0, 1, K1_libmv, K1_libmv,
				image_size1, image_size1, &recons);

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


// Copyright (c) 2011 libmv authors.
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
/**
 * Mosaicing_video is a tool for making a mosaic from a video (list of images).
 * It uses the following simple approach:
 * From the given features matches, the chained relatives matrices are estimated
 * (euclidean or homography) and images are warped and written in a global 
 * mosaic image. The overlapping zones are blended and the last image takes 50% 
 * of the blend.
 * The mozaic is then saved into a file.
 * 
 * TODO(julien) Mosaicing of an image set = same as this but without the 
 *              recursive $qi = Ai-1 * ...* A1 q1$!
 *              Use the same graph traversal as in image_selection.
 */
#include <algorithm>
#include <string>
#include <vector>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "libmv/correspondence/feature.h"
#include "libmv/correspondence/import_matches_txt.h"
#include "libmv/correspondence/matches.h"
#include "libmv/correspondence/tracker.h"
#include "libmv/image/image.h"
#include "libmv/image/image_converter.h"
#include "libmv/image/image_sequence_io.h"
#include "libmv/image/cached_image_sequence.h"
#include "libmv/image/sample.h"
#include "libmv/multiview/robust_affine.h"
#include "libmv/multiview/robust_euclidean.h"
#include "libmv/multiview/robust_homography.h"
#include "libmv/multiview/robust_similarity.h"
#include "libmv/logging/logging.h"

enum eGEOMETRIC_TRANSFORMATION  {
  EUCLIDEAN = 0,// Euclidean 2D (3 dof: 2 translations (x, y) + 1 rotation)
  SIMILARITY ,  // Similarity 2D (4 dof: EUCLIDEAN + scale)
  AFFINE,       // Affinity 2D (6 dof)
  HOMOGRAPHY,   // Homography 2D (8 dof: general planar case)
};

DEFINE_string(m, "matches.txt", "Matches input file");
DEFINE_string(o, "mosaic.jpg", "Mosaic output file");
DEFINE_int32 (transformation, SIMILARITY, "Transformation type:\n\t 0: \
Euclidean\n\t 1:Similarity\n\t 2:Affinity\n\t 3:Homography");
DEFINE_double(blending_ratio, 0.7, "Blending ratio");
DEFINE_bool(draw_lines, false, "Draw image bounds");
             
using namespace libmv;

cv::Matx33d
MatToMatx(const Mat3 & mat3)
{
  cv::Matx33d matx;
  for (char j = 0; j < 3; ++j)
    for (char i = 0; i < 3; ++i)
      matx(j, i) = mat3(j, i);
  return matx;
}

/**
 * Computes relative euclidean matrices
 *
 * \param matches The 2D features matches
 * \param Ss Vector of relative similarity matrices such that 
 *        $q2 = E1 q1$ and $qi = Ei-1 * ...* E1 q1$
 *        where qi is a point in the image i
 *        and q1 is its position in the image 1
 * \param outliers_prob The outliers probability [0, 1[
 * \param max_error_2d The maximun 2D error in pixel
 *
 * TODO(julien) put this in reconstruction
 */
void ComputeRelativeEuclideanMatrices(const Matches &matches,
                                      vector<cv::Matx33d> *Es,
                                      double outliers_prob = 1e-2,
                                      double max_error_2d = 1) {
  Es->reserve(matches.NumImages() - 1);
  Mat3 E;
  vector<Mat> xs2;
  std::set<Matches::ImageID>::const_iterator image_iter =
    matches.get_images().begin();
  std::set<Matches::ImageID>::const_iterator prev_image_iter = image_iter;
  image_iter++;
  for (;image_iter != matches.get_images().end(); ++image_iter) {
    TwoViewPointMatchMatrices(matches, *prev_image_iter, *image_iter, &xs2);
      if (xs2[0].cols() >= 2) {
        Euclidean2DFromCorrespondences2PointRobust(xs2[0], xs2[1], 
                                                   max_error_2d , 
                                                   &E, NULL, 
                                                   outliers_prob);
        Es->push_back(MatToMatx(E));
        VLOG(2) << "E = " << std::endl << cv::Mat(MatToMatx(E)) << std::endl;
      } // TODO(julien) what to do when no enough points?
    ++prev_image_iter;
  }
}

/**
 * Computes relative similarity matrices
 *
 * \param matches The 2D features matches
 * \param Ss Vector of relative similarity matrices such that 
 *        $q2 = S1 q1$ and $qi = Si-1 * ...* S1 q1$
 *        where qi is a point in the image i
 *        and q1 is its position in the image 1
 * \param outliers_prob The outliers probability [0, 1[
 * \param max_error_2d The maximun 2D error in pixel
 *
 * TODO(julien) put this in reconstruction
 */
void ComputeRelativeSimilarityMatrices(const Matches &matches,
                                       vector<cv::Matx33d> *Ss,
                                       double outliers_prob = 1e-2,
                                       double max_error_2d = 1) {
  Ss->reserve(matches.NumImages() - 1);
  Mat3 S;
  vector<Mat> xs2;
  std::set<Matches::ImageID>::const_iterator image_iter =
    matches.get_images().begin();
  std::set<Matches::ImageID>::const_iterator prev_image_iter = image_iter;
  image_iter++;
  for (;image_iter != matches.get_images().end(); ++image_iter) {
    TwoViewPointMatchMatrices(matches, *prev_image_iter, *image_iter, &xs2);
      if (xs2[0].cols() >= 2) {
        Similarity2DFromCorrespondences2PointRobust(xs2[0], xs2[1], 
                                                    max_error_2d , 
                                                    &S, NULL, 
                                                    outliers_prob);
        Ss->push_back(MatToMatx(S));
        VLOG(2) << "S = " << std::endl << cv::Mat(MatToMatx(S)) << std::endl;
      } // TODO(julien) what to do when no enough points?
    ++prev_image_iter;
  }
}

/**
 * Computes relative affine matrices
 *
 * \param matches The 2D features matches
 * \param As A vector of relative affine matrices such that 
 *        $q2 = A1 q1$ and $qi = Ai-1 * ...* A1 q1$
 *        where qi is a point in the image i
 *        and q1 is its position in the image 1
 * \param outliers_prob The outliers probability [0, 1[
 * \param max_error_2d The maximun 2D error in pixel
 *
 * TODO(julien) put this in reconstruction
 */
void ComputeRelativeAffineMatrices(const Matches &matches,
                                   vector<cv::Matx33d> *As,
                                   double outliers_prob = 1e-2,
                                   double max_error_2d = 1) {
  As->reserve(matches.NumImages() - 1);
  Mat3 A;
  vector<Mat> xs2;
  std::set<Matches::ImageID>::const_iterator image_iter =
    matches.get_images().begin();
  std::set<Matches::ImageID>::const_iterator prev_image_iter = image_iter;
  image_iter++;
  for (;image_iter != matches.get_images().end(); ++image_iter) {
    TwoViewPointMatchMatrices(matches, *prev_image_iter, *image_iter, &xs2);
      if (xs2[0].cols() >= 3) {
        Affine2DFromCorrespondences3PointRobust(xs2[0], xs2[1], 
                                                max_error_2d , 
                                                &A, NULL, 
                                                outliers_prob);
        As->push_back(MatToMatx(A));
        VLOG(2) << "A = " << std::endl << cv::Mat(MatToMatx(A)) << std::endl;
      } // TODO(julien) what to do when no enough points?
    ++prev_image_iter;
  }
}

/**
 * Computes relative homography matrices
 *
 * \param matches The 2D features matches
 * \param Hs A vector of relative homography matrices such that 
 *        $q2 = H1 q1$ and $qi = Hi-1 * ...* H1 q1$
 *        where qi is a point in the image i
 *        and q1 is its position in the image 1
 * \param outliers_prob The outliers probability [0, 1[
 * \param max_error_2d The maximun 2D error in pixel
 *
 * TODO(julien) Put this in reconstruction
 */
void ComputeRelativeHomographyMatrices(const Matches &matches,
                                       vector<cv::Matx33d> *Hs,
                                       double outliers_prob = 1e-2,
                                       double max_error_2d = 1) {
  Hs->reserve(matches.NumImages() - 1);
  Mat3 H;
  vector<Mat> xs2;
  std::set<Matches::ImageID>::const_iterator image_iter =
    matches.get_images().begin();
  std::set<Matches::ImageID>::const_iterator prev_image_iter = image_iter;
  image_iter++;
  for (;image_iter != matches.get_images().end(); ++image_iter) {
    TwoViewPointMatchMatrices(matches, *prev_image_iter, *image_iter, &xs2);
      if (xs2[0].cols() >= 4) {
        Homography2DFromCorrespondences4PointRobust(xs2[0], xs2[1], 
                                                    max_error_2d, 
                                                    &H, NULL, 
                                                    outliers_prob);
        Hs->push_back(MatToMatx(H));
        VLOG(2) << "H = " << std::endl << cv::Mat(MatToMatx(H)) << std::endl;
      } // TODO(julien) what to do when no enough points?
    ++prev_image_iter;
  }
}

/**
 * Computes the global bounding box of a set of image warps.
 *
 * \param Hs The 2D relative warp matrices
 * \param images_size The common image size (width, height)
 * \param bbox The global bounding box (xmin, xmax, ymin, ymax)
 *
 * TODO(julien) put this in image/image_warp? 
 */
void ComputeGlobalBoundingBox(const Vec2u &images_size,
                              const vector<cv::Matx33d> &Hs,
                              Vec4i *bbox) {
  cv::Matx33d H = cv::Matx33d::eye();
  cv::Matx34d q_bounds = cv::Matx34d::zeros();
  q_bounds(0, 2) = images_size(0);
  q_bounds(0, 3) = images_size(0);
  q_bounds(1, 1) = images_size(1);
  q_bounds(1, 2) = images_size(1);
  q_bounds(0, 0) = 1;
  q_bounds(0, 1) = 1;
  q_bounds(0, 2) = 1;
  q_bounds(0, 3) = 1;

  cv::Matx<double, 3, 1> q;
  for (size_t i = 0; i < Hs.size(); ++i) {
    H = Hs[i].inv() * H;
    H = H * (1.0 / H(2, 2));
    VLOG(1) << "H = " << std::endl << cv::Mat(H) << std::endl;
    for (int i = 0; i < 4; ++i)
    {
      q = H * q_bounds.col(i);
      q = q * (1.0 / q(2));
      q(0) = ceil0<double>(q(0));
      q(1) = ceil0<double>(q(1));
      if (q(0) < (*bbox)(0))
        (*bbox)(0) = q(0);
      else if (q(0) > (*bbox)(1))
        (*bbox)(1) = q(0);
      if (q(1) < (*bbox)(2))
        (*bbox)(2) = q(1);
      else if (q(1) > (*bbox)(3))
        (*bbox)(3) = q(1);
    }
  }
  // HACK to protect from huge memory allocation
  (*bbox)(0) = std::max((int)-5e3, (*bbox)(0));
  (*bbox)(2) = std::max((int)-5e3, (*bbox)(2));
  (*bbox)(1) = std::min((int) 5e3, (*bbox)(1));
  (*bbox)(3) = std::min((int) 5e3, (*bbox)(3));
}

/**
 * Builds a mosaic of a list of image files.
 * 
 * \param image_files The input image files
 * \param Hs The 2D relative warp matrices
 * \param blending_ratio The blending ratio for overlapping zones, 
 *        a typical value is 0.5
 * \param draw_lines If true, the images bounds are drawn
 * \param mosaic The ouput mosaic image
 * 
 * TODO(julien) This rendering doesn't scale well?
 */
void BuildMosaic(const std::vector<std::string> &image_files,
                 const vector<cv::Matx33d> &Hs,
                 float blending_ratio,
                 bool draw_lines,
                 cv::Mat & mosaic) {
  assert(image_files.size() == Hs.size() - 1);
  
  // Get the size of the first image
  Vec2u images_size;
  cv::Mat imageArrayBytes = cv::imread(image_files[0], 0);
  images_size << imageArrayBytes.cols, imageArrayBytes.rows;
  unsigned int depth = imageArrayBytes.depth();

  Vec4i bbox;
  VLOG(0) << "Computing global bounding box..." << std::endl;
  ComputeGlobalBoundingBox(images_size, Hs, &bbox);
  VLOG(0) << "Computing global bounding box...[DONE]." << std::endl;
  VLOG(0) << "bbox: " << bbox.transpose() << std::endl;
  assert(bbox(1) < bbox(0));
  assert(bbox(3) < bbox(2));
  
  cv::Matx33d H = cv::Matx33d::eye();
  const unsigned int w = bbox(1) - bbox(0);
  const unsigned int h = bbox(3) - bbox(2);

  VLOG(0) << "Image size: h=" << h << " "
          << "w="             << w << " "
          << "d="             << depth << std::endl;

  // Register everyone so that the min (x, y) are (0, 0)
  cv::Matx33d Hreg;
  Hreg << 1, 0, -bbox(0),
          0, 1, -bbox(2),
          0, 0, 1;
  //for (size_t i = 0; i < image_files.size() / 2; ++i)
  //  Hreg = Hreg * Hs[i];
  cv::Scalar lines_color(255,255,255);
  FloatImage *image = NULL;
  ImageCache cache;
  cv::Ptr<ImageSequence> source(ImageSequenceFromFiles(image_files, &cache));
  for (size_t i = 0; i < image_files.size(); ++i) {
    if (i > 0)
      H = Hs[i - 1].inv() * H;
    image = source->GetFloatImage(i);
    if (image) {
      cv::Mat image_cv, image_cv_warp;
      Image2Mat(image, image_cv);
      if (draw_lines)
      {
        cv::line(image_cv, cv::Point2f(0, 0), cv::Point2f(image_cv.cols - 1, 0), lines_color);
        cv::line(image_cv, cv::Point2f(0, image_cv.cols - 1), cv::Point2f(image_cv.rows - 1, image_cv.cols - 1),
                 lines_color);
        cv::line(image_cv, cv::Point2f(image_cv.rows - 1, image_cv.cols - 1), cv::Point2f(image_cv.rows - 1, 0),
                 lines_color);
        cv::line(image_cv, cv::Point2f(image_cv.rows - 1, 0), cv::Point2f(0, 0), lines_color);
      }
      cv::warpPerspective(image_cv, H, image_cv_warp, image_cv.size());
      mosaic = (1 - blending_ratio) * image_cv + blending_ratio * image_cv_warp;
    }
    source->Unpin(i);
  }
}

int main(int argc, char **argv) {

  std::string usage ="Creates a mosaic from a video.\n";
  usage += "Usage: " + std::string(argv[0]) + " IMAGE1 [IMAGE2 ... IMAGEN] ";
  usage += "-m MATCHES.txt [-o MOSAIC_IMAGE]";
  usage += "\t - IMAGEX is an input image {PNG, PNM, JPEG}\n";
  usage += "\t - MOSAIC_IMAGE is the output image {PNG, PNM, JPEG}\n";
  google::SetUsageMessage(usage);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // This is not the place for this. I am experimenting with what sort of API
  // will be convenient for the tracking base classes.
  std::vector<std::string> files;
  for (int i = 1; i < argc; ++i) {
    files.push_back(argv[i]);
  }
  std::sort(files.begin(), files.end());
  if (files.size() < 2) {
    VLOG(0) << "Not enough files." << std::endl;
    return 1;
  }
  // Imports matches
  tracker::FeaturesGraph fg;
  FeatureSet *fs = fg.CreateNewFeatureSet();
  VLOG(0) << "Loading Matches file..." << std::endl;
  ImportMatchesFromTxt(FLAGS_m, &fg.matches_, fs);
  VLOG(0) << "Loading Matches file...[DONE]." << std::endl;
    
  vector<cv::Matx33d> Hs;
  double outliers_prob = 1e-2;
  VLOG(0) << "Estimating relative matrices..." << std::endl;
  switch (FLAGS_transformation) {
    // TODO(julien) add custom degree of freedom selection (e.g. x, y, x & y, ...)
    case EUCLIDEAN:
      ComputeRelativeEuclideanMatrices(fg.matches_, &Hs, outliers_prob);
    break;
    case SIMILARITY:
      ComputeRelativeSimilarityMatrices(fg.matches_, &Hs, outliers_prob);
    break;
    case AFFINE:
      ComputeRelativeAffineMatrices(fg.matches_, &Hs, outliers_prob);
    break;
    case HOMOGRAPHY:
      ComputeRelativeHomographyMatrices(fg.matches_, &Hs, outliers_prob);
    break;
  }
  VLOG(0) << "Estimating relative matrices...[DONE]." << std::endl;

  cv::Mat mosaic;
  VLOG(0) << "Building mosaic..." << std::endl;
  BuildMosaic(files, Hs, 
              FLAGS_blending_ratio, 
              FLAGS_draw_lines,
              mosaic);
  VLOG(0) << "Building mosaic...[DONE]." << std::endl;
  
  // Write the mosaic
  VLOG(0) << "Saving mosaic image." << std::endl;
  cv::imwrite(FLAGS_o, mosaic);
  // Delete the features graph
  fg.DeleteAndClear();
  return 0;
}

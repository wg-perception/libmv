
#include "libmv/reconstruction/motion.h"
#include "libmv/correspondence/matches.h"
#include "libmv/multiview/robust_affine.h"
#include "libmv/multiview/robust_euclidean.h"
#include "libmv/multiview/robust_homography.h"
#include "libmv/multiview/robust_similarity.h"
#include "libmv/logging/logging.h"
#include "libmv/multiview/fundamental.h"
#include "libmv/multiview/robust_fundamental.h"

#include <algorithm>
#include <string>
#include <stdexcept>

#include <opencv2/core/eigen.hpp>

using namespace libmv;

//TODO do a propper integration
cv::Matx33d
MatToMatx(const Mat3 & mat3)
{
    cv::Matx33d mat;
    eigen2cv(mat3,mat);
    return mat;
}

void libmv::EuclideanMatricesFromVideo(const Matches &matches,
        vector<cv::Matx33d> *Es,
        double outliers_prob,
        double max_error_2d)
{
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
            VLOG(2) << "E = " << std::endl << E << std::endl;
        } // TODO(julien) what to do when no enough points?
        ++prev_image_iter;
    }
}

void libmv::SimilarityMatricesFromVideo(const Matches &matches,
        vector<cv::Matx33d> *Ss,
        double outliers_prob,
        double max_error_2d) 
{
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
            VLOG(2) << "S = " << std::endl << S << std::endl;
        } // TODO(julien) what to do when no enough points?
        ++prev_image_iter;
    }
}

void libmv::AffineMatricesFromVideo(const Matches &matches,
        vector<cv::Matx33d> *As,
        double outliers_prob,
        double max_error_2d) 
{
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
            VLOG(2) << "A = " << std::endl << A << std::endl;
        } // TODO(julien) what to do when no enough points?
        ++prev_image_iter;
    }
}

void libmv::HomographyMatricesFromVideo(const Matches &matches,
        vector<cv::Matx33d> *Hs,
        double outliers_prob,
        double max_error_2d) 
{
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
            VLOG(2) << "H = " << std::endl << H << std::endl;
        } // TODO(julien) what to do when no enough points?
        ++prev_image_iter;
    }
}

bool libmv::CameraMotionTwoViews(const libmv::Matches &matches,
        Matches::ImageID image1,
        Matches::ImageID image2,
        const libmv::Mat3 &K1,
        const libmv::Mat3 &K2,
        double epipolar_threshold,
        double outliers_probability,
        libmv::Mat3 *dR,
        libmv::Vec3 *dt,
        vector<Matches::TrackID> *feature_inliers)
{
    assert(dR);
    assert(dt);
    if (image1 == image2)
        throw std::runtime_error("CameraPositionTwoVies: image1 and image2 must not be equal");

    vector<Mat> xs(2);
    vector<Matches::TrackID> tracks;
    vector<Matches::ImageID> images;
    images.push_back(image1);
    images.push_back(image2);

    PointMatchMatrices(matches, images, &tracks, &xs);
    // TODO(julien) Also remove structures that are on the same location
    if (xs[0].cols() < 7) {
        LOG(ERROR) << "Error: there are not enough common matches ("
            << xs[0].cols()<< "<7).";
        return false;
    }

    Mat &x0 = xs[0];
    Mat &x1 = xs[1];
    vector<int> feature_inliers_id;
    Mat3 F;
    // Computes fundamental matrix
    // TODO(julien) For the calibrated case, we can squeeze the fundamental using
    // directly the 5 points algorithm
    FundamentalFromCorrespondences7PointRobust(x0,x1,
            epipolar_threshold,
            &F, &feature_inliers_id,
            outliers_probability);

    // Only inliers are selected in order to estimation the relative motion
    Mat2X v0(2, feature_inliers_id.size());
    Mat2X v1(2, feature_inliers_id.size());
    size_t index_inlier = 0;
    for (size_t c = 0; c < feature_inliers_id.size(); ++c) {
        index_inlier = feature_inliers_id[c];
        v0.col(c) = x0.col(index_inlier);
        v1.col(c) = x1.col(index_inlier);
    }
    //convert feature id to track id
    if(feature_inliers)
    {
        feature_inliers->reserve(feature_inliers_id.size());
        for (size_t c = 0; c < feature_inliers_id.size(); ++c)
            feature_inliers->push_back(tracks[feature_inliers_id[c]]);
    }
    Mat3 E;
    // Computes essential matrix
    EssentialFromFundamental(F, K1, K2, &E);

    // Recover motion between the two images
    return MotionFromEssentialAndCorrespondence(E, K1, v0.col(0),
            K2, v1.col(0),
            dR, dt);
}


#ifndef LIBMV_RECONSTRUCTION_MOTION_H_
#define LIBMV_RECONSTRUCTION_MOTION_H_

#include "libmv/correspondence/matches.h"
#include "libmv/numeric/numeric.h"

namespace libmv {
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
     */
    void EuclideanMatricesFromVideo(const Matches &matches,
            vector<cv::Matx33d> *Es,
            double outliers_prob = 1e-2,
            double max_error_2d = 1);

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
     */
    void SimilarityMatricesFromVideo(const Matches &matches,
            vector<cv::Matx33d> *Ss,
            double outliers_prob = 1e-2,
            double max_error_2d = 1);

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
     */
    void AffineMatricesFromVideo(const Matches &matches,
            vector<cv::Matx33d> *As,
            double outliers_prob = 1e-2,
            double max_error_2d = 1);

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
     */
    void HomographyMatricesFromVideo(const Matches &matches,
            vector<cv::Matx33d> *Hs,
            double outliers_prob = 1e-2,
            double max_error_2d = 1);

    /**
     * Computes the camera motion relative to the first image
     *
     * \param matches The 2D features matches
     */
    bool CameraMotionTwoViews(const libmv::Matches &matches,
            Matches::ImageID image1,
            Matches::ImageID image2,
            const libmv::Mat3 &K1,
            const libmv::Mat3 &K2,
            double epipolar_threshold,
            double outliers_probability,
            libmv::Mat3 *dR,
            libmv::Vec3 *dt,
            libmv::vector<libmv::Matches::TrackID> *feature_inliers = NULL);
}

#endif


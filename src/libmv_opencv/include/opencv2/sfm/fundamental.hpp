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

#ifndef __OPENCV_SFM_FUNDAMENTAL_HPP__
#define __OPENCV_SFM_FUNDAMENTAL_HPP__

#ifdef __cplusplus

#include <opencv2/core/core.hpp>

namespace cv
{

CV_EXPORTS
void
projectionsFromFundamental( const Mat &F,
                            Mat &P1,
                            Mat &P2 );

CV_EXPORTS
void
fundamentalFromProjections( const Mat &P1,
                            const Mat &P2,
                            Mat &F );

/**
 * The normalized 8-point fundamental matrix solver.
 * Reference: HZ2 11.2 pag.281 (x1 = x, x2 = x')
 */
CV_EXPORTS
void
normalizedEightPointSolver( const cv::Mat &x1,
                            const cv::Mat &x2,
                            cv::Mat &F );

/**
 * Compute the relative camera motion between two cameras.
 *
 * Given the motion parameters of two cameras, computes the motion parameters
 * of the second one assuming the first one to be at the origin.
 * If T1 and T2 are the camera motions, the computed relative motion is
 *    T = T2 T1^{-1}
 */
CV_EXPORTS
void
relativeCameraMotion( const Mat &R1,
                      const Mat &t1,
                      const Mat &R2,
                      const Mat &t2,
                      Mat &R,
                      Mat &t );

/** Get Motion (R's and t's ) from Essential matrix.
 *  HZ 9.6 pag 259 (Result 9.19)
 */
CV_EXPORTS
void
motionFromEssential(const Mat &E, std::vector<Mat> &Rs, std::vector<Mat> &ts);

/** Get Essential matrix from Fundamental and Camera matrices
 *  HZ 9.6 pag 257 (formula 9.12)
 *  Or http://ai.stanford.edu/~birch/projective/node20.html
 */
CV_EXPORTS
void
fundamentalFromEssential(const Mat &E, const Mat &K1, const Mat &K2, Mat &F);

/** Get Essential matrix from Fundamental and Camera matrices
 *  HZ 9.6 pag 257 (formula 9.12)
 */
CV_EXPORTS
void
essentialFromFundamental(const Mat &F, const Mat &K1, const Mat &K2, Mat &E);

} /* namespace cv */

} /* namespace cv */

#endif /* __cplusplus */

#endif

/* End of file. */

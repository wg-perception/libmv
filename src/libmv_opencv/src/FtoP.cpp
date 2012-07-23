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

#include <opencv2/sfm/sfm.hpp>

namespace cv
{

  void
  FtoP(InputArray F_, OutputArrayOfArrays Ps, bool is_projective)
  {
    Mat F = F_.getMat();
    Mat P = Mat::eye(3, 4, CV_64F);
    Mat Pp = Mat::zeros(3, 4, CV_64F);

    // Projective
    if (is_projective)
    {
      cv::Mat Ep;

      // Todo: Double check this part from matlab code - isempty(F)???
      if (F.empty())
      {
        // Reference: HZ2, p246, Table 9.1
        cv::Mat ep = F.col(3);
        skew(ep, Ep);
        Pp = Ep * F;
      }

      // Affine
      else
      {
        // Reference: HZ2, p256, Result 9.14

        // SVD
        cv::Mat U, Vt, W;
        cv::SVD::compute(F, W, U, Vt);

        cv::Mat ep = U.col(U.cols - 1);
        skew(ep, Ep);

        Pp.colRange(0, 2) = Ep * F; // 3x3
        Pp.col(3) = ep;

      }

    }
    else
    {
      //Todo:
    }

    // Pack output
    Ps.create(2, 1, CV_64F);
    P.copyTo(Ps.getMatRef(0));
    Pp.copyTo(Ps.getMatRef(1));

  }

}
/* namespace cv */

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
using namespace libmv;
using namespace std;


namespace cv
{

void reconstruct(const InputArrayOfArrays points2d,
		OutputArrayOfArrays projection_matrices, OutputArray points3d, bool,
		bool is_projective, bool has_outliers, bool is_sequence)
{
	std::vector<double> v;
	int n=v.size();
	int nviews=points2d.size();

	/*	Variables for libmv functions */
	Matches matches;
	Matches *matches_inliers;
	Reconstruction reconstruction;

	/*	Convert to libmv compatible data types */
//	for (int v = 0; v < nviews; ++v)
//	{
//		for (int p = 0; p < npts; ++p) {
//			PointFeature * feature = new PointFeature(W[v].at<double>(0, p),
//					W[v].at<double>(1, p));
//			matches.Insert(v, p, feature);
//		}
//	}

	if (is_projective)
	{
//ReconstructFromTwoUncalibratedViews	(const Matches &matches,
//			CameraID image_id1,
//			CameraID image_id2,
//			Matches *matches_inliers,
//			Reconstruction *reconstruction);

	}
	else // Eucledian?
	{

	}
}
}

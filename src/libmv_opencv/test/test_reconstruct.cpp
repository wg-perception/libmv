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

#include "test_precomp.hpp"

using namespace cv;
using namespace std;

TEST(Sfm_reconstruct, twoViewProjective)
{
    vector<vector<Point2d> > points2d;
    vector<Point3d> points3d;
    vector<Mat> projection_matrices;
    vector<Mat> estimated_projection_matrices;

    // ToDo (pablo): fix it (THIS_SOURCE_DIR definition is temporal solution, see CMakeLists.txt file)
    // string filename(cvtest::TS::ptr()->get_data_path() + "sfm/rnd_N10_F3.yml");
    string filename(string(THIS_SOURCE_DIR) + "/testdata/cv/sfm/rnd_N10_F3.yml");

    cvtest::readtestdata(filename, 2, 10, points2d);
    cvtest::readtestdata(filename, 2, projection_matrices);

    reconstruct(points2d, estimated_projection_matrices, points3d, true);

    cout << "Groundtruth:" << endl;
    cout << projection_matrices[0] << endl;
    cout << projection_matrices[1] << endl;
    cout << "Estimate:" << endl;
    cout << estimated_projection_matrices[0] << endl;
    cout << estimated_projection_matrices[1] << endl;
    cout << "Not necessarily equal, the first one should be identity though. Better to check diff in PX" << endl;
}

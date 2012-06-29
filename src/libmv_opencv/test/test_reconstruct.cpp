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

  string filename(cvtest::TS::ptr()->get_data_path() + "sfm/rnd_N10_F3.yml");

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
}

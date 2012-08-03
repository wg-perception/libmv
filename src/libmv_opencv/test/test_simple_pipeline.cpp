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

#include <fstream>
#include <cstdlib>

using namespace cv;
using namespace std;


/**
 * 2D tracked points (dinosaur dataset)
 * ------------------------------------
 *
 * The format is:
 *
 * row1 : x1 y1 x2 y2 ... x36 y36 for track 1
 * row2 : x1 y1 x2 y2 ... x36 y36 for track 2
 * etc
 *
 * i.e. a row gives the 2D measured position of a point as it is tracked
 * through frames 1 to 36.  If there is no match found in a view then x
 * and y are -1.
 *
 * Each row corresponds to a different point.  There are 4983 values.
 *
 */
void vgg_2D_tracked_points_parser( Mat &_values )
{
    // Copy 'viff.xy.txt' file (2.4 MB) into "testdata/cv/sfm/" folder.
    // Dinosaur dataset: http://www.robots.ox.ac.uk/~vgg/data/data-mview.html
    string filename = string(TEST_DATA_DIR) + "viff.xy.txt"; 
    ifstream file( filename.c_str() );

    Mat_<double> values(4983, 36*2);
    string token;
    for (int row = 0; getline(file, token); ++row)
    {
        istringstream line(token);

        int col = 0;
        while (line >> token)
        {
            // cout << "row: " << row << " col: " << col << " Token :" << token << endl;
            values(row, col) = atof(token.c_str());
            col++;
        }
//         if (file.unget().get() == '\n')
//         {
//             // cout << "newline found" << endl;
//         }
    }

    values.copyTo( _values );
}


TEST(Sfm_simple_pipeline, dinosaur)
{
    Mat_<double> values;
    vgg_2D_tracked_points_parser( values );

    EXPECT_DOUBLE_EQ( 197.44, values(4979,70));
    EXPECT_DOUBLE_EQ( 260.19, values(4977,66));

    // ToDo: complete the test
//     FAIL();
}

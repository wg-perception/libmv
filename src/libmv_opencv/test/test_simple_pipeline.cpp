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

#include "libmv/simple_pipeline/bundle.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"
#include "libmv/simple_pipeline/initialize_reconstruction.h"
#include "libmv/simple_pipeline/pipeline.h"
#include "libmv/simple_pipeline/tracks.h"
#include "third_party/ssba/Math/v3d_optimization.h"

#include <fstream>
#include <cstdlib>

using namespace cv;
using namespace std;

/**
 * 2D tracked points
 * -----------------
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
 * Each row corresponds to a different point.
 *
 */
void parser_2D_tracks( libmv::Tracks &libmv_tracks, string _filename )
{
    string filename = string(TEST_DATA_DIR) + _filename;
    ifstream file( filename.c_str() );

    double x, y;
    string str;

    for (int track = 0; getline(file, str); ++track)
    {
        istringstream line(str);
        bool is_first_time = true;

        for (int frame = 0; line >> x >> y; ++frame)
        {
            // valid marker
            if ( x > 0 && y > 0 )
            {
                libmv_tracks.Insert( frame, track, x, y );

                if ( is_first_time )
                    is_first_time = false;
            }

            // lost track
            else if ( x < 0 && y < 0 )
            {
                is_first_time = true;
            }

            // some error
            else
            {
                exit(1);
            }
        }
    }
}



typedef struct libmv_Reconstruction
{
    libmv::EuclideanReconstruction reconstruction;

    /* used for per-track average error calculation after reconstruction */
    libmv::Tracks tracks;
    libmv::CameraIntrinsics intrinsics;

    double error;
} libmv_Reconstruction;


// ToDo (pablo): rewrite this, and move to "src/" folder
// Based on the 'libmv_capi' function (blender API)
void libmv_solveReconstruction(const libmv::Tracks &tracks, int keyframe1, int keyframe2,
                               double focal_length, double principal_x, double principal_y, double k1, double k2, double k3,
                               libmv_Reconstruction &libmv_reconstruction, bool refine_intrinsics = false)
{
    /* Invert the camera intrinsics. */
    libmv::vector<libmv::Marker> markers = tracks.AllMarkers();
    libmv::EuclideanReconstruction *reconstruction = &libmv_reconstruction.reconstruction;
    libmv::CameraIntrinsics *intrinsics = &libmv_reconstruction.intrinsics;

    intrinsics->SetFocalLength(focal_length, focal_length);
    intrinsics->SetPrincipalPoint(principal_x, principal_y);
    intrinsics->SetRadialDistortion(k1, k2, k3);

    cout << "\tNumber of markers: " << markers.size() << endl;
    for (int i = 0; i < markers.size(); ++i)
    {
        intrinsics->InvertIntrinsics(markers[i].x,
                                     markers[i].y,
                                     &(markers[i].x),
                                     &(markers[i].y));
    }

    libmv::Tracks normalized_tracks(markers);

    cout << "\tframes to init from: " << keyframe1 << " " << keyframe2 << endl;
    libmv::vector<libmv::Marker> keyframe_markers =
        normalized_tracks.MarkersForTracksInBothImages(keyframe1, keyframe2);
    cout << "\tNumber of markers for init: " << keyframe_markers.size() << endl;

    libmv::EuclideanReconstructTwoFrames(keyframe_markers, reconstruction);
    libmv::EuclideanBundle(normalized_tracks, reconstruction);
    libmv::EuclideanCompleteReconstruction(normalized_tracks, reconstruction);

    // ToDo (pablo): autocalibration?
//     if (refine_intrinsics) {
//         libmv_solveRefineIntrinsics(tracks, intrinsics, reconstruction,
//             refine_intrinsics, progress_update_callback, callback_customdata);
//     }

    libmv_reconstruction.tracks = tracks;
    libmv_reconstruction.error = libmv::EuclideanReprojectionError(tracks, *reconstruction, *intrinsics);
}



TEST(Sfm_simple_pipeline, backyard)
{
    V3D::optimizerVerbosenessLevel = 0; // less logging messages

    // Get tracks from file: check backyard.blend file
    libmv::Tracks tracks;
    parser_2D_tracks( tracks, "backyard_tracks.txt" );

    // Initial reconstruction
    int keyframe1 = 1, keyframe2 = 30;

    // Camera data
    double focal_length = 860.986572265625;  // f = 24mm (checked debugging blender)
    double principal_x = 400, principal_y = 225, k1 = -0.158, k2 = 0.131, k3 = 0;


    libmv_Reconstruction libmv_reconstruction;
    libmv_solveReconstruction( tracks, keyframe1, keyframe2,
                               focal_length, principal_x, principal_y, k1, k2, k3,
                               libmv_reconstruction );

    cout << "libmv_reconstruction.error = " << libmv_reconstruction.error << endl;

    EXPECT_LE( libmv_reconstruction.error, 1.6 );  // actually 1.50247
}





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

#include <iostream>

#include "testing/testing.h"
#include "libmv/simple_pipeline/keyframe_selection.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"
#include "libmv/logging/logging.h"

namespace libmv {

TEST(KeyframeSelection, SyntheticNeighborFrame) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(900.0,900.0);
  intrinsics.SetPrincipalPoint(640.0, 540.0);
  intrinsics.SetRadialDistortion(0.0, 0.0, 0.0);

  Tracks tracks;
  const int markers_per_size = 15;

  // Fill in tracks for homography estimation
  for (int x = 0; x < markers_per_size; x++) {
    for (int y = 0; y < markers_per_size; y++) {
      double current_x = 10 + x * 40, current_y = 10 + y * 40;
      double next_x = current_x + 10, next_y = current_y + 10;

       // TODO(sergey): somehow test only works nice now if tracks are
       // in pixel space, however this need to be investigated because
       // on real data algebraic estimation could give degenerated matrices
      //intrinsics.InvertIntrinsics(current_x, current_y, &current_x, &current_y);
      //intrinsics.InvertIntrinsics(next_x, next_y, &next_x, &next_y);

      tracks.Insert(1, y * markers_per_size + x, current_x, current_y);
      tracks.Insert(2, y * markers_per_size + x, next_x, next_y);
    }
  }

  vector<int> keyframes;
  SelectkeyframesBasedOnGRIC(tracks, keyframes);

  // Synthetic second frame shouldn't be considered a keyframe
  EXPECT_TRUE(keyframes.size() == 1);
}

} // namespace libmv

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

// synthetic test, N markers with the same translation
//. should not be keyframe
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

      intrinsics.InvertIntrinsics(current_x, current_y, &current_x, &current_y);
      intrinsics.InvertIntrinsics(next_x, next_y, &next_x, &next_y);

      tracks.Insert(1, y * markers_per_size + x, current_x, current_y);
      tracks.Insert(2, y * markers_per_size + x, next_x, next_y);
    }
  }

  vector<int> keyframes;
  SelectkeyframesBasedOnGRIC(tracks, keyframes);

  // Synthetic second frame shouldn't be considered a keyframe
  EXPECT_EQ(keyframes.size(), 0);
}

// frames 1 and 2 of FabrikEingang footage
// only one wall is tracked, should not be keyframes
TEST(KeyframeSelection, FabrikEingangNeighborFrames) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(1605.797, 1605.797);
  intrinsics.SetPrincipalPoint(960.000, 544.000);
  intrinsics.SetRadialDistortion(0.0, 0.0, 0.0);

  Marker markers[] = {
      {1, 0, 737.599983, 646.397594},
      {2, 0, 737.906628, 648.113327},
      {1, 1, 863.045425, 646.081905},
      {2, 1, 863.339767, 647.650040},
      {1, 2, 736.959972, 574.080151},
      {2, 2, 737.217350, 575.604900},
      {1, 3, 864.097424, 573.374908},
      {2, 3, 864.383469, 574.900307},
      {1, 4, 789.429073, 631.677521},
      {2, 4, 789.893131, 633.124451},
      {1, 5, 791.051960, 573.442028},
      {2, 5, 791.336575, 575.088890},
      {1, 6, 738.973961, 485.130308},
      {2, 6, 739.435501, 486.734207},
      {1, 7, 862.403240, 514.866074},
      {2, 7, 862.660618, 516.413261},
      {1, 8, 802.240162, 485.759838},
      {2, 8, 802.602253, 487.432899},
      {1, 9, 754.340630, 500.624559},
      {2, 9, 754.559956, 502.079920},
      {1, 10, 849.398689, 484.480545},
      {2, 10, 849.599934, 486.079937},
      {1, 11, 788.803768, 515.924391},
      {2, 11, 789.119911, 517.439932},
      {1, 12, 838.733940, 558.212688},
      {2, 12, 839.039898, 559.679916},
      {1, 13, 760.014782, 575.194466},
      {2, 13, 760.319881, 576.639904},
      {1, 14, 765.321636, 616.015957},
      {2, 14, 765.759945, 617.599915},
      {1, 15, 800.963230, 660.032082},
      {2, 15, 801.279945, 661.759876},
      {1, 16, 846.321087, 602.313053},
      {2, 16, 846.719913, 603.839878},
      {1, 17, 864.288311, 616.790524},
      {2, 17, 864.639931, 618.239918},
      {1, 18, 800.006790, 602.573425},
      {2, 18, 800.319958, 604.159912},
      {1, 19, 739.026890, 617.944138},
      {2, 19, 739.199924, 619.519924},
      {1, 20, 801.987419, 544.134888},
      {2, 20, 802.239933, 545.599911},
      {1, 21, 753.619823, 542.961300},
      {2, 21, 753.919945, 544.639874},
      {1, 22, 787.921257, 499.910206},
      {2, 22, 788.159924, 501.439917},
      {1, 23, 839.095459, 529.287903},
      {2, 23, 839.359932, 530.879934},
      {1, 24, 811.760330, 630.732269},
      {2, 24, 812.159901, 632.319859}
    };
  int num_markers = sizeof(markers) / sizeof(Marker);

  Tracks tracks;
  for (int i = 0; i < num_markers; i++) {
    double x = markers[i].x, y = markers[i].y;
    intrinsics.InvertIntrinsics(x, y, &x, &y);
    tracks.Insert(markers[i].image, markers[i].track, x, y);
  }

  vector<int> keyframes;
  SelectkeyframesBasedOnGRIC(tracks, keyframes);

  EXPECT_EQ(keyframes.size(), 0);
}

// Frames 120 and 200 from FabrikEingang footage
// should be enough of parallax for keyframing
TEST(KeyframeSelection, FabrikEingangFarFrames) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(1605.797, 1605.797);
  intrinsics.SetPrincipalPoint(960.000, 544.000);
  intrinsics.SetRadialDistortion(0.0, 0.0, 0.0);

  Marker markers[] = {
      {1, 0, 369.459200, 619.315258},
      {2, 0, 279.677496, 722.086842},
      {1, 1, 376.831970, 370.278397},
      {2, 1, 221.695247, 460.065418},
      {1, 2, 1209.139023, 567.705605},
      {2, 2, 1080.760117, 659.230083},
      {1, 3, 1643.495750, 903.620453},
      {2, 3, 1618.405037, 1015.374908},
      {1, 4, 1494.849815, 425.302460},
      {2, 4, 1457.467575, 514.727587},
      {1, 5, 1794.637299, 328.728609},
      {2, 5, 1742.161446, 408.988636},
      {1, 6, 1672.822723, 102.240358},
      {2, 6, 1539.287224, 153.536892},
      {1, 7, 1550.843925, 53.424943},
      {2, 7, 1385.579109, 96.450085},
      {1, 8, 852.953281, 465.399578},
      {2, 8, 779.404564, 560.091843},
      {1, 9, 906.853752, 299.827040},
      {2, 9, 786.923218, 385.570770},
      {1, 10, 406.322966, 87.556041},
      {2, 10, 140.339413, 150.877481},
      {1, 11, 254.811573, 851.296478},
      {2, 11, 94.478302, 969.350189},
      {1, 12, 729.087868, 806.092758},
      {2, 12, 606.212139, 919.876560},
      {1, 13, 1525.719452, 920.398083},
      {2, 13, 1495.579720, 1031.971218}
    };
  int num_markers = sizeof(markers) / sizeof(Marker);

  Tracks tracks;
  for (int i = 0; i < num_markers; i++) {
    double x = markers[i].x, y = markers[i].y;
    intrinsics.InvertIntrinsics(x, y, &x, &y);
    tracks.Insert(markers[i].image, markers[i].track, x, y);
  }

  vector<int> keyframes;
  SelectkeyframesBasedOnGRIC(tracks, keyframes);

  EXPECT_EQ(keyframes.size(), 2);
}

TEST(KeyframeSelection, CopterManualFrames) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(1155.043, 1155.043);
  intrinsics.SetPrincipalPoint(640.000, 360.000);
  intrinsics.SetRadialDistortion(-0.08590, 0.0, 0.0);

  Marker markers[] = {
      {1, 0, 645.792694, 403.115931},
      {2, 0, 630.641174, 307.996409},
      {1, 1, 783.469086, 403.904328},
      {2, 1, 766.001129, 308.998225},
      {1, 2, 650.000000, 160.000001},
      {1, 3, 785.225906, 158.619039},
      {2, 3, 767.526474, 70.449695},
      {1, 4, 290.640526, 382.335634},
      {2, 4, 273.001728, 86.993319},
      {1, 5, 291.162739, 410.602684},
      {2, 5, 273.287849, 111.937487},
      {1, 6, 136.919317, 349.895797},
      {1, 7, 490.844345, 47.572222},
      {1, 8, 454.406433, 488.935761},
      {1, 9, 378.655815, 618.522248},
      {2, 9, 357.061806, 372.265077},
      {1, 10, 496.011391, 372.668824},
      {2, 10, 477.979164, 222.986112},
      {1, 11, 680.060272, 256.103625},
      {2, 11, 670.587540, 204.830453},
      {1, 12, 1070.817108, 218.775322},
      {2, 12, 1046.129913, 128.969783},
      {1, 14, 242.516403, 596.048512},
      {2, 14, 224.182606, 248.272154},
      {1, 15, 613.936272, 287.519073},
      {2, 15, 600.467644, 196.085722},
      {1, 31, 844.637451, 256.354315},
      {2, 31, 823.200150, 165.714952},
    };
  int num_markers = sizeof(markers) / sizeof(Marker);

  Tracks tracks;
  for (int i = 0; i < num_markers; i++) {
    double x = markers[i].x, y = markers[i].y;
    intrinsics.InvertIntrinsics(x, y, &x, &y);
    tracks.Insert(markers[i].image, markers[i].track, x, y);
  }

  vector<int> keyframes;
  SelectkeyframesBasedOnGRIC(tracks, keyframes);

  EXPECT_EQ(keyframes.size(), 2);
}

} // namespace libmv

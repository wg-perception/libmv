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

// Synthetic test, N markers with the same translation
// Should not be keyframe
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
  SelectkeyframesBasedOnGRIC(tracks, intrinsics, keyframes);

  // Synthetic second frame shouldn't be considered a keyframe
  EXPECT_EQ(keyframes.size(), 0);
}

// Frames 1 and 2 of FabrikEingang footage
// Only one wall is tracked, should not be keyframes
TEST(KeyframeSelection, FabrikEingangNeighborFrames) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(1605.797, 1605.797);
  intrinsics.SetPrincipalPoint(960.000, 544.000);
  intrinsics.SetRadialDistortion(0.0, 0.0, 0.0);

  Marker markers[] = {
      {1, 0, 737.599983, 646.397594},  {2, 0, 737.906628, 648.113327},  {1, 1, 863.045425, 646.081905},
      {2, 1, 863.339767, 647.650040},  {1, 2, 736.959972, 574.080151},  {2, 2, 737.217350, 575.604900},
      {1, 3, 864.097424, 573.374908},  {2, 3, 864.383469, 574.900307},  {1, 4, 789.429073, 631.677521},
      {2, 4, 789.893131, 633.124451},  {1, 5, 791.051960, 573.442028},  {2, 5, 791.336575, 575.088890},
      {1, 6, 738.973961, 485.130308},  {2, 6, 739.435501, 486.734207},  {1, 7, 862.403240, 514.866074},
      {2, 7, 862.660618, 516.413261},  {1, 8, 802.240162, 485.759838},  {2, 8, 802.602253, 487.432899},
      {1, 9, 754.340630, 500.624559},  {2, 9, 754.559956, 502.079920},  {1, 10, 849.398689, 484.480545},
      {2, 10, 849.599934, 486.079937}, {1, 11, 788.803768, 515.924391}, {2, 11, 789.119911, 517.439932},
      {1, 12, 838.733940, 558.212688}, {2, 12, 839.039898, 559.679916}, {1, 13, 760.014782, 575.194466},
      {2, 13, 760.319881, 576.639904}, {1, 14, 765.321636, 616.015957}, {2, 14, 765.759945, 617.599915},
      {1, 15, 800.963230, 660.032082}, {2, 15, 801.279945, 661.759876}, {1, 16, 846.321087, 602.313053},
      {2, 16, 846.719913, 603.839878}, {1, 17, 864.288311, 616.790524}, {2, 17, 864.639931, 618.239918},
      {1, 18, 800.006790, 602.573425}, {2, 18, 800.319958, 604.159912}, {1, 19, 739.026890, 617.944138},
      {2, 19, 739.199924, 619.519924}, {1, 20, 801.987419, 544.134888}, {2, 20, 802.239933, 545.599911},
      {1, 21, 753.619823, 542.961300}, {2, 21, 753.919945, 544.639874}, {1, 22, 787.921257, 499.910206},
      {2, 22, 788.159924, 501.439917}, {1, 23, 839.095459, 529.287903}, {2, 23, 839.359932, 530.879934},
      {1, 24, 811.760330, 630.732269}, {2, 24, 812.159901, 632.319859}
    };
  int num_markers = sizeof(markers) / sizeof(Marker);

  Tracks tracks;
  for (int i = 0; i < num_markers; i++) {
    double x = markers[i].x, y = markers[i].y;
    intrinsics.InvertIntrinsics(x, y, &x, &y);
    tracks.Insert(markers[i].image, markers[i].track, x, y);
  }

  vector<int> keyframes;
  SelectkeyframesBasedOnGRIC(tracks, intrinsics, keyframes);

  EXPECT_EQ(keyframes.size(), 0);
}

// Frames 120 and 200 from FabrikEingang footage
// Should be enough of parallax for keyframing
TEST(KeyframeSelection, FabrikEingangFarFrames) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(1605.797, 1605.797);
  intrinsics.SetPrincipalPoint(960.000, 544.000);
  intrinsics.SetRadialDistortion(0.0, 0.0, 0.0);

  Marker markers[] = {
      {1, 0, 369.459200, 619.315258},  {2, 0, 279.677496, 722.086842},   {1, 1, 376.831970, 370.278397},
      {2, 1, 221.695247, 460.065418},  {1, 2, 1209.139023, 567.705605},  {2, 2, 1080.760117, 659.230083},
      {1, 3, 1643.495750, 903.620453}, {2, 3, 1618.405037, 1015.374908}, {1, 4, 1494.849815, 425.302460},
      {2, 4, 1457.467575, 514.727587}, {1, 5, 1794.637299, 328.728609},  {2, 5, 1742.161446, 408.988636},
      {1, 6, 1672.822723, 102.240358}, {2, 6, 1539.287224, 153.536892},  {1, 7, 1550.843925, 53.424943},
      {2, 7, 1385.579109, 96.450085},  {1, 8, 852.953281, 465.399578},   {2, 8, 779.404564, 560.091843},
      {1, 9, 906.853752, 299.827040},  {2, 9, 786.923218, 385.570770},   {1, 10, 406.322966, 87.556041},
      {2, 10, 140.339413, 150.877481}, {1, 11, 254.811573, 851.296478},  {2, 11, 94.478302, 969.350189},
      {1, 12, 729.087868, 806.092758}, {2, 12, 606.212139, 919.876560},  {1, 13, 1525.719452, 920.398083},
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
  SelectkeyframesBasedOnGRIC(tracks, intrinsics, keyframes);

  EXPECT_EQ(keyframes.size(), 2);
}

// Manually selected keyframes from copter footage from Sebastian
// Keyframes were 167 and 237
TEST(KeyframeSelection, CopterManualKeyFrames) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(1155.043, 1155.043);
  intrinsics.SetPrincipalPoint(640.000, 360.000);
  intrinsics.SetRadialDistortion(-0.08590, 0.0, 0.0);

  Marker markers[] = {
      {1, 0, 645.792694, 403.115931},   {2, 0, 630.641174, 307.996409},  {1, 1, 783.469086, 403.904328},
      {2, 1, 766.001129, 308.998225},   {1, 2, 650.000000, 160.000001},  {1, 3, 785.225906, 158.619039},
      {2, 3, 767.526474, 70.449695},    {1, 4, 290.640526, 382.335634},  {2, 4, 273.001728, 86.993319},
      {1, 5, 291.162739, 410.602684},   {2, 5, 273.287849, 111.937487},  {1, 6, 136.919317, 349.895797},
      {1, 7, 490.844345, 47.572222},    {1, 8, 454.406433, 488.935761},  {1, 9, 378.655815, 618.522248},
      {2, 9, 357.061806, 372.265077},   {1, 10, 496.011391, 372.668824}, {2, 10, 477.979164, 222.986112},
      {1, 11, 680.060272, 256.103625},  {2, 11, 670.587540, 204.830453}, {1, 12, 1070.817108, 218.775322},
      {2, 12, 1046.129913, 128.969783}, {1, 14, 242.516403, 596.048512}, {2, 14, 224.182606, 248.272154},
      {1, 15, 613.936272, 287.519073},  {2, 15, 600.467644, 196.085722}, {1, 31, 844.637451, 256.354315},
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
  SelectkeyframesBasedOnGRIC(tracks, intrinsics, keyframes);

  EXPECT_EQ(keyframes.size(), 2);
}

// Used old friend elevator scene MMI_2366 with automatic feature selection
// and manual outlier elimination and manual keyframe selection
// Selected keyframes were 29 and 41
TEST(KeyframeSelection, ElevatorManualKeyframesFrames) {
  CameraIntrinsics intrinsics;
  intrinsics.SetFocalLength(1380.000, 1380.000);
  intrinsics.SetPrincipalPoint(960.000, 540.000);
  intrinsics.SetRadialDistortion(-0.034, 0.0, 0.0);

  Marker markers[] = {
      {1, 2, 1139.861412, 1034.634984},  {2, 2, 1143.512192, 1065.355718},  {1, 3, 1760.821953, 644.658036},
      {2, 3, 1770.901108, 697.899928},   {1, 4, 858.071823, 1068.520746},   {1, 6, 1633.952408, 797.050145},
      {2, 6, 1642.508469, 849.157140},   {1, 8, 1716.695824, 451.805491},   {2, 8, 1726.513939, 502.095687},
      {1, 9, 269.577627, 724.986935},    {2, 9, 269.424820, 764.154246},    {1, 10, 1891.321907, 706.948843},
      {2, 10, 1903.338547, 766.068377},  {1, 12, 1806.227074, 956.089604},  {2, 12, 1816.619568, 1013.767376},
      {1, 14, 269.544153, 1002.333570},  {2, 14, 269.367542, 1043.509254},  {1, 15, 1402.772141, 281.392962},
      {2, 15, 1409.089165, 318.731629},  {1, 16, 195.877233, 919.454341},   {2, 16, 192.531109, 997.367899},
      {1, 17, 1789.584045, 120.036661},  {2, 17, 1800.391846, 167.822964},  {1, 18, 999.363213, 765.004807},
      {2, 18, 1002.345772, 790.560122},  {1, 19, 647.342491, 1044.805727},  {2, 19, 649.328041, 1058.682940},
      {1, 20, 1365.486832, 440.901829},  {2, 20, 1371.413040, 477.888730},  {1, 21, 1787.125282, 301.431606},
      {2, 21, 1798.527260, 355.224531},  {1, 22, 1257.805824, 932.797258},  {2, 22, 1263.017578, 969.376774},
      {1, 23, 961.969528, 843.148112},   {2, 23, 964.869461, 868.587620},   {1, 24, 158.076110, 1052.643592},
      {1, 25, 1072.884521, 1005.296981}, {2, 25, 1076.091156, 1032.776856}, {1, 26, 1107.656937, 526.577228},
      {2, 26, 1111.618423, 555.524454},  {1, 27, 1416.410751, 529.857581},  {2, 27, 1422.663574, 570.025957},
      {1, 28, 1498.673630, 1005.453086}, {2, 28, 1505.381813, 1051.827149}, {1, 29, 1428.647804, 652.473629},
      {2, 29, 1434.898224, 692.715390},  {1, 30, 1332.318764, 503.673599},  {2, 30, 1338.000069, 540.507967},
      {1, 32, 1358.642693, 709.837904},  {2, 32, 1364.231529, 748.678265},  {1, 33, 1850.911560, 460.475668},
      {2, 33, 1862.221413, 512.797347},  {1, 34, 1226.117821, 607.053959},  {2, 34, 1230.736084, 641.091449},
      {1, 35, 619.598236, 523.341744},   {2, 35, 621.601124, 554.453287},   {1, 36, 956.591492, 958.223183},
      {2, 36, 959.289265, 983.289263},   {1, 37, 1249.922218, 419.095856},  {2, 37, 1255.005913, 452.556177},
      {1, 39, 1300.528450, 386.251166},  {2, 39, 1305.957413, 420.185595},  {1, 40, 1128.689919, 972.558346},
      {2, 40, 1132.413712, 1003.984737}, {1, 41, 503.304749, 1053.504388},  {2, 41, 505.019703, 1069.175613},
      {1, 42, 1197.352982, 472.681564},  {2, 42, 1201.910706, 503.459880},  {1, 43, 1794.391022, 383.911400},
      {2, 43, 1805.324135, 436.116468},  {1, 44, 789.641418, 1058.045647},  {1, 45, 1376.575241, 928.714979},
      {2, 45, 1381.995850, 969.511957},  {1, 46, 1598.023567, 93.975592},   {2, 46, 1606.937141, 136.827035},
      {1, 47, 1455.550232, 762.128685},  {2, 47, 1462.014313, 805.164878},  {1, 48, 1357.123489, 354.460326},
      {2, 48, 1363.071899, 390.363121},  {1, 49, 939.792652, 781.549895},   {2, 49, 942.802620, 806.164012},
      {1, 50, 1380.251083, 805.948620},  {2, 50, 1385.637932, 845.592098},  {1, 51, 1021.769943, 1049.802361},
      {1, 52, 1065.634918, 608.099055},  {2, 52, 1069.142189, 635.361736},  {1, 53, 624.324188, 463.202863},
      {2, 53, 626.395454, 494.994088},   {1, 54, 1451.459885, 881.557624},  {2, 54, 1457.679634, 924.345531},
      {1, 55, 1201.885986, 1057.079022}, {1, 56, 581.157532, 947.661438},   {2, 56, 583.242359, 960.831449},
      {1, 58, 513.593102, 954.175858},   {2, 58, 515.470047, 971.309574},   {1, 59, 928.069038, 901.774421},
      {2, 59, 930.847950, 925.613744},   {1, 60, 1065.860023, 740.395389},  {2, 60, 1069.484253, 768.971086},
      {1, 61, 990.479393, 906.264632},   {2, 61, 993.217506, 933.088803},   {1, 62, 1776.196747, 776.278453},
      {2, 62, 1786.292496, 831.136880},  {1, 63, 834.454365, 1012.449725},  {2, 63, 836.868324, 1033.451807},
      {1, 64, 1355.190697, 869.184809},  {2, 64, 1360.736618, 909.773347},  {1, 65, 702.072487, 897.519686},
      {2, 65, 704.203377, 911.931131},   {1, 66, 1214.022903, 856.199934},  {2, 66, 1218.109016, 890.753052},
      {1, 67, 327.676048, 236.814036},   {2, 67, 328.335285, 277.251878},   {1, 68, 289.064083, 454.793912},
      {2, 68, 288.651924, 498.882444},   {1, 69, 1626.240692, 278.374350},  {2, 69, 1634.131508, 315.853672},
      {1, 70, 1245.375710, 734.862142},  {2, 70, 1250.047417, 769.670885},  {1, 71, 497.015305, 510.718904},
      {2, 71, 498.682308, 541.070201},   {1, 72, 1280.542030, 153.939185},  {2, 72, 1286.993637, 198.436196},
      {1, 73, 1534.748840, 138.601043},  {2, 73, 1542.961349, 180.170819},  {1, 74, 1477.412682, 200.608061},
      {2, 74, 1484.683914, 240.413260},  {1, 76, 450.637321, 407.279642},   {2, 76, 451.695642, 441.666291},
      {1, 78, 246.981239, 220.786298},   {2, 78, 244.524879, 290.016564},   {1, 79, 36.696489, 420.023407},
      {2, 79, 21.364746, 591.245492},
    };
  int num_markers = sizeof(markers) / sizeof(Marker);

  Tracks tracks;
  for (int i = 0; i < num_markers; i++) {
    double x = markers[i].x, y = markers[i].y;
    intrinsics.InvertIntrinsics(x, y, &x, &y);
    tracks.Insert(markers[i].image, markers[i].track, x, y);
  }

  vector<int> keyframes;
  SelectkeyframesBasedOnGRIC(tracks, intrinsics, keyframes);

  EXPECT_EQ(keyframes.size(), 2);
}

} // namespace libmv

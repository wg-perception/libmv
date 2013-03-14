// Copyright (c) 2010, 2011 libmv authors.
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

#ifndef LIBMV_SIMPLE_PIPELINE_KEYFRAME_SELECTION_H_
#define LIBMV_SIMPLE_PIPELINE_KEYFRAME_SELECTION_H_

#include "libmv/base/vector.h"
#include "libmv/simple_pipeline/tracks.h"
#include "libmv/simple_pipeline/camera_intrinsics.h"

namespace libmv {

// Get list of all images which are good enough to be as keyframes from
// camera reconstruction. Based on GRIC criteria and uses Pollefeys'
// approach for correspondence ratio constraint.
//
// \param tracks contains all tracked correspondences between frames
//        expected to be undistorted and normalized
// \param intrinsics is camera intrinsics
// \param keyframes will contain all images number which are considered
//        good to be used for reconstruction
void SelectkeyframesBasedOnGRIC(const Tracks &tracks,
                                CameraIntrinsics &intrinsics,
                                vector<int> &keyframes);

}  // namespace libmv

#endif  // LIBMV_SIMPLE_PIPELINE_KEYFRAME_SELECTION_H_

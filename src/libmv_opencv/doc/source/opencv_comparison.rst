OpenCV Comparison
=================

Generic problems:
 * no orthographic camera
 * no generic camera (do we want a Camera class ?)
 * for projective cameras, skew is usually assumed to be 0
 * no multiview (almost everything is 2 cameras)
 * no multiview for video sequences

In calib3d, we have:
 * calibrateCamera: generic problems
 * calibrationMatrixValues: seems good
 * composeRT: is that needed ? Should we have a Pose class ?
 * computeCorrespondEpilines: very stereo specific
 * convertPointsToHomogeneous: seems good
 * convertPointsFromHomegeneous: seems good
 * correctMatches: only two sets of points
 * decomposeProjectionMatrix: seems good
 * chessboard, dots: good
 * solvePnP: has good methods
 * solvePnPRansac: should be merged with the above as an algorithm class
 * findFundamentalMat: only does RANSAC. Practical but maybe not in certain cases where matches are known to be good
 * findHomography: seems ok (same, we could have a non-RANSCA implementation)
 * estimateAffine3d: same
 * filterSpeckle: sure
 * getOptimalNewCameraMatrix: what is that ??
 * initCameraMatrix2d: only planar points
 * matMulDeiv: should be in core ?
 * projectPoints: no ortho
 * reprojectImageTo2d: seems good
 * RQDecomp3x3: that should be core
 * Rodrigues: should be core
 * stereoCalibrate: need to look deeper
 * stereoRectify: 
 * stereoRectifyUncalibrated: 
 * triangulatePoints: need to extend to multiple views
 * please continue alphabetically

What is missing
===============

 * visible RANSAC (maybe get the ones from PCL and untemplatize them (only RANSAC and PROSAC are really useful))
 * multi-camera setup

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

#include "libmv/simple_pipeline/camera_intrinsics.h"
#include "libmv/numeric/levenberg_marquardt.h"

namespace libmv {

CameraIntrinsics::CameraIntrinsics()
    : K_(Mat3::Identity()),
      image_width_(0),
      image_height_(0),
      k1_(0),
      k2_(0),
      k3_(0),
      p1_(0),
      p2_(0),
      grid_(0) {}

void CameraIntrinsics::ApplyIntrinsics(double normalized_x,
                                       double normalized_y,
                                       double *image_x,
                                       double *image_y) const {
  double x = normalized_x;
  double y = normalized_y;

  // Apply distortion to the normalized points to get (xd, yd).
  double r2 = x*x + y*y;
  double r4 = r2 * r2;
  double r6 = r4 * r2;
  double r_coeff = (1 + k1_*r2 + k2_*r4 + k3_*r6);
  double xd = x * r_coeff + 2*p1_*x*y + p2_*(r2 + 2*x*x);
  double yd = y * r_coeff + 2*p2_*x*y + p1_*(r2 + 2*y*y);

  // Apply focal length and principal point to get the final image coordinates.
  *image_x = focal_length_x() * xd + principal_point_x();
  *image_y = focal_length_y() * yd + principal_point_y();
}

struct InvertIntrinsicsCostFunction {
 public:
  typedef Vec2 FMatrixType;
  typedef Vec2 XMatrixType;

  InvertIntrinsicsCostFunction(const CameraIntrinsics &intrinsics,
                               double image_x, double image_y)
    : intrinsics(intrinsics), x(image_x), y(image_y) {}

  Vec2 operator()(const Vec2 &u) const {
    double xx, yy;
    intrinsics.ApplyIntrinsics(u(0), u(1), &xx, &yy);
    Vec2 fx;
    fx << (xx - x), (yy - y);
    return fx;
  }
  const CameraIntrinsics &intrinsics;
  double x, y;
};

void CameraIntrinsics::InvertIntrinsics(double image_x,
                                        double image_y,
                                        double *normalized_x,
                                        double *normalized_y) const {
  // Compute the initial guess. For a camera with no distortion, this will also
  // be the final answer; the LM iteration will terminate immediately.
  Vec2 normalized;
  normalized(0) = (image_x - principal_point_x()) / focal_length_x();
  normalized(1) = (image_y - principal_point_y()) / focal_length_y();

  typedef LevenbergMarquardt<InvertIntrinsicsCostFunction> Solver;

  InvertIntrinsicsCostFunction intrinsics_cost(*this, image_x, image_y);
  Solver::SolverParameters params;
  Solver solver(intrinsics_cost);

  /*Solver::Results results =*/ solver.minimize(params, &normalized);

  // TODO(keir): Better error handling.

  *normalized_x = normalized(0);
  *normalized_y = normalized(1);
}

void CameraIntrinsics::ComputeLookupGrid() {
  //TODO: interpolate sparse grid
  //TODO: float32 -> fixed point
  if(grid_) return;
  grid_ = new float[2*image_width()*image_height()];
  for (int y = 0; y < image_height(); y++) {
    for (int x = 0; x < image_width(); x++) {
      double image_x, image_y;
      ApplyIntrinsics((x-principal_point_x())/focal_length_x(),
                      (y-principal_point_y())/focal_length_y(),
                      &image_x,&image_y);
      grid_[(y*image_width()+x)*2+0] = image_x;
      grid_[(y*image_width()+x)*2+1] = image_y;
    }
  }
}

template<typename T,int N>
void CameraIntrinsics::Warp(const T* src, T* dst, int x0, int y0,
                                   int width, int height) {
  for (int y = x0; y < height; y++) {
    for (int x = y0; x < width; x++) {
      float image_x = grid_[(y*width+x)*2+0];
      float image_y = grid_[(y*width+x)*2+1];
      int ix = int(image_x), iy = int(image_y);
      if( ix < 0 || iy < 0 || ix >= width || iy > height ) {
        dst[y*width+x] = 0;
        continue;
      }
      // TODO: bilinear
      dst[y*width+x] = src[iy*width+ix];
    }
  }
}

void CameraIntrinsics::Undistort(const float* src, float* dst, int x0, int y0,
                                 int width, int height, int channels) {
  ComputeLookupGrid();
  if(channels==1) Warp<float,1>(src,dst,x0,y0,width,height);
  if(channels==2) Warp<float,2>(src,dst,x0,y0,width,height);
  if(channels==3) Warp<float,3>(src,dst,x0,y0,width,height);
  if(channels==4) Warp<float,4>(src,dst,x0,y0,width,height);
}

void CameraIntrinsics::Undistort(const unsigned char* src, unsigned char* dst, int x0, int y0,
                                 int width, int height, int channels) {
  ComputeLookupGrid();
  if(channels==1) Warp<unsigned char,1>(src,dst,x0,y0,width,height);
  if(channels==2) Warp<unsigned char,2>(src,dst,x0,y0,width,height);
  if(channels==3) Warp<unsigned char,3>(src,dst,x0,y0,width,height);
  if(channels==4) Warp<unsigned char,4>(src,dst,x0,y0,width,height);
}

}  // namespace libmv

// Copyright (c) 2009 libmv authors.
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

#ifndef LIBMV_IMAGE_IMAGE_CONVERTER_H
#define LIBMV_IMAGE_IMAGE_CONVERTER_H

#include "libmv/image/array_nd.h"
#include "libmv/image/image.h"

#include "opencv2/core/core.hpp"
#include "opencv2/core/types_c.h"
#include "opencv2/highgui/highgui.hpp"

namespace libmv{

// The factor comes from http://www.easyrgb.com/
// RGB to XYZ : Y is the luminance channel
// var_R * 0.2126 + var_G * 0.7152 + var_B * 0.0722
template<typename T>
inline T RGB2GRAY(const T r,const T g, const T b) {
  return static_cast<T>(r * 0.2126 + g * 0.7152 + b * 0.0722);
}

/*
// Specialization for the uchar type. (that do not want to work...)
template<>
inline unsigned char RGB2GRAY<unsigned char>(const unsigned char r,
                                             const unsigned char g,
                                             const unsigned char b) {
  return (unsigned char)(r * 0.2126 + g * 0.7152 + b * 0.0722 +0.5);
}*/

template<class ImageIn, class ImageOut>
void Rgb2Gray(const ImageIn &imaIn, ImageOut *imaOut) {

  assert( imaIn.Depth() == 3 );

  imaOut->Resize(imaIn.Height(), imaIn.Width(), 1);
  // Convert each RGB pixel into Gray value (luminance)

  for(int j = 0; j < imaIn.Height(); ++j) {
    for(int i = 0; i < imaIn.Width(); ++i)  {
      (*imaOut)(j,i) = RGB2GRAY(imaIn(j,i,0) , imaIn(j,i,1), imaIn(j,i,2));
    }
  }
}

// Convert from libmv's type to Mat (opencv type)
template<class T>
void Image2Mat( const Array3D<T> &imaIn, cv::OutputArray imaOut ) {
  int k, n, m;

  k = imaIn.Depth();
  n = imaIn.Height();
  m = imaIn.Width();

  // Create a bogus matrix referencing to the data, no copy is made
  cv::Mat_<T> imgTmp(n, m*k, const_cast<T*>(imaIn.Data()));

  // We could have a non-copy function for efficiency but that could create random bugs and we would need to count
  // references in both type. This function Image2Mat is not meant for speed anyway but for compatibility
  imgTmp.reshape(k,n).copyTo(imaOut);
}

static inline void Image2Mat( const Image &imaIn, cv::OutputArray imaOut ) {
  Array3Du * ima = imaIn.AsArray3Du();
  if (ima == NULL) {
    Array3Df * imaf = imaIn.AsArray3Df();
    Image2Mat<float>(*imaf, imaOut);
  } else
    Image2Mat<unsigned char>(*ima, imaOut);
}

template<class T>
void Mat2Image( const cv::Mat &imaIn, Array3D<T> &imaOut ) {
  int k, n, m;

  k = imaIn.channels();
  n = imaIn.rows;
  m = imaIn.cols;

  // Check if we need to deep copy and convert imaIn or not
  cv::Mat_<T> imaInT;
  if (imaInT.depth() == imaIn.depth())
    imaInT = imaIn;
  else
    imaIn.convertTo(imaInT, imaInT.depth());

  // Create an array without a deep copy
  Array3D<T> array(reinterpret_cast<T*>(const_cast<unsigned char*>(imaInT.data)), n, m, k);

  // And deep copy just for sure
  imaOut.CopyFrom(array);
}

static inline Image Mat2Image( const cv::Mat &imaIn) {
  if (imaIn.depth() == CV_8U) {
    Array3Du ima;
    Mat2Image( imaIn, ima );
    return Image(&ima);
  } else {
    Array3Df ima;
    Mat2Image( imaIn, ima );
    return Image(&ima);
  }
}

} // namespace libmv

#endif  // LIBMV_IMAGE_IMAGE_CONVERTER_H

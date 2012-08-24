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

#ifndef __OPENCV_SFM_NUMERIC_HPP__
#define __OPENCV_SFM_NUMERIC_HPP__

#ifdef __cplusplus

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

namespace cv
{

  template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
  void eigen2cv( const Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& src,
                 Matx<_Tp, _rows, _cols>& dst )
  {
      if( !(src.Flags & Eigen::RowMajorBit) )
      {
          dst = Matx<_Tp, _cols, _rows>(static_cast<const _Tp*>(src.data())).t();
      }
      else
      {
          dst = Matx<_Tp, _rows, _cols>(static_cast<const _Tp*>(src.data()));
      }
  }

  template<typename _Tp, int _rows, int _cols, int _options, int _maxRows, int _maxCols>
  void cv2eigen( const Matx<_Tp, _rows, _cols>& src,
                 Eigen::Matrix<_Tp, _rows, _cols, _options, _maxRows, _maxCols>& dst )
  {
      if( !(dst.Flags & Eigen::RowMajorBit) )
      {
          Mat _dst(_cols, _rows, DataType<_Tp>::type,
                   dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
          transpose(src, _dst);
          CV_DbgAssert(_dst.data == (uchar*)dst.data());
      }
      else
      {
          Mat _dst(_rows, _cols, DataType<_Tp>::type,
                   dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
          Mat(src).copyTo(_dst);
          CV_DbgAssert(_dst.data == (uchar*)dst.data());
      }
  }

  template<typename _Tp, int _rows, int _cols>
  void cv2eigen( const Matx<_Tp, _rows, _cols>& src,
                 Eigen::Matrix<_Tp, Eigen::Dynamic, Eigen::Dynamic>& dst )
  {
      dst.resize(_rows, _cols);
      if( !(dst.Flags & Eigen::RowMajorBit) )
      {
          Mat _dst(_cols, _rows, DataType<_Tp>::type,
               dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
          transpose(src, _dst);
          CV_DbgAssert(_dst.data == (uchar*)dst.data());
      }
      else
      {
          Mat _dst(_rows, _cols, DataType<_Tp>::type,
                   dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
          Mat(src).copyTo(_dst);
          CV_DbgAssert(_dst.data == (uchar*)dst.data());
      }
  }


  template<typename _Tp, int _rows>
  void cv2eigen( const Matx<_Tp, _rows, 1>& src,
                 Eigen::Matrix<_Tp, Eigen::Dynamic, 1>& dst )
  {
      dst.resize(_rows);

      if( !(dst.Flags & Eigen::RowMajorBit) )
      {
          Mat _dst(1, _rows, DataType<_Tp>::type,
                   dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
          transpose(src, _dst);
          CV_DbgAssert(_dst.data == (uchar*)dst.data());
      }
      else
      {
          Mat _dst(_rows, 1, DataType<_Tp>::type,
                   dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
          src.copyTo(_dst);
          CV_DbgAssert(_dst.data == (uchar*)dst.data());
      }
  }


  template<typename _Tp, int _cols>
  void cv2eigen( const Matx<_Tp, 1, _cols>& src,
                 Eigen::Matrix<_Tp, 1, Eigen::Dynamic>& dst )
  {
      dst.resize(_cols);
      if( !(dst.Flags & Eigen::RowMajorBit) )
      {
          Mat _dst(_cols, 1, DataType<_Tp>::type,
                   dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
          transpose(src, _dst);
          CV_DbgAssert(_dst.data == (uchar*)dst.data());
      }
      else
      {
          Mat _dst(1, _cols, DataType<_Tp>::type,
                   dst.data(), (size_t)(dst.stride()*sizeof(_Tp)));
          Mat(src).copyTo(_dst);
          CV_DbgAssert(_dst.data == (uchar*)dst.data());
      }
  }

CV_EXPORTS
void
meanAndVarianceAlongRows( const Mat &A,
                          Mat &mean,
                          Mat &variance );


/** Returns the skew anti-symmetric matrix of a vector */
CV_EXPORTS
Matx33d
skewMat( const Vec3d &x );

/** Returns the skew anti-symmetric matrix of a vector with only
 *  the first two (independent) lines */
CV_EXPORTS
Matx33d
skewMatMinimal( const Vec3d &x );


} /* namespace cv */

#endif /* __cplusplus */

#endif

/* End of file. */

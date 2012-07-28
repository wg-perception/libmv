
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

#include <opencv2/sfm/numeric.hpp>

#include "libmv/numeric/numeric.h"
#include <opencv2/core/eigen.hpp>

#include <iostream>

namespace cv
{

template<typename T>
void
meanAndVarianceAlongRows( const Mat &_A,
                          Mat &_mean,
                          Mat &_variance )
{
    libmv::Mat A;
    libmv::Vec mean, variance;

    cv2eigen( _A, A );

    libmv::MeanAndVarianceAlongRows( A, &mean, &variance );

    eigen2cv( mean, _mean );
    eigen2cv( variance, _variance );
}

void
meanAndVarianceAlongRows( const Mat &A,
                          Mat &mean,
                          Mat &variance )
{
    int depth = A.depth();
    if( depth == CV_32F )
    {
        // meanAndVarianceAlongRows<float>( A, mean, variance );
        std::cerr << "Function meanAndVarianceAlongRows not handled for float" << std::endl;
    }
    else
    {
        meanAndVarianceAlongRows<double>( A, mean, variance );
    }
}

} /* namespace cv */

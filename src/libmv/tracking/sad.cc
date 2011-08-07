/****************************************************************************
**
** Copyright (c) 2011 libmv authors.
**
** Permission is hereby granted, free of charge, to any person obtaining a copy
** of this software and associated documentation files (the "Software"), to
** deal in the Software without restriction, including without limitation the
** rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
** sell copies of the Software, and to permit persons to whom the Software is
** furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in
** all copies or substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
** IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
** FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
** AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
** LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
** FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
** IN THE SOFTWARE.
**
****************************************************************************/

#include "libmv/tracking/sad.h"
#include <stdlib.h>
#include <math.h>
#include <QDebug>

namespace libmv {

//! fixed point bilinear sample with precision k
template <int k> inline int sample(const ubyte* image,int stride, int x, int y, int u, int v) {
  const ubyte* s = &image[y*stride+x];
  return ((s[     0] * (k-u) + s[       1] * u) * (k-v)
        + (s[stride] * (k-u) + s[stride+1] * u) * (  v) ) / (k*k);
}

void SamplePattern(const ubyte* image, int stride, float x, float y, ubyte* pattern) {
  const int k = 256;
  int fx = lround(x*k), fy = lround(y*k);
  int ix = fx/k, iy = fy/k;
  int u = fx%k, v = fy%k;
  for (int i = 0; i < 16; i++) for (int j = 0; j < 16; j++) {
    pattern[i*16+j] = sample<256>(image,stride,ix+j-8,iy+i-8,u,v);
  }
}

#ifdef __SSE2__
#include <emmintrin.h>
inline uint SAD(const ubyte* pattern, const ubyte* image, int stride) {
  __m128i a = _mm_setzero_si128();
  for(int i = 0; i < 16; i++) {
    a = _mm_adds_epu16(a, _mm_sad_epu8( _mm_loadu_si128((__m128i*)(pattern+i*16)),
                                        _mm_loadu_si128((__m128i*)(image+i*stride))));
  }
  return _mm_extract_epi16(a,0) + _mm_extract_epi16(a,4);
}
#else
inline uint SAD(const ubyte* pattern, const ubyte* image, int stride) {
  uint sad=0;
  for(int i = 0; i < 16; i++) {
    for(int j = 0; j < 16; j++) {
      sad += abs((int)pattern[i*16+j] - image[i*stride+j]);
    }
  }
  return sad;
}
#endif

float sq( float x ) { return x*x; }
bool Track(const ubyte* pattern, const ubyte* image, int stride, int w, int h, float* px, float* py) {
  int ix = *px-8, iy = *py-8;
  uint min=-1;
  // integer pixel
  for(int y = 0; y < h-16; y++) {
    for(int x = 0; x < w-16; x++) {
      uint d = SAD(pattern,&image[y*stride+x],stride); //image L1 distance
      d += sq(x-w/2-8)+sq(y-h/2-8); //spatial L2 distance
      if(d < min) {
        min = d;
        ix = x, iy = y;
      }
    }
  }

  const int kPrecision = 4; //subpixel precision in bits
  const int kScale = 1<<kPrecision;
  int fx=0,fy=0;
#define LINEAR_SEARCH 1
#if FULL_SEARCH
  for(int y = -kScale+1; y < kScale; y++) {
    for(int x = -kScale+1; x < kScale; x++) {
      int sx = ix, sy = iy;
      int u = x, v = y;
      if( x < 0 ) sx--, u+=kScale;
      if( y < 0 ) sy--, v+=kScale;
      uint sad=0;
      for(int i = 0; i < 16; i++) {
        for(int j = 0; j < 16; j++) {
          sad += abs((int)pattern[i*16+j] - sample<kScale>(image,stride,sx+j,sy+i,u,v));
        }
      }
      if(sad < min) {
        min = sad;
        fx = x; fy = y;
      }
    }
  }
#elif LINEAR_SEARCH
  for(int k = 1; k <= kPrecision; k++) {
    fx *= 2, fy *= 2;
    int nx = fx, ny = fy;
    int p = kPrecision-k;
    for(int y = -1; y <= 1; y++) {
      for(int x = -1; x <= 1; x++) {
        uint sad=0;
        int sx = ix, sy = iy;
        int u = (fx+x)<<p, v = (fy+y)<<p;
        if( u < 0 ) u+=kScale, sx--;
        if( v < 0 ) v+=kScale, sy--;
        for(int i = 0; i < 16; i++) {
          for(int j = 0; j < 16; j++) {
            sad += abs((int)pattern[i*16+j] - sample<kScale>(image,stride,sx+j,sy+i,u,v));
          }
        }
        if(sad < min) {
          min = sad;
          nx = fx + x, ny = fy + y;
        }
      }
    }
    fx = nx, fy = ny;
  }
#endif
  /*qDebug().nospace()<< int(*px)-w/2 <<"."<< int(kScale*(*px-int(*px)))
              <<", "<< int(*py)-h/2 <<"."<< int(kScale*(*py-int(*py)))
            <<" -> "<< ix      -w/2+8 <<"."<< fx
              <<", "<< iy      -h/2+8 <<"."<< fy;*/
  *px = float((ix*kScale)+fx)/kScale+8;
  *py = float((iy*kScale)+fy)/kScale+8;
  return true;
}

}  // namespace libmv

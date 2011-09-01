/*
    Copyright (c) <year>, <copyright holder>
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
        * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
        * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
        * Neither the name of the <organization> nor the
        names of its contributors may be used to endorse or promote products
        derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY <copyright holder> ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL <copyright holder> BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef V4RCOLORS_H
#define V4RCOLORS_H

#include <opencv/cv.h>

namespace V4R {

static const double COLOR_ARRAY[] = { 255.,  0.,  0.,0., 
				        0.,255.,  0.,0., 
				        0.,  0.,255.,0., 
				      255.,255.,  0.,0., 
				      255.,  0.,255.,0., 
				        0.,255.,255.,0., 
				      128.,128.,128.,0., 
				      255.,128.,128.,0., 
				      128.,255.,128.,0., 
				      128.,128.,255.,0., 
				      255.,255.,128.,0., 
				      255.,128.,255.,0., 
				      128.,255.,255.,0.};
static const cv::Scalar *COLOR_TABLE = (cv::Scalar *) COLOR_ARRAY;
static const double COLOR_WHITE[] = {255.,255.,255.,0.};
static const int COLOR_BLACK[] = {  0.,  0.,  0.,0.};
  
class Colors
{
public:
  Colors();
  ~Colors();
    struct YUV {
        unsigned char y,u,v;
    };
    struct YUV422 {
        unsigned char y1,u,y2,v;
    };
    struct RGB {
        unsigned char r,g,b;
    };
    struct BGR {
        unsigned char b,g,r;
    };
    void RGB24toYUV422(const cv::Mat &rSrc, cv::Mat &rDes);
    void BGR24toYUV422(const cv::Mat &rSrc, cv::Mat &rDes);
    void YUV422toBGR24(const cv::Mat &rSrc, cv::Mat &rDes);
    void YUV422toRGB24(const cv::Mat &rSrc, cv::Mat &rDes);
    void BGR24toYUV(const cv::Mat &rSrc, cv::Mat &rDes);
    void RGB24toYUV(const cv::Mat &rSrc, cv::Mat &rDes);
    void YUVtoBGR24(const cv::Mat &rSrc, cv::Mat &rDes);
    void YUVtoRGB24(const cv::Mat &rSrc, cv::Mat &rDes);
    Colors::RGB YUVtoRGB (const Colors::YUV &yuv);
    static const cv::Scalar color(int idx){
      return COLOR_TABLE[idx];
    }
private:
    void initLut (  );
    void freeLut (  );    
    unsigned char RGB24_TO_Y (const unsigned char &r, const unsigned char &g, const unsigned char &b ) const ;
    unsigned char YR_TO_V (const unsigned char &r, const unsigned char &y ) const ;
    unsigned char YB_TO_U (const unsigned char &b, const unsigned char &y ) const ;
    unsigned char R_FROMYV ( const unsigned char &y, const unsigned char &v ) const;
    unsigned char G_FROMYUV ( const unsigned char &y, const unsigned char &u, const unsigned char &v ) const;
    unsigned char B_FROMYU ( const unsigned char &y, const unsigned char &u ) const;

    int *LutYr, *LutYg, *LutYb, *LutVr, *LutVrY, *LutUb, *LutUbY, *LutRv, *LutGu, *LutGv, *LutBu;
};
};
#endif // COLORS_H

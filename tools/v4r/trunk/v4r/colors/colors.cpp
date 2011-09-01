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

#include "colors.h"

#ifndef CLIP
#define CLIP(color) (unsigned char)(((color)>0xFF)?0xff:(((color)<0)?0:(color)))
#endif
#ifndef YfromRGB
#define YfromRGB(r,g,b) CLIP((77*(r)+150*(g)+29*(b))>>8)
#endif
#ifndef UfromRGB
#define UfromRGB(r,g,b) CLIP(((128*(b)-85*(g)-43*(r))>>8 )+128)
#endif
#ifndef VfromRGB
#define VfromRGB(r,g,b) CLIP(((128*(r)-107*(g)-21*(b))>>8) +128)
#endif

namespace V4R {

Colors::Colors()
        : LutYr(NULL)
        , LutYg(NULL)
        , LutYb(NULL)
        , LutVr(NULL)
        , LutVrY(NULL)
        , LutUb(NULL)
        , LutUbY(NULL)
        , LutRv(NULL)
        , LutGu(NULL)
        , LutGv(NULL)
        , LutBu(NULL) {
}
Colors::~Colors() {
    freeLut();
}
void Colors::initLut ( void ) {
    int i;
#define Rcoef 299
#define Gcoef 587
#define Bcoef 114
#define Vrcoef 711 //656 //877
#define Ubcoef 560 //500 //493 564

#define CoefRv 1402
#define CoefGu 714 // 344
#define CoefGv 344 // 714
#define CoefBu 1772

    LutYr = ( int* ) malloc ( 256*sizeof ( int ) );
    LutYg = ( int* ) malloc ( 256*sizeof ( int ) );
    LutYb = ( int* ) malloc ( 256*sizeof ( int ) );
    LutVr = ( int* ) malloc ( 256*sizeof ( int ) );
    LutVrY = ( int* ) malloc ( 256*sizeof ( int ) );
    LutUb = ( int* ) malloc ( 256*sizeof ( int ) );
    LutUbY = ( int* ) malloc ( 256*sizeof ( int ) );

    LutRv = ( int* ) malloc ( 256*sizeof ( int ) );
    LutGu = ( int* ) malloc ( 256*sizeof ( int ) );
    LutGv = ( int* ) malloc ( 256*sizeof ( int ) );
    LutBu = ( int* ) malloc ( 256*sizeof ( int ) );
    for ( i= 0;i < 256;i++ ) {
        LutYr[i] = i*Rcoef/1000 ;
        LutYg[i] = i*Gcoef/1000 ;
        LutYb[i] = i*Bcoef/1000 ;
        LutVr[i] = i*Vrcoef/1000;
        LutUb[i] = i*Ubcoef/1000;
        LutVrY[i] = 128 - ( i*Vrcoef/1000 );
        LutUbY[i] = 128 - ( i*Ubcoef/1000 );
        LutRv[i] = ( i-128 ) *CoefRv/1000;
        LutBu[i] = ( i-128 ) *CoefBu/1000;
        LutGu[i] = ( 128-i ) *CoefGu/1000;
        LutGv[i] = ( 128-i ) *CoefGv/1000;
    }
}
void Colors::freeLut ( void ) {
    free ( LutYr );
    free ( LutYg );
    free ( LutYb );
    free ( LutVr );
    free ( LutVrY );
    free ( LutUb );
    free ( LutUbY );
    free ( LutRv );
    free ( LutGu );
    free ( LutGv );
    free ( LutBu );
}

unsigned char Colors::RGB24_TO_Y ( const unsigned char &r, const unsigned char &g, const unsigned char &b ) const {
    return ( LutYr[ ( r ) ] + LutYg[ ( g ) ] + LutYb[ ( b ) ] );
}
unsigned char Colors::YR_TO_V ( const unsigned char &r, const unsigned char &y ) const {
    return ( LutVr[ ( r ) ] + LutVrY[ ( y ) ] );
}
unsigned char Colors::YB_TO_U ( const unsigned char &b, const unsigned char &y ) const {
    return ( LutUb[ ( b ) ] + LutUbY[ ( y ) ] );
}
unsigned char Colors::R_FROMYV ( const unsigned char &y, const unsigned char &v ) const {
    return CLIP ( ( y ) + LutRv[ ( v ) ] );
}
unsigned char Colors::G_FROMYUV ( const unsigned char &y, const unsigned char &u, const unsigned char &v ) const {
    return CLIP ( ( y ) + LutGu[ ( u ) ] + LutGv[ ( v ) ] );
}
unsigned char Colors::B_FROMYU ( const unsigned char &y, const unsigned char &u ) const {
    return CLIP ( ( y ) + LutBu[ ( u ) ] );
}

Colors::RGB Colors::YUVtoRGB (const Colors::YUV &yuv) {
    if (LutYr == NULL) initLut();
    Colors::RGB rgb;
    rgb.r = R_FROMYV  ( yuv.y, yuv.v  );
    rgb.g = G_FROMYUV ( yuv.y, yuv.u, yuv.v  );
    rgb.b = B_FROMYU  ( yuv.y, yuv.u );
    return rgb;
}

void Colors::BGR24toYUV422(const cv::Mat &rSrc, cv::Mat &rDes) {
    if ( rSrc.type() != CV_8UC3)  CV_Error( CV_StsUnsupportedFormat, "image are not CV_8UC3" );
    rDes.create(rSrc.size(), CV_8UC2);
    if (LutYr == NULL) initLut();
    int r = 0, g = 0, b = 0;
    BGR *src = (BGR *) rSrc.data;
    YUV422 *des = (YUV422 *) rDes.data;
    BGR *end = src  + ((rSrc.cols * rSrc.rows));
    while ( src != end ) {
        r = src->r, g = src->g, b = src->b;
        des->y1 = YfromRGB ( r, g, b );
        des->u  = UfromRGB ( r, g, b );
        src++;
        r = src->r, g = src->g, b = src->b;
        des->y2 = YfromRGB ( r, g, b );
        des->v  = VfromRGB ( r, g, b );
        src++;
        des++;
    }
}


void Colors::RGB24toYUV422(const cv::Mat &rSrc, cv::Mat &rDes) {
    if ( rSrc.type() != CV_8UC3)  CV_Error( CV_StsUnsupportedFormat, "image are not CV_8UC3" );
    rDes.create(rSrc.size(), CV_8UC2);
    if (LutYr == NULL) initLut();
    int r = 0, g = 0, b = 0;
    RGB *src = (RGB *) rSrc.data;
    YUV422 *des = (YUV422 *) rDes.data;
    RGB *end = src  + ((rSrc.cols * rSrc.rows));
    while ( src != end ) {
        r = src->r, g = src->g, b = src->b;
        des->y1 = YfromRGB ( r, g, b );
        des->u  = UfromRGB ( r, g, b );
        src++;
        r = src->r, g = src->g, b = src->b;
        des->y2 = YfromRGB ( r, g, b );
        des->v  = VfromRGB ( r, g, b );
        src++;
        des++;
    }
}

void Colors::YUV422toBGR24(const cv::Mat &rSrc, cv::Mat &rDes) {
    if ( rSrc.type() != CV_8UC2) CV_Error( CV_StsUnsupportedFormat, "image are not CV_8UC2" );
    rDes.create(rSrc.size(), CV_8UC3);
    if (LutYr == NULL) initLut();
    YUV422 *src = (YUV422 *) rSrc.data;
    BGR *des = (BGR *) rDes.data;
    YUV422 *end = src  + ((rSrc.cols * rSrc.rows) / 2);
    while ( src != end ) {
        des->r = R_FROMYV  ( src->y1, src->v );
        des->g = G_FROMYUV ( src->y1, src->u, src->v  );
        des->b = B_FROMYU  ( src->y1, src->u );
        des++;
        des->r = R_FROMYV  ( src->y2, src->v  );
        des->g = G_FROMYUV ( src->y2, src->u, src->v  );
        des->b = B_FROMYU  ( src->y2, src->u );
        des++;
        src++;
    }
}

void Colors::YUV422toRGB24(const cv::Mat &rSrc, cv::Mat &rDes) {
    if ( rSrc.type() != CV_8UC2) CV_Error( CV_StsUnsupportedFormat, "image are not CV_8UC2" );
    rDes.create(rSrc.size(), CV_8UC3);
    if (LutYr == NULL) initLut();
    YUV422 *src = (YUV422 *) rSrc.data;
    RGB *des = (RGB *) rDes.data;
    YUV422 *end = src  + ((rSrc.cols * rSrc.rows) / 2);
    while ( src != end ) {
        des->r = R_FROMYV  ( src->y1, src->v );
        des->g = G_FROMYUV ( src->y1, src->u, src->v  );
        des->b = B_FROMYU  ( src->y1, src->u );
        des++;
        des->r = R_FROMYV  ( src->y2, src->v  );
        des->g = G_FROMYUV ( src->y2, src->u, src->v  );
        des->b = B_FROMYU  ( src->y2, src->u );
        des++;
        src++;
    }
}

void Colors::BGR24toYUV(const cv::Mat &rSrc, cv::Mat &rDes) {
    if ( rSrc.type() != CV_8UC3)  CV_Error( CV_StsUnsupportedFormat, "image are not CV_8UC3" );
    rDes.create(rSrc.size(), CV_8UC3);
    if (LutYr == NULL) initLut();
    int r = 0, g = 0, b = 0;
    BGR *src = (BGR *) rSrc.data;
    YUV *des = (YUV *) rDes.data;
    BGR *end = src  + ((rSrc.cols * rSrc.rows));
    while ( src != end ) {
        des->y = YfromRGB ( r, g, b );
        des->u  = UfromRGB ( r, g, b );
        des->u  = VfromRGB ( r, g, b );
        src++;
        des++;
    }
}
void Colors::RGB24toYUV(const cv::Mat &rSrc, cv::Mat &rDes) {
    if ( rSrc.type() != CV_8UC3)  CV_Error( CV_StsUnsupportedFormat, "image are not CV_8UC3" );
    rDes.create(rSrc.size(), CV_8UC3);
    if (LutYr == NULL) initLut();
    int r = 0, g = 0, b = 0;
    RGB *src = (RGB *) rSrc.data;
    YUV *des = (YUV *) rDes.data;
    RGB *end = src  + ((rSrc.cols * rSrc.rows));
    while ( src != end ) {
        des->y = YfromRGB ( r, g, b );
        des->u  = UfromRGB ( r, g, b );
        des->u  = VfromRGB ( r, g, b );
        src++;
        des++;
    }
}
void Colors::YUVtoBGR24(const cv::Mat &rSrc, cv::Mat &rDes) {
    if ( rSrc.type() != CV_8UC3) CV_Error( CV_StsUnsupportedFormat, "image are not CV_8UC3" );
    rDes.create(rSrc.size(), CV_8UC3);
    if (LutYr == NULL) initLut();
    YUV *src = (YUV *) rSrc.data;
    BGR *des = (BGR *) rDes.data;
    YUV *end = src  + ((rSrc.cols * rSrc.rows) / 2);
    while ( src != end ) {
        des->r = R_FROMYV  ( src->y, src->v );
        des->g = G_FROMYUV ( src->y, src->u, src->v  );
        des->b = B_FROMYU  ( src->y, src->u );
        des++;
        src++;
    }
}
void Colors::YUVtoRGB24(const cv::Mat &rSrc, cv::Mat &rDes) {
    if ( rSrc.type() != CV_8UC3) CV_Error( CV_StsUnsupportedFormat, "image are not CV_8UC3" );
    rDes.create(rSrc.size(), CV_8UC3);
    if (LutYr == NULL) initLut();
    YUV *src = (YUV *) rSrc.data;
    RGB *des = (RGB *) rDes.data;
    YUV *end = src  + ((rSrc.cols * rSrc.rows) / 2);
    while ( src != end ) {
        des->r = R_FROMYV  ( src->y, src->v );
        des->g = G_FROMYUV ( src->y, src->u, src->v  );
        des->b = B_FROMYU  ( src->y, src->u );
        des++;
        src++;
    }
}
}

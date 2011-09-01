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

#include <cmvision.h>
#include "segmentation.h"

namespace V4R {
  
// returns index of least significant set bit
int log2modp[37] = {
    0, 1, 2,27, 3,24,28, 0, 4,17,25,31,29,12, 0,14, 5, 8,18,
    0,26,23,32,16,30,11,13, 7, 0,22,15,10, 6,21, 9,20,19
};

template <class num>
inline int bottom_bit(num n)
{
    return(log2modp[(n & -n) % 37]);
}

CMUSegmentation::CMUSegmentation() :
        mpVision(new CMVision) {

}
CMUSegmentation::~CMUSegmentation() {
    delete mpVision;
}


void CMUSegmentation::init() {
    mpVision->initialize(mSrc.cols, mSrc.rows);
    mpVision->enable ( CMV_DENSITY_MERGE | CMV_COLOR_AVERAGES );
}

void CMUSegmentation::perform(cv::Mat& rSrc)
{
    mSrc = rSrc;
    if ((mpVision->getWidth() != mSrc.cols) || (mpVision->getHeight() != mSrc.rows)) {
        init();
    }
    image_pixel *img = ( image_pixel * ) mSrc.data;
    mpVision->processFrame(img);

    mRegions.clear();
    mRegions.resize(mpVision->numColors());
    for (int c=0; c < mpVision->numColors(); c++) {
        mRegions[c].clear();
        CMVision::region *reg = mpVision->getRegions(c);
        CMUSegmentation::Region region;
        while (reg && reg->area>16) {
            region.bounding_box = cv::Rect(reg->x1, reg->y1, reg->x2 - reg->x1, reg->y2 - reg->y1);
            region.centroid = cv::Point2d(reg->cen_x, reg->cen_y);
            region.area = reg->area;
            reg = reg->next;
            mRegions[c].push_back(region);
        }
    }

}
void CMUSegmentation::testClassify(cv::Mat &rSrc) {
    int i,s;
    rgb black = {0,0,0};

    rgb * out = (rgb * ) rSrc.data;
    unsigned *map = mpVision->getMap();
    
    s = mpVision->getWidth() * mpVision->getHeight();

    CMVision::color_info *colors = mpVision->getColorInfo(0);
    
    i = 0;
    while (i < s) {
        while (i<s && !map[i]) {
            out[i] = black;
            i++;
        }
        while (i<s && map[i]) {
            out[i] = colors[bottom_bit(map[i])-1].color;
            i++;
        }
    }
}

void CMUSegmentation::addColor(const Colors &c) {
    CMVision::color_info ci;
    ci.color.red = c.color[0];
    ci.color.green= c.color[1];
    ci.color.blue = c.color[2];
    sprintf(ci.name,"%s",c.name.c_str());
    ci.merge = c.merge;
    ci.expected_num = c.expected_num;
    ci.y_low = c.y.start,  ci.y_high = c.y.end;
    ci.u_low = c.u.start,  ci.u_high = c.u.end;
    ci.v_low = c.v.start,  ci.v_high = c.v.end;
    mpVision->addOptions(&ci);
}

void CMUSegmentation::clearColors() {
  mpVision->clearColors();
}

void CMUSegmentation::loadColors(const std::string &filename) {
    mpVision->loadOptions((char*) filename.c_str());
    mColors.clear();
    mColors.resize(mpVision->numColors());
    for (int c = 0; c < mpVision->numColors(); c++) {
        CMVision::color_info *ci = mpVision->getColorInfo(c);
        mColors[c].color[0] = ci->color.red;
        mColors[c].color[1] = ci->color.green;
        mColors[c].color[2] = ci->color.blue;
        mColors[c].name = std::string(ci->name);
        mColors[c].merge = ci->merge;
        mColors[c].expected_num = ci->expected_num;
        mColors[c].y = cv::Range(ci->y_low, ci->y_high);
        mColors[c].u = cv::Range(ci->u_low, ci->u_high);
        mColors[c].v = cv::Range(ci->v_low, ci->v_high);
    }
}

}

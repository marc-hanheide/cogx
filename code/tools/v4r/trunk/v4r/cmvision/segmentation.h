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

#ifndef V4RSEGMENTATION_H
#define V4RSEGMENTATION_H

#include <opencv/cv.h>
#include <vector>

class CMVision;

namespace V4R {

class CMUSegmentation
{
public:

    class Region {
    public:
        cv::Rect bounding_box;
        cv::Point2d centroid;
        double area;
    };
    class Colors {
    public:
        void setColor(uchar r, uchar g, uchar b, std::string _name = "") {
            color[0] = r, color[1] = g, color[2] = b;
            if (!_name.empty()) name = _name;
        }
        void setRange(int y_low, int y_high, int u_low, int u_high, int v_low, int v_high) {
            y = cv::Range(y_low, y_high), u = cv::Range(u_low, u_high), v = cv::Range(v_low, v_high);
        }
        void setOption(double _merge, int _expected_num) {
            merge = _merge, expected_num = _expected_num;
        }
        char color[3];      // example color (such as used in test output)
        std::string name;   // color's meaninful name (e.g. ball, goal)
        double merge;       // merge density threshold
        int expected_num;   // expected number of regions (used for merge)
        cv::Range y, u, v;
    };



    CMUSegmentation();
    ~CMUSegmentation();
    void perform(cv::Mat &rSrc);
    void testClassify(cv::Mat &rSrc);
    std::vector< std::vector<CMUSegmentation::Region> > &getRegions() {
        return mRegions;
    }
    void addColor(const Colors &c);
    void clearColors();
    void loadColors(const std::string &filename);
    CMVision *cmv(){
      return mpVision;
    };
private:
    CMVision *mpVision;
    cv::Mat mSrc;
    std::vector< std::vector<CMUSegmentation::Region> > mRegions;
    std::vector<CMUSegmentation::Colors> mColors;
    void init();
};
}

#endif // CMVISION_H

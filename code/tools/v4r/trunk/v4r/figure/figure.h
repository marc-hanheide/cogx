/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2011  Markus Bader <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef V4R_FIGURE_H
#define V4R_FIGURE_H
#include <opencv/cv.h>
#include <opencv/cv.hpp>

namespace V4R {

class Figure
{
public:
    class Parameters {
    public:
        enum CoordinateSystem {
            left_handed = 0,
            right_handed = 1 // Austrian-Kangaroos System
        };
        Parameters();
        Parameters(const Parameters& param);
        std::string name;
        cv::Size size;
        cv::Rect_<double> roi;
        double alpha;
        CoordinateSystem system;
        double grid;
    };
    Figure();
    void init();
    void init(const Parameters& param);
    void clear();
    void show(int ms = 10);
    void plotOrigin(bool write = true);
    void setGrid(double boxSize, uchar r = 200, uchar g = 200, uchar b = 200);
    void setColor(uchar r, uchar g, uchar b, double opacity = 1);
    void setBackground(uchar r, uchar g, uchar b);
    void setFont(int fontFace);
    void plotLine(double x1, double y1, double x2, double y2, int thickness=1, int lineType=8);
    void plotArrow(double x1, double y1, double x2, double y2, int thickness=1, int lineType=8);
    void plotDot(double x, double y);
    void plotGrid();
    void plotText(double x, double y, const std::string& text, int ox = 0, int oy = 0);
    void plotText(double x, double y, double nr, int ox = 0, int oy = 0);
    void plotText(double x, double y, int nr, int ox = 0, int oy = 0);
    void plotCircle(double x, double y, double radius = 3, int thickness=1, int lineType=8);
    void plotCross(double x, double y, double radius = 3, int thickness=1, int lineType=8);
    cv::Point2d getWorld(double x, double y);
    cv::Mat &img(){
      return img_;
    }
    void plotState(double *p);
protected:
    Parameters param_;
    cv::Mat_<cv::Vec3b> img_;
    cv::Mat_<double> RT_;
    cv::Scalar colorPen_;
    cv::Scalar colorGrid_;
    cv::Scalar colorBackGround_;
    double opacity_;
    int fontFace_;
    double unit;
    double unitInv;
};


};

#endif // V4R_FIGURE_H

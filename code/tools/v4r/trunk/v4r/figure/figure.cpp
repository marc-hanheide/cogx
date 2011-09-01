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


#include "figure.h"
#include <iostream>
#include <iomanip>
#include "opencv/highgui.h"

namespace V4R {

void mouse_callback( int event, int x, int y, int flags, void* param ){
  Figure *fig = (Figure *) param;
  if(event == CV_EVENT_LBUTTONDOWN){
    cv::Point2d p = fig->getWorld(x,y);
    std::cout<< std::setprecision (5) << p.x << ", " << p.y << std::endl;
  }
}



inline cv::Point tp(const cv::Mat_<double> &m) {
    return cv::Point(round(m(0,0)), round(m(1,0)));
}
template <typename T>
inline void twist(T& a, T&b) {
    T tmp = a;
    a = b;
    b = tmp;
}

Figure::Parameters::Parameters()
        : name("plot")
        , size(400, 600)
        , roi(-2,-3, 4, 6)
        , alpha(M_PI/2.)
        , system(right_handed)
        , grid(1) {
}
Figure::Parameters::Parameters(const V4R::Figure::Parameters& r) {
    name = r.name;
    size = r.size;
    roi = r.roi;
    alpha = r.alpha;
    system = r.system;
    grid = r.grid;
}


Figure::Figure()
        : param_(Parameters())
        , img_()
        , RT_()
        , colorPen_(0,0,0,1)
        , colorGrid_(200,200,200)
        , colorBackGround_(255,255,255,255)
        , opacity_(1.0)
        , fontFace_(cv::FONT_HERSHEY_PLAIN)
        , unit(1.0) {
    init();
}

void Figure::init() {
    img_ = cv::Mat::zeros(param_.size.height, param_.size.width, CV_8UC3);
    cv::namedWindow ( param_.name, 1 ); 
    cvSetMouseCallback( param_.name.c_str(), mouse_callback, (void*) this);

    RT_ = cv::Mat_<double>::eye(3,3);
    unit = ((double) param_.size.width) / 100;
    unitInv = param_.roi.width / 100;
    double c = cos(param_.alpha), s = sin(param_.alpha);
    double sx = ((double)param_.size.width)  / (c*param_.roi.width - s*param_.roi.height);
    double sy = ((double)param_.size.height) / (s*param_.roi.width + c*param_.roi.height);
    double tx = param_.size.width/2.0;
    double ty = param_.size.height/2.0;
    if (param_.system == V4R::Figure::Parameters::left_handed) {
        std::cerr << "left_handed systems are not suppored" << std::endl;
        return;
    }
    if (param_.system == V4R::Figure::Parameters::right_handed) {
        double c = cos(param_.alpha), s = sin(-param_.alpha);
        RT_(0,0) = c*sx, RT_(0,1) =  s*sx, RT_(0,2) = tx;
        RT_(1,0) = s*sy, RT_(1,1) = -c*sy, RT_(1,2) = ty;
    }
}
void Figure::init(const V4R::Figure::Parameters& param) {
    param_ = param;
    init();
}

void Figure::clear() {
    img_.setTo(colorBackGround_);

}

void Figure::setBackground(uchar r, uchar g, uchar b)
{
    colorBackGround_ = cv::Scalar(b, g, r);
}


void Figure::plotGrid() {
    cv::Scalar pen = colorPen_;
    colorPen_ = colorGrid_;
    int x_max = (param_.roi.x + param_.roi.width) / param_.grid;
    int x_min = (param_.roi.x) / param_.grid;
    if (x_max < x_min) twist(x_max, x_min);
    for (int i = x_min; i <= x_max; i++) {
        double x = param_.grid * i;
        plotLine(x, param_.roi.y, x, param_.roi.y + param_.roi.height);
        if (i != x_min && i != x_max ) plotText(x, param_.roi.y, x, 5, -10);
        if (i == x_max) plotText(x, param_.roi.y, "X", 5, +20);
    }
    int y_max = (param_.roi.y + param_.roi.height) / param_.grid;
    int y_min = (param_.roi.y) / param_.grid;
    if (y_max < y_min) twist(y_max, y_min);
    for (int i = y_min; i <= y_max; i++) {
        double y = param_.grid * i;
        plotLine(param_.roi.x, y, param_.roi.x + param_.roi.width, y);
        if (i != y_min && i != y_max ) plotText(param_.roi.x, y, y, 5, -10);
        if (i == y_min) plotText(param_.roi.x, y, "Y", -20, -10);
    }

    colorPen_ = pen;
}

void Figure::show(int ms) {
    cv::imshow(param_.name, img_);
    cv::waitKey(ms);
}

void Figure::setGrid(double boxSize, uchar r, uchar g, uchar b) {
    param_.grid = boxSize;
    colorGrid_ = cv::Scalar(b, g, r);
}

void Figure::plotOrigin(bool write)
{
    double l = (param_.roi.width / 10) ;


    cv::Mat_<double> o = (cv::Mat_<double>(3,1) << 0, 0, 1); 
    cv::Mat_<double> px = (cv::Mat_<double>(3,1) << l, 0, 1); 
    cv::Mat_<double> py = (cv::Mat_<double>(3,1) << 0, l, 1); 
    cv::Mat_<double>  oi = RT_ * o;
    cv::Mat_<double>  pxi = RT_ * px;
    cv::Mat_<double>  pyi = RT_ * py;
    cv::line(img_, tp(oi), tp(pxi), cv::Scalar(0,0,255),1);
    cv::line(img_, tp(oi), tp(pyi), cv::Scalar(255,0,0),1);
    if (write) {
        cv::putText(img_, "X", cv::Point(pxi(0,0)+10,pxi(1,0)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,0,255));
        cv::putText(img_, "Y", cv::Point(pyi(0,0),pyi(1,0)+10), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255,0,0));
    }
}
void Figure::setFont(int fontFace) {
    fontFace_ = fontFace;
}

void Figure::setColor(uchar r, uchar g, uchar b, double opacity)
{
    colorPen_ = cv::Scalar(b, g, r, opacity);
    opacity_ = opacity;
}

void Figure::plotDot(double x, double y)
{
    cv::Mat_<double> pw = (cv::Mat_<double>(3,1) << x, y, 1); 
    cv::Mat_<double> p = RT_ * pw;
    img_(tp(p)) = cv::Vec3b(colorPen_[0], colorPen_[1], colorPen_[2]);
}

void Figure::plotLine(double x1, double y1, double x2, double y2, int thickness, int lineType)
{
    cv::Mat_<double> p1w = (cv::Mat_<double>(3,1) << x1, y1, 1); 
    cv::Mat_<double> p2w = (cv::Mat_<double>(3,1) << x2, y2, 1); 
    cv::Mat_<double> p1 = RT_ * p1w;
    cv::Mat_<double> p2 = RT_ * p2w;
    cv::line(img_, tp(p1), tp(p2), colorPen_, thickness, lineType);
}

void Figure::plotArrow(double x1, double y1, double x2, double y2, int thickness, int lineType)
{
    cv::Mat_<double> p1w = (cv::Mat_<double>(3,1) << x1, y1, 1); 
    cv::Mat_<double> p2w = (cv::Mat_<double>(3,1) << x2, y2, 1); 
    cv::Mat_<double> p1 = RT_ * p1w;
    cv::Mat_<double> p2 = RT_ * p2w;
    cv::Mat_<double> v = p2 - p1;
    double a = atan2(v(1,0), v(0,0));
    double l = unit*2;
    cv::Mat_<double> v1 = (cv::Mat_<double>(3,1) << cos(a+M_PI/8)*l, sin(a+M_PI/8)*l, 0); 
    cv::Mat_<double> v2 = (cv::Mat_<double>(3,1) << cos(a-M_PI/8)*l, sin(a-M_PI/8)*l, 0); 
    cv::line(img_, tp(p1), tp(p2), colorPen_, thickness, lineType);
    cv::line(img_, tp(p2), tp(p2-v1), colorPen_, thickness, lineType);
    cv::line(img_, tp(p2), tp(p2-v2), colorPen_, thickness, lineType);
}

void Figure::plotText(double x, double y, const std::string& text, int ox, int oy)
{
    cv::Mat_<double> pw = (cv::Mat_<double>(3,1) << x, y, 1); 
    cv::Mat_<double> p = RT_ * pw;
    cv::putText(img_, text, cv::Point(p(0,0)+ox,p(1,0)+oy), fontFace_, 1, colorPen_);
}
void Figure::plotText(double x, double y, double nr, int ox, int oy)
{
    std::stringstream ss;
    ss << std::setprecision (5) << nr;
    plotText(x, y, ss.str(), ox, oy);
}
void Figure::plotText(double x, double y, int nr, int ox, int oy)
{
    std::stringstream ss;
    ss << nr;
    plotText(x, y, ss.str(), ox, oy);
}

void Figure::plotCircle(double x, double y, double radius, int thickness, int lineType)
{
    int r = radius * unit;
    cv::Mat_<double> pw = (cv::Mat_<double>(3,1) << x, y, 1); 
    cv::Mat_<double> p = RT_ * pw;
    cv::circle(img_, tp(p), r, colorPen_, thickness, lineType);
}

void Figure::plotCross(double x, double y, double radius, int thickness, int lineType)
{
    int r = radius * unit;
    cv::Mat_<double> pw = (cv::Mat_<double>(3,1) << x, y, 1); 
    cv::Mat_<double> p = RT_ * pw;
    cv::line(img_, cv::Point(p(0,0)+r,p(1,0)+r), cv::Point(p(0,0)-r,p(1,0)-r), colorPen_, thickness, lineType);
    cv::line(img_, cv::Point(p(0,0)-r,p(1,0)+r), cv::Point(p(0,0)+r,p(1,0)-r), colorPen_, thickness, lineType);
}

cv::Point2d Figure::getWorld(double x, double y)
{
    cv::Mat_<double> pw = (cv::Mat_<double>(3,1) << x, y, 1); 
    cv::Mat_<double> p = RT_.inv() * pw;
  return cv::Point2d(p(0,0), p(1,0));
}

void Figure::plotState(double* p)
{
  double x1 = p[0];
  double y1 = p[1];
  plotCircle(x1, y1, 1);
  double l = 1.*unitInv;
  double a = p[2];
  double dx = cos(a)*l;
  double dy = sin(a)*l;
  plotLine(x1, y1, x1+dx, y1+dy);
}


}




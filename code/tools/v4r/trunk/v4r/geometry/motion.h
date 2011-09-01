/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) <year>  <name of author>

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#ifndef V4R_MOTION_H
#define V4R_MOTION_H

#include <opencv2/opencv.hpp>

namespace V4R {
template <typename T>
class LinearMotion {

private:
    cv::Mat_<T> s_;
    cv::Mat_<T> cov_;
    double locationVarinaz_;
    double velocityVarinaz_;
public:
    LinearMotion()
            : s_(6,1), cov_(6,6), locationVarinaz_(-1.), velocityVarinaz_(-1.) {
    }
    LinearMotion(const LinearMotion &m)
            : s_(m.s_)
            , cov_(m.cov_)
            , locationVarinaz_(m.locationVarinaz())
            , velocityVarinaz_(m.velocityVarinaz()) {
    }
    LinearMotion(const cv::Mat_<T> &s)
            : s_(s), cov_(6,6), locationVarinaz_(-1.), velocityVarinaz_(-1.) {
    }
    LinearMotion(const cv::Mat_<T> &s, const cv::Mat_<T> &cov)
            : s_(s), cov_(cov), locationVarinaz_(-1.), velocityVarinaz_(-1.) {
    }
    LinearMotion(const std::vector< cv::Mat_<T> > &states){
      calcCov(states, cov_, s_, locationVarinaz_, velocityVarinaz_);
    }
    void copyTo(LinearMotion<T> &m) const {
        s_.copyTo(m.state());
        cov_.copyTo(m.covarianz());
        m.locationVarinaz() = locationVarinaz_;
        m.velocityVarinaz() = velocityVarinaz_;
    }
    void location(cv::Point3_<T> &p) const {
        p.x = s_(0,0), p.y = s_(1,0), p.z = s_(2,0);
    }
    void setLocation(const cv::Point3_<T> &p) {
        s_(0,0) = p.x, s_(1,0) = p.y, s_(2,0) = p.z;
    }
    void location(cv::Point3_<T> &p, double dt) const {
        p.x = s_(0,0) + s_(3,0) * dt, p.y = s_(1,0) + s_(4,0) * dt, p.z = s_(2,0) + s_(5,0) * dt;
    }
    void velocity(cv::Vec<T, 3> &v) const {
        v[0] = s_(3,0), v[1] = s_(4,0), v[2] = s_(5,0);
    }
    const T &velocityVarinaz() const {
        return velocityVarinaz_;
    }
    T &velocityVarinaz() {
        return velocityVarinaz_;
    }
    const T &locationVarinaz() const {
        return locationVarinaz_;
    }
    T &locationVarinaz() {
        return locationVarinaz_;
    }
    const T &x() const {
        return s_(0,0);
    }
    T &x() {
        return s_(0,0);
    }
    const T &y() const {
        return s_(1,0);
    }
    T &y() {
        return s_(1,0);
    }
    const T &z() const {
        return s_(2,0);
    }
    T &z() {
        return s_(2,0);
    }
    const T &vx() const {
        return s_(3,0);
    }
    T &vx() {
        return s_(3,0);
    }
    const T &vy() const {
        return s_(4,0);
    }
    T &vy() {
        return s_(4,0);
    }
    const T &vz() const {
        return s_(5,0);
    }
    T &vz() {
        return s_(5,0);
    }
    const cv::Mat_<T> &state() const {
        return s_;
    }
    cv::Mat_<T> &state() {
        return s_;
    }
    const cv::Mat_<T> &covarianz() const {
        return cov_;
    }
    cv::Mat_<T> &covarianz() {
        return cov_;
    }
    const T &state(int i) const {
        return s_(i,0);
    }
    T &state(int i) {
        return s_(i,0);
    }
    const T &covarianz(int r, int c) const {
        return cov_(r, c);
    }
    T &covarianz(int r, int c) {
        return cov_(r, c);
    }
    std::string human_readable(bool more = false) const {
        std::stringstream ss;
        ss << "s = [ " << std::setw(8) << std::showpos << std::setprecision (5) << x();
        ss << "; " << std::setw(8) << std::showpos << std::setprecision (5) << y();
        ss << "; " << std::setw(8) << std::showpos << std::setprecision (5) << z();
        ss << "; " << std::setw(8) << std::showpos << std::setprecision (5) << vx();
        ss << "; " << std::setw(8) << std::showpos << std::setprecision (5) << vy();
        ss << "; " << std::setw(8) << std::showpos << std::setprecision (5) << vz() << " ]";
        ss << "; locVar = " << std::setw(8) << std::showpos << std::setprecision (5) << velocityVarinaz();
        ss << "; velVar = " << std::setw(8) << std::showpos << std::setprecision (5) << velocityVarinaz();
        ss << "; " << std::setw(8) << std::showpos << std::setprecision (5) << y();
        if (more) {
            ss << std::endl;
            ss << "; cov = [";
            for (int r = 0; r < cov_.rows; r++) {
                for (int c = 0; c < cov_.cols; c++) {
                    if (c > 0) ss << ", ";
                    ss << std::setw(8) << std::showpos << std::setprecision (5) << cov_(r,c);
                }
                if (r < cov_.rows-1)  ss << "; ";
            }
            ss << "];";
        }
        return ss.str();
    }
    
    void calcCov(const std::vector<cv::Mat_<T> > &states, cv::Mat &cov, cv::Mat &mean, double &locVar, double &velVar){
      cv::Mat_<double> samples(6,states.size());
      for (unsigned int i = 0; i < states.size(); i++) {
	samples.col(i) = states[i];	
      }
      cv::calcCovarMatrix(samples, cov, mean, CV_COVAR_NORMAL | CV_COVAR_ROWS);
      mean = cv::Mat_<T>::zeros(6,1);
      for (unsigned int i = 0; i < states.size(); i++) {
        mean += states[i];
      }
      mean = mean * 1./(double) states.size();  
      std::map< double, const cv::Mat_<T>*> weighted;
      for (unsigned int i = 0; i < states.size(); i++) {
        cv::Mat_<T> diff =  (mean - states[i]);
        double d =  sqrt(diff(0,0)*diff(0,0) + diff(1,0)*diff(1,0) + diff(2,0)*diff(2,0));      
        weighted[d] = &states[i];
      }
      
    }
};
typedef LinearMotion<double> LinearMotionD;
};

template <typename T>
inline std::ostream& operator << ( std::ostream &os, const V4R::LinearMotion<T> &r) {
    os << r.human_readable();
    return os;
};

#endif // MOTION_H

/*
    Copyright (c) 2011, Markus Bader
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

#ifndef V4R_TRANSFORMATION_H
#define V4R_TRANSFORMATION_H

#include <opencv/cv.h>
#include <sys/time.h>
#include <v4r/cvextensions/operator_cv.h>
#include <iomanip>
#include <iostream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string.hpp>

namespace V4R {
#ifndef V4R_TRANSFORMATION
#define V4R_TRANSFORMATION

template <typename T>
class Transformation {
public:
    cv::Vec<T,3> tvec;
    cv::Vec<T,3> rvec;
public:
    Transformation() {};
    Transformation(const Transformation &r)
            : tvec(r.tvec)
            , rvec(r.rvec) {};

    template <typename T2>
    Transformation(const cv::Vec<T2,3> &rTvec, const cv::Vec<T2,3> &rRvec)
            : tvec(rTvec)
            , rvec(rRvec) {   }
    template <typename T2>
    Transformation(const std::vector<T2> &rTRvec) {
        setTF(rTRvec);
    }
    Transformation(const cv::Mat &rTvec, const cv::Mat &rRvec) {
        setTF(rTvec, rRvec);
    }
    void setT(const cv::Mat &rTvec) {
        cv::copy(rTvec, tvec);
    }
    void setT(const double *pTvec) {
	tvec[0] = (T) pTvec[0], tvec[1] = (T) pTvec[1], tvec[2] = (T) pTvec[2];
    }
    void setT(const float *pTvec) {
	tvec[0] = (T) pTvec[0], tvec[1] = (T) pTvec[1], tvec[2] = (T) pTvec[2];
    }
    void setR(const cv::Mat &rRvec) {
        cv::copy(rRvec, rvec);
    }
    void setR(const double *pRvec) {
	rvec[0] = (T) pRvec[0], rvec[1] = (T) pRvec[1], rvec[2] = (T) pRvec[2];
    }
    void setR(const float *pRvec) {
	rvec[0] = (T) pRvec[0], rvec[1] = (T) pRvec[1], rvec[2] = (T) pRvec[2];
    }
    void setTF(const cv::Mat_<T> &rRTMat) {
        tvec[0] = rRTMat(0,3);
        tvec[1] = rRTMat(1,3);
        tvec[2] = rRTMat(2,3);
        cv::Mat R(rRTMat, cv::Rect(0, 0, 3, 3));
        cv::Mat_<T> r;
        cv::Rodrigues(R, r);
        rvec[0] = r(0,0);
        rvec[1] = r(1,0);
        rvec[2] = r(2,0);
    }
    void setTF(const cv::Mat &rTvec, const cv::Mat &rRvec) {
        setT(rTvec), setR(rRvec);
    }
    void setTF(double *pTvec, double *pRvec) {
        tvec[0] = pTvec[0], tvec[1] = pTvec[1], tvec[2] = pTvec[2];
        rvec[0] = pRvec[0], rvec[1] = pRvec[1], rvec[2] = pRvec[2];
    }
    void setTF(const float *pTvec, const float *pRvec) {
        tvec[0] = pTvec[0], tvec[1] = pTvec[1], tvec[2] = pTvec[2];
        rvec[0] = pRvec[0], rvec[1] = pRvec[1], rvec[2] = pRvec[2];
    }
    template <typename T2>
    void setTF(const cv::Vec<T2,3> &rTvec, const cv::Vec<T2,3> &rRvec) {
        tvec[0] = rTvec[0], tvec[1] = rTvec[1], tvec[2] = rTvec[2];
        rvec[0] = rRvec[0], rvec[1] = rRvec[1], rvec[2] = rRvec[2];
    }
    template <typename T2>
    void setTF(const std::vector<T2> &rTRvec) {
        if (rTRvec.size() != 6) return;
        tvec[0] = rTRvec[0], tvec[1] = rTRvec[1], tvec[2] = rTRvec[2];
        rvec[0] = rTRvec[3], rvec[1] = rTRvec[4], rvec[2] = rTRvec[5];
    }
    template <typename T2>
    void setTF(const cv::Vec<T2, 6> &rTRvec) {
        tvec[0] = rTRvec[0], tvec[1] = rTRvec[1], tvec[2] = rTRvec[2];
        rvec[0] = rTRvec[3], rvec[1] = rTRvec[4], rvec[2] = rTRvec[5];
    }
    const T &x() const {
        return tvec[0];
    }
    T &x() {
        return tvec[0];
    }
    const T &y() const {
        return tvec[1];
    }
    T &y() {
        return tvec[1];
    }
    const T &z() const {
        return tvec[2];
    }
    T &z() {
        return tvec[2];
    }
    const T &wx() const {
        return rvec[0];
    }
    T &wx() {
        return rvec[0];
    }
    const T &wy() const {
        return rvec[1];
    }
    T &wy() {
        return rvec[1];
    }
    const T &wz() const {
        return rvec[2];
    }
    T &wz() {
        return rvec[2];
    }
    cv::Mat TVec() const {
        return (cv::Mat_<double>(3,1) << tvec[0], tvec[1], tvec[2]);
    }
    cv::Mat RVec() const {
        return (cv::Mat_<double>(3,1) << rvec[0], rvec[1], rvec[2]);
    }
    cv::Mat RTMat() const {
        cv::Mat_<double> RT = cv::Mat_<double> ::eye(4,4);
        RT(0,3) =  tvec[0], RT(1,3) =  tvec[1], RT(2,3) =  tvec[2];
        cv::Mat R(RT, cv::Rect(0, 0, 3, 3));
        cv::Rodrigues(RVec(), R);
        return RT;
    }
    void clear() {
        tvec[0] = 0, tvec[1] = 0, tvec[2] = 0;
        rvec[0] = 0, rvec[1] = 0, rvec[2] = 0;
    }
    /**
     * produces a human readable string
     * @return string t = %-8.3f, %-8.3f, %-8.3f; r = %-5.4f, %-5.4f, %-5.4f;
     **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << "t = [ ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << tvec[0] << ", ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << tvec[1] << ", ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << tvec[2] << "]; ";
        ss << "r= [ ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << rvec[0] << ", ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << rvec[1] << ", ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << rvec[2] << "]; ";
        return ss.str();
    }
    /**
     * reads a string into the data
     * @param str tx, ty, tz, wx, wy, wz
     * @return zero on success
     **/
  int human_readable(const std::string &str) {
    if (str.empty()) return 1;
    std::string tmp = str;
    std::vector<std::string> values;
    boost::erase_all(tmp, " ");
    boost::split(values, tmp, boost::is_any_of(";,"));
    if(values.size() != 3) return 1;
    if (values.size() > 0) x() = boost::lexical_cast<double>(values[0]);
    if (values.size() > 1) y() = boost::lexical_cast<double>(values[1]);
    if (values.size() > 2) z() = boost::lexical_cast<double>(values[2]);
    if (values.size() > 3) wx() = boost::lexical_cast<double>(values[0]);
    if (values.size() > 4) wy() = boost::lexical_cast<double>(values[1]);
    if (values.size() > 5) wz() = boost::lexical_cast<double>(values[2]);
    return 0;
  }
};

template <typename T>
class TFLink : public Transformation<T> {
public:
    TFLink() {}
    template <typename T2>
    TFLink(const TFLink<T> &r) {
        r.copyTo(this);
    }
    template <typename T2>
    TFLink(const cv::Vec<T2,3> &rTvec, const cv::Vec<T2,3> &rRvec, const std::string &rSrc, const std::string &rDes, const timeval &t, double conficence)
            : Transformation<T>(rTvec, rRvec) {
        setLink(rSrc, rDes, t, conficence);
    }
    template <typename T2>
    TFLink(const std::vector<T2> &rTRvec, const std::string &rSrc, const std::string &rDes, const timeval &t, double conficence)
            : Transformation<T>(rTRvec) {
        setLink(rSrc, rDes, t, conficence);
    }
    template <typename T2>
    TFLink(const cv::Vec<T2,3> &rTvec, const cv::Vec<T2,3> &rRvec)
            : Transformation<T>(rTvec, rRvec)
            , cf(0) {}

    template <typename T2>
    TFLink(const std::vector<T2> &rTRvec)
            : Transformation<T>(rTRvec)
            , cf(0) {    }
    TFLink(const cv::Mat &rTvec, const cv::Mat &rRvec)
            : Transformation<T>(rTvec, rRvec)
            , cf(0) {    }
    char src[32]; /// link base name
    char des[32]; /// link destination name
    double cf; ///confidence
    double confidence() const {
        return cf;
    }
    std::string getNameSrc() const {
        return std::string(src);
    }
    std::string getNameDes() const {
        return std::string(des);
    }
    timeval tstamp;
    void setNames(const std::string &rSrc, const std::string &rDes) {
        if (rSrc.length() < 32) strcpy(src, rSrc.c_str());
        else std::cerr << "TFLink link name to long: " << rSrc << std::endl;
        if (rDes.length() < 32) strcpy(des, rDes.c_str());
        else std::cerr << "TFLink link name to long: " << rDes << std::endl;
    }
    void setLink(const std::string &rSrc, const std::string &rDes,  const timeval &t, double conficence) {
        if (rSrc.length() < 32) strcpy(src, rSrc.c_str());
        else std::cerr << "TFLink link name to long: " << rSrc << std::endl;
        if (rDes.length() < 32) strcpy(des, rDes.c_str());
        else std::cerr << "TFLink link name to long: " << rDes << std::endl;
        tstamp = t;
        cf = conficence;
    }
    template <typename T2>
    void setLink(const cv::Vec<T2,3> &rTvec, const cv::Vec<T2,3> &rRvec, const std::string &rSrc, const std::string &rDes, const timeval &t, double conficence) {
        setTF(rTvec, rRvec);
        setNames(rSrc, rDes);
        cf = conficence, tstamp = t;
    }
    template <typename T2>
    void setLink(const std::vector<T2> &rTRvec, const std::string &rSrc, const std::string &rDes, const timeval &t, double conficence) {
        this->set(rTRvec);
        setNames(rSrc, rDes);
        cf = conficence, tstamp = t;
    }

    void clear() {
        Transformation<T>::clear();
        sprintf(src, "src_NA"), sprintf(des, "des_NA");
        cf = 0, tstamp.tv_sec = 0, tstamp.tv_usec = 0;
    }
    template <typename T2>
    void copyTo(TFLink<T2> &r) const {
        setTF(r.tvec, r.rvec);
        strcpy(src, r.src), strcpy(des, r.des);
        cf = r.cf, tstamp = r.tstamp;
    }
    /**
     * produces a human readable string
     * @return string t = %-8.3f, %-8.3f, %-8.3f; r = %-5.4f, %-5.4f, %-5.4f;
     **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << Transformation<T>::human_readable();
        ss << " cf: " << std::setw(8) << std::setprecision (3) << cf;
        ss << " " << getNameSrc() << " --> " << getNameDes();
        return ss.str();
    }

};
typedef TFLink<double> TFLinkD;


#endif// V4R_TRANSFORMATION
}
#endif // V4R_TRANSFORMATION_H

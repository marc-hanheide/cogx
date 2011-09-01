/**
 * @file cvoperators.h
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#ifndef CVEXT_OPERATORS_H
#define CVEXT_OPERATORS_H

#include <opencv/cv.h>

template <typename T>
inline cv::Point_<T> operator - (const cv::Point_<T> &p, const cv::Vec<T,2> &v) {
    return cv::Point_<T>(p.x - v[0], p.y - v[1]);
};

template <typename T>
inline cv::Vec<T,2> operator - (const cv::Vec<T,2> &v, const cv::Point_<T> &p) {
    return cv::Vec<T,2>(v[0] - p.x, v[1] - p.y);
};

template <typename T>
inline cv::Vec<T,2> &operator -= (cv::Vec<T,2> &v, const cv::Point_<T> &p) {
    v[0] -= p.x, v[1] -= p.y;
    return v;
};

template <typename T>
inline cv::Point_<T> &operator -= (cv::Point_<T> &p, const cv::Vec<T,2> &v) {
    p.x -= v[0], p.y -= v[1];
    return p;
};

template <typename T>
inline cv::Point_<T> operator + (const cv::Point_<T> &p, const cv::Vec<T,2> &v) {
    return cv::Point_<T>(p.x + v[0], p.y + v[1]);
};

template <typename T>
inline cv::Vec<T,2> operator + (const cv::Vec<T,2> &v, const cv::Point_<T> &p) {
    return cv::Vec<T,2>(v[0] + p.x, v[1] + p.y);
};
template <typename T>
inline cv::Vec<T,2> &operator += (cv::Vec<T,2> &v, const cv::Point_<T> &p) {
    v[0] += p.x, v[1] += p.y;
    return v;
};

template <typename T>
inline cv::Point_<T> &operator += (cv::Point_<T> &p, const cv::Vec<T,2> &v) {
    p.x += v[0], p.y += v[1];
    return p;
};


template <typename T>
inline cv::Mat operator * (const cv::Mat &M, const cv::Point_<T> &p) {
    cv::Mat_<T> v = cv::Mat_<T>(3,1);
    T *pM = (T*) M.data;
    v(0,0) = pM[0] * p.x + pM[1] * p.y + pM[2];
    v(1,0) = pM[3] * p.x + pM[4] * p.y + pM[5];
    v(2,0) = pM[6] * p.x + pM[7] * p.y + pM[8];
    return v;
};
template <typename T>
inline cv::Mat operator * (const cv::Mat &M, const std::vector< cv::Point_<T> > &points) {
    cv::Mat_<T> v = cv::Mat_<T>(3,points.size());
    T *pM = (T*) M.data;
    for (int i = 0; i < points.size(); i++) {
        v(0,i) = pM[0] * points[i].x + pM[1] * points[i].y + pM[2];
        v(1,i) = pM[3] * points[i].x + pM[4] * points[i].y + pM[5];
        v(2,i) = pM[6] * points[i].x + pM[7] * points[i].y + pM[8];
    }
    return v;
};

#endif //CVEXT_OPERATORS_H

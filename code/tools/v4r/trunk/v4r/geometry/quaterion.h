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

#ifndef V4R_QUATERION_H
#define V4R_QUATERION_H

#include <opencv/cv.h>

namespace V4R {

#ifndef V4R_QUATERION
#define V4R_QUATERION
template <typename T>
class Quaterion  : public cv::Vec<T,4> {
public:
    Quaterion() {};
    Quaterion(const cv::Vec<T,4> &r) : cv::Vec<T,4>(r) {};
    Quaterion(const cv::Vec<T,3> &r, bool invert = false) {
      setRVec(r, invert);
    }
    Quaterion(const cv::Mat_<T> &mat, bool invert = false)  {
      if(set(mat, invert) == 1) clear(); 
    }
    void clear(){ 
        this->val[0] = 0;
        this->val[1] = 0;
        this->val[2] = 0;
        this->val[3] = 0;
    }
    int setQuat(const cv::Mat_<T> &qvec) {
        if (((qvec.cols != 4) || (qvec.rows !=1)) && ((qvec.cols != 1) || (qvec.rows != 4))) return 1;
        this->val[0] = qvec(0);
        this->val[1] = qvec(1);
        this->val[2] = qvec(2);
        this->val[3] = qvec(3);
	return 0;
    }
    int setRVec(const cv::Vec<T,3> &rvec, bool invert = false) {
        cv::Mat_<T> r(3,1, rvec.val);
        return setRVec(r, invert);
    }
    int setRVec(const cv::Mat_<T> &rvec, bool invert = false) {
        if (((rvec.cols != 3) || (rvec.rows !=1)) && ((rvec.cols != 1) || (rvec.rows != 3))) return 1;
        cv::Mat_<T> R(3,3);
        cv::Rodrigues(rvec, R);
        return setRMat(R, invert);
    };
    int setRMat(const cv::Mat_<T> &R, bool invert = false) {
        if ((R.cols < 3) || (R.rows < 3)) return 1;
        T w = R(0,0) + R(1,1)+ R(2,2) + 1;
        if ( w < 0.0 ) return 1;
        w = sqrt( w );
        if (invert == false) {
            this->val[0] = (R(2,1) - R(1,2)) / (w*2.0);
            this->val[1] = (R(0,2) - R(2,0)) / (w*2.0);
            this->val[2] = (R(1,0) - R(0,1)) / (w*2.0);
        } else {
            this->val[0] = (R(1,2) - R(2,1)) / (w*2.0);
            this->val[1] = (R(2,0) - R(0,2)) / (w*2.0);
            this->val[2] = (R(0,1) - R(1,0)) / (w*2.0);
        }
        this->val[3] = w / 2.0;
        return 0;
    };
    int set(const cv::Mat_<T> &mat, bool invert = false)  {
        if (setQuat(mat) == 0) return 0;
        if (setRVec(mat, invert) == 0) return 0;
        if (setRMat(mat, invert) == 0) return 0;
	return 1;
    };
};

typedef Quaterion<double> QuaterionD;
#endif //V4R_QUATERION
};
#endif // V4R_QUATERION_H

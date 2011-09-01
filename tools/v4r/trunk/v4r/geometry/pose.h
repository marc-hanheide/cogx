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

#ifndef V4R_POSE_H
#define V4R_POSE_H

#include <opencv/cv.h>
#include <iomanip>

namespace V4R {

#ifndef V4R_POSE
#define V4R_POSE

/**
 * Pose
 * @author Markus Bader
 **/
template <typename T>
class Pose {
private:
    cv::Mat_<T> rvec_;
    cv::Mat_<T> tvec_;
    cv::Mat_<T> RT_;
public:
    /// constructor
    Pose() : rvec_(3,1), tvec_(3,1), RT_(4,4) {
      RT_ = cv::Mat_<T>::eye(4,4);
    };
    /**
     * copy constructor
     * @param pose
     **/
    Pose(const Pose &pose) :  rvec_(pose.rvec_), tvec_(pose.tvec_), RT_(pose.RT_) {}
    /**
     * copy constructor
     * @param pose = x, y, z, wx, wy, wz
     * @see updateMatRT
     **/
    Pose(const cv::Vec<T,6> &pose) : rvec_(3,1), tvec_(3,1), RT_(4,4) {
      RT_ = cv::Mat_<T>::eye(4,4);
        tvec_(0,0) = pose[0], tvec_(1,0) = pose[1], tvec_(2,0) = pose[2];
        rvec_(0,0) = pose[3], rvec_(1,0) = pose[4], rvec_(2,0) = pose[5];
    }
    /**
     * constructor updateMatRT is not called
     * @param rvec
     * @param tvec
     * @see updateMatRT
     **/
    Pose(const cv::Mat_<T> &rvec, const cv::Mat_<T> &tvec) :  rvec_(rvec), tvec_(tvec) {}
    /**
    * constructor updateVecRT is not called
    * @param RT
    * @param update on true it will call updateVecRT
    * @see updateVecRT
    * @post updateVecRT
    **/
    Pose(const cv::Mat_<T> &RT) :  RT_(RT) {}
    /**
    * constructor updateVecRT is not called
    * @param RT
    * @param update on true it will call updateVecRT
    * @see updateVecRT
    * @post updateVecRT
    **/
    template <typename T2>
    Pose(const cv::Mat_<T2> &RT, bool update) :  RT_(4,4) {
      RT_ = cv::Mat_<T>::eye(4,4);
        for (int r = 0; (r < RT.rows) && (r < RT_.rows); r++)
            for (int c = 0; (c < RT.cols) && (c < RT_.cols); c++)
                RT_(r,c) = (T) RT(r,c);
        if ( update) updateVecRT();
    }
    /**
    * constructor updateMatRT is not called
    * @param rvec
    * @param tvec
    * @see updateMatRT
    * @post updateMatRT
    **/
    Pose(double *rvec, double *tvec): rvec_(3,1), tvec_(3,1) {
        setRVec(rvec), setTVec(tvec);
    }
    /**
    * constructor updateMatRT is not called
    * @param rvec
    * @param tvec
    * @see updateMatRT
    * @post updateMatRT
    **/
    Pose(float *rvec, float *tvec): rvec_(3,1), tvec_(3,1) {
        setRVec(rvec), setTVec(tvec);
    }
    /**
    * constructor updateMatRT is not called
    * @param rvec
    * @param tvec
    * @see updateMatRT
    * @post updateMatRT
    **/
    Pose(const cv::Vec<T,3> &rvec, const cv::Vec<T,3> &tvec): rvec_(3,1), tvec_(3,1) {
        setRVec(rvec), setTVec(tvec);
    }
    /**
     * @param rvec
     * @param tvec
     * @see updateMatRT
     * @post updateMatRT
     **/
    void set(const cv::Vec<T,3> &rvec, const cv::Vec<T,3> &tvec) {
        setRVec(rvec), setTVec(tvec);
    }
    /**
     * @param rvec
     * @see updateMatRT
     * @see setTVec
     * @post updateMatRT
     **/
    void setRVec(const cv::Vec<T,3> &rvec) {
        rvec_(0,0) = rvec[0], rvec_(1,0) = rvec[1],  rvec_(2,0) = rvec[2];
    }
    /**
     * @param tvec
     * @see updateMatRT
     * @see setRVec
     * @post updateMatRT
     **/
    void setTVec(const cv::Vec<T,3> &tvec) {
        tvec_(0,0) = tvec[0], tvec_(1,0) = tvec[1],  tvec_(2,0) = tvec[2];
    }
    /**
     * @param tvec
     * @see updateMatRT
     * @see setRVec
     * @post updateMatRT
     **/
    void setTVec(const cv::Point3_<T> &tvec) {
        tvec_(0,0) = tvec.x, tvec_(1,0) = tvec.y,  tvec_(2,0) = tvec.z;
    }
    /**
     * @param rvec
     * @see updateMatRT
     * @see setTVec
     * @post updateMatRT
     **/
    void setRVec(const double *rvec) {
        rvec_(0,0) = rvec[0], rvec_(1,0) = rvec[1],  rvec_(2,0) = rvec[2];
    }
    /**
     * @param tvec
     * @see updateMatRT
     * @see setRVec
     * @post updateMatRT
     **/
    void setTVec(const double *tvec) {
        tvec_(0,0) = tvec[0], tvec_(1,0) = tvec[1],  tvec_(2,0) = tvec[2];
    }
    /**
     * @param rvec
     * @see updateMatRT
     * @see setTVec
     * @post updateMatRT
     **/
    void setRVec(const float *rvec) {
        rvec_(0,0) = rvec[0], rvec_(1,0) = rvec[1],  rvec_(2,0) = rvec[2];
    }
    /**
     * @param tvec
     * @see updateMatRT
     * @see setRVec
     * @post updateMatRT
     **/
    void setTVec(const float *tvec) {
        tvec_(0,0) = tvec[0], tvec_(1,0) = tvec[1],  tvec_(2,0) = tvec[2];
    }
    /**
     * @param pose = x, y, z, wx, wy, wz
     * @see updateMatRT
     * @post updateMatRT
     **/
    void set(cv::Vec<T,6> &pose) {
        tvec_ = cv::Mat_<T>(3,1, &pose[0]);
        rvec_ = cv::Mat_<T>(3,1, &pose[3]);
    }
    /**
     * @param rvec
     * @param tvec
     * @see updateMatRT
     **/
    void set(cv::Mat_<T> &rvec, cv::Mat_<T> &tvec) {
        rvec_ = rvec;
        tvec_ = tvec;
    }
    /**
     * fills the allocated vectors with the data
     * @param rvec
     * @param tvec
     **/
    void copyTo(double *rvec, double *tvec) {
        rvec[0] = (double) rvec_(0,0), rvec[1] = (double) rvec_(1,0), rvec[2] = (double) rvec_(2,0);
        tvec[0] = (double) tvec_(0,0), tvec[1] = (double) tvec_(1,0), tvec[2] = (double) tvec_(2,0);
    }
    /**
     * fills the allocated vectors with the data
     * @param rvec
     * @param tvec
     **/
    void copyTo(float *rvec, float *tvec) {
        rvec[0] = (float) rvec_(0,0), rvec[1] = (float) rvec_(1,0), rvec[2] = (float) rvec_(2,0);
        tvec[0] = (float) tvec_(0,0), tvec[1] = (float) tvec_(1,0), tvec[2] = (float) tvec_(2,0);
    }
    /**
     * Converts the pose calls into an other template
     * @param pose
     **/
    template <typename T2>
    void copyTo(Pose<T2> &pose) {
        pose.clear();
        pose.wx() = (T2) wx(),  pose.wy() = (T2) wy(), pose.wz() = (T2) wz();
        pose.x() = (T2) x(),  pose.y() = (T2) y(), pose.z() = (T2) z();
    }
    /**
     * updates the internal 4x4 rotation translation matrix
     * @return reference to the intermal 4x4 matrix
     **/
    const cv::Mat &updateMatRT() {
        RT_ = cv::Mat_<T>::eye(4,4);
        RT_(0,3) = tvec_(0,0), RT_(1,3) = tvec_(1,0), RT_(2,3) = tvec_(2,0);
        cv::Mat R(RT_, cv::Rect(0, 0, 3, 3));
        cv::Rodrigues(rvec_, R);
        return RT_;
    }
    /**
     * updates the internal 3x1 rotation and 3x1 translation vector
     **/
    void updateVecRT() {
        tvec_.create(3,1);
        tvec_(0,0) = RT_(0,3), tvec_(1,0) = RT_(1,3), tvec_(2,0) = RT_(2,3);
        cv::Mat R(RT_, cv::Rect(0, 0, 3, 3));
        cv::Rodrigues(R, rvec_);
    }
    /**
     * @return rotation vector 3x1
     **/
    cv::Mat_<T> &rvec() {
        return rvec_;
    }
    /**
     * @return translation vector 3x1
     **/
    cv::Mat_<T> &tvec() {
        return tvec_;
    }
    /**
     * @return roation translation matrix 4x4
     **/
    cv::Mat_<T> &RT() {
        return RT_;
    }
    /**
     * retuns an inverted pose
     * @param updateRTSrc on true it updates the internal matrix for the invertion first
     * @param updateRTDes on true it updates the internal vectors of the result after words
     * @return pose
     **/
    Pose<T> inv(bool updateRTSrc = true, bool updateRTDes = true) {
        if (updateRTSrc) updateMatRT();
        Pose<T> poseInv(RT_.inv());
        if (updateRTDes) poseInv.updateVecRT();
        return poseInv;
    }
    /**
     * retuns the roation as quaterion
     * @param updateRT on true it updates the internal matrix for the computation first
     * @return quaterion 4x1
     **/
    cv::Mat_<T> quaterion(bool updateRT = true) {
        cv::Mat_<T> quat;
        quaterion(quat, updateRT);
        return quat;
    }
    /**
     * computes the roation as quaterion
     * @param quat destiantion matrix 4x1
     * @param updateRT on true it updates the internal matrix for the computation first
     * @return zero on sugsess
     **/
    int quaterion(cv::Mat_<T> &quat, bool updateRT = true) {
        quat.create(4,1);
        if (updateRT) updateMatRT();
        T w = RT_(0,0) + RT_(1,1)+ RT_(2,2) + 1;
        if ( w < 0.0 ) return 1;
        w = sqrt( w );
        quat(0) = (RT_(2,1) - RT_(1,2)) / (w*2.0);
        quat(1) = (RT_(0,2) - RT_(2,0)) / (w*2.0);
        quat(2) = (RT_(1,0) - RT_(0,1)) / (w*2.0);
        quat(3) = w / 2.0;
        return 0;
    }
    /**
     * writes a file in opencv yml format which includes rvec and tvec
     * optional you can name the translation source and destination
     * @param file
     * @param frameSrc
     * @param frameDes
     * @see Pose::read
     **/
    void write(const std::string &file, std::string frameSrc = "", std::string frameDes = "") {
        cv::FileStorage fs ( file, cv::FileStorage::WRITE );
        if (!frameSrc.empty() && !frameDes.empty()) {
            fs << "src" << frameSrc;
            fs << "des" << frameDes;
        }
        fs << "rvec" << rvec_;
        fs << "tvec" << tvec_;
    }
    /**
     * reads a file
     * @param file
     * @see Pose::write
     **/
    void read(const std::string &file) {
        cv::FileStorage fs ( file, cv::FileStorage::READ );
        fs["rvec"] >> rvec_;
        fs["tvec"] >> tvec_;
    }
    /**
     * reads a file
     * @param file
     * @param frameSrc
     * @param frameDes
     * @see Pose::write
     **/
    void read(const std::string &file, std::string &frameSrc, std::string &frameDes) {
        cv::FileStorage fs ( file, cv::FileStorage::READ );
        fs["src"] >> frameSrc;
        fs["des"] >> frameDes;
        fs["rvec"] >> rvec_;
        fs["tvec"] >> tvec_;
    }

    const T &x() const {
        return tvec_(0,0);
    }
    T &x() {
        return tvec_(0,0);
    }
    const T &y() const {
        return tvec_(1,0);
    }
    T &y() {
        return tvec_(1,0);
    }
    const T &z() const {
        return tvec_(2,0);
    }
    T &z() {
        return tvec_(2,0);
    }
    const T &wx() const {
        return rvec_(0,0);
    }
    T &wx() {
        return rvec_(0,0);
    }
    const T &wy() const {
        return rvec_(1,0);
    }
    T &wy() {
        return rvec_(1,0);
    }
    const T &wz() const {
        return rvec_(2,0);
    }
    T &wz() {
        return rvec_(2,0);
    }
    /**
     * produces a human readable string
     * @return string t = %-8.3f, %-8.3f, %-8.3f; r = %-5.4f, %-5.4f, %-5.4f;
     **/
    std::string human_readable() const {
        std::stringstream ss;
        ss << "t = [ ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << x() << ", ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << y() << ", ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << z() << "]; ";
        ss << "r= [ ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << wx() << ", ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << wy() << ", ";
        ss << std::setw(8) << std::showpos << std::setprecision (5) << wz() << "]; ";
        return ss.str();
    }
    /**
     * clears all data and reserves memory if needed
     **/
    void clear() {
        rvec_ = cv::Mat_<T>::zeros(3,1);
        tvec_ = cv::Mat_<T>::zeros(3,1);
        RT_ = cv::Mat_<T>::eye(4,4);;
    }
};

typedef Pose<double> PoseD;

#endif //V4R_POSE
};

template <typename T>
inline std::ostream& operator << ( std::ostream &os, const V4R::Pose<T> &r) {
    os << r.string_human_readable();
    return os;
};

#endif // V4R_POSE_H

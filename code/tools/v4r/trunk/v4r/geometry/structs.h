/**
 * @file structs.h
 * @author Markus Bader
 * @brief
 *
 * @see
 **/

#ifndef V4R_STRUCTS_H
#define V4R_STRUCTS_H

#include <v4r/cvextensions/print_cv.h>
#include <v4r/geometry/pose.h>
#include <boost/concept_check.hpp>

namespace AK {
template<typename T> struct Pose3D;
template<typename T, uint16_t TypeVar> class SharedMemHeader;
template<typename T, uint16_t TypeVar> class SharedMemVar;
template<typename T, uint16_t TypeVar> class SharedMemArray;
template<typename T, uint16_t TypeVar> class SharedMemVector;
template<typename KeyType, typename ValueType, uint16_t TypeVar> class SharedMemMap;
}

namespace V4R {

template <typename T>
inline void xchange(T &a, T &b) {
    T tmp = a;
    a = b;
    b = tmp;
}

#ifndef V4R_REGION2D
#define V4R_REGION2D
template <typename T>
class Region2D  : public cv::Vec<T,4> {
public:
    Region2D() {};
    Region2D(const Region2D<T> &r)  : cv::Vec<T,4>(r) {};
    template <typename T2>
    Region2D(T2 x1, T2 y1, T2 x2, T2 y2)
            : cv::Vec<T,4>(x1, y1, x2, y2) {}
    template <typename T2>
    Region2D(const cv::Point_<T2> &rP1, const cv::Point_<T2> &rP2)
            : cv::Vec<T,4>(rP1.x, rP1.y, rP2.x, rP2.y) {}
    cv::Point_<T> &p1() {
        return (cv::Point_<T> &) this->val[0];
    }
    const cv::Point_<T> &p1() const {
        return (cv::Point_<T> &) this->val[0];
    }
    cv::Point_<T> &p2() {
        return (cv::Point_<T> &) this->val[2];
    }
    const cv::Point_<T> &p2() const {
        return (cv::Point_<T> &) this->val[2];
    }
    T &x1() {
        return this->val[0];
    }
    const T &x1() const {
        return this->val[0];
    }
    T &y1() {
        return this->val[1];
    }
    const T &y1() const {
        return this->val[1];
    }
    T &x2() {
        return this->val[2];
    }
    const T &x2() const {
        return this->val[2];
    }
    T &y2() {
        return this->val[3];
    }
    const T &y2() const {
        return this->val[3];
    }
    void sort() {
        if (x1() > x2()) xchange(x1(), x2());
        if (y1() > y2()) xchange(y1(), y2());
    }
    /// @pre sort
    template <typename T2>
    bool contains(const cv::Point3_<T2> &p) {
        return ( (p.x >= x1()) && (p.x <= x2()) && (p.y >= y1()) && (p.y <= y2()));
    }
};
typedef Region2D<int> Region2Di;
typedef Region2D<float> Region2Df;
typedef Region2D<double> Region2Dd;
#endif //V4R_REGION2D

#ifndef V4R_REGION3D
#define V4R_REGION3D
template <typename T>
class Region3D  : public cv::Vec<T,6> {
public:
    Region3D() {};
    Region3D(const Region2D<T> &r)  : cv::Vec<T,6>(r) {};
    template <typename T2>
    Region3D(const cv::Point3_<T2> &rP1, const cv::Point3_<T2> &rP2)
            : cv::Vec<T,6>(rP1.x, rP1.y, rP1.z, rP2.x, rP2.y, rP2.z) {}
    template <typename T2>
    Region3D(T2 x1, T2 y1, T2 z1, T2 x2, T2 y2, T2 z2)
            : cv::Vec<T,6>(x1, y1, z1, x2, y2, z2) {}
    cv::Point3_<T> &p1() {
        return (cv::Point3_<T> &) this->val[0];
    }
    const cv::Point3_<T> &p1() const {
        return (cv::Point3_<T> &) this->val[0];
    }
    cv::Point3_<T> &p2() {
        return (cv::Point3_<T> &) this->val[3];
    }
    const cv::Point3_<T> &p2() const {
        return (cv::Point3_<T> &) this->val[3];
    }
    T &x1() {
        return this->val[0];
    }
    const T &x1() const {
        return this->val[0];
    }
    T &y1() {
        return this->val[1];
    }
    const T &y1() const {
        return this->val[1];
    }
    T &z1() {
        return this->val[2];
    }
    const T &z1() const {
        return this->val[2];
    }
    T &x2() {
        return this->val[3];
    }
    const T &x2() const {
        return this->val[3];
    }
    T &y2() {
        return this->val[4];
    }
    const T &y2() const {
        return this->val[4];
    }
    T &z2() {
        return this->val[5];
    }
    const T &z2() const {
        return this->val[5];
    }
    /// @brief sorts the boarders
    void sort() {
        if (x1() > x2()) xchange(x1(), x2());
        if (y1() > y2()) xchange(y1(), y2());
        if (z1() > z2()) xchange(z1(), z2());
    }
    /// @pre sort
    template <typename T2>
    bool contains(const cv::Point3_<T2> &p) {
        return ( (p.x >= x1()) && (p.x <= x2()) && (p.y >= y1()) && (p.y <= y2()) && (p.z >= z1()) && (p.z <= z2()));
    }
};
typedef Region3D<int> Region3Di;
typedef Region3D<float> Region3Df;
typedef Region3D<double> Region3Dd;
#endif //V4R_REGION3D


#ifndef V4R_LINE2D
#define V4R_LINE2D
template <typename T>
class Line2D : public cv::Vec<T,3> {
public:
    Line2D() {};
    Line2D(cv::Vec<T,3> &r, bool normalize = true)
            : cv::Vec<T,3>(r) {
        if (normalize) this->normalize();
    };
    Line2D(cv::Mat &r, bool normalize = true)
            : cv::Vec<T,3>(*r.ptr<T>(0), *r.ptr<T>(1), *r.ptr<T>(2))  {
        if (normalize) this->normalize();

    };
    template <typename T2>
    Line2D(const cv::Point_<T2> &pt1, const cv::Point_<T2> &pt2, bool normalize = true) {
        cv::Vec<T, 3> p1(pt1.x, pt1.y, 1);
        cv::Vec<T, 3> p2(pt2.x, pt2.y, 1);
        vec() = p1.cross(p2);
        if (normalize) this->normalize();
    }
    T &A() {
        return this->val[0];
    }
    const T &A() const {
        return this->val[0];
    }
    T &B() {
        return this->val[1];
    }
    const T &B() const {
        return this->val[1];
    }
    T &C() {
        return this->val[2];
    }
    const T &C() const {
        return this->val[2];
    }
    void normalize() {
        double r = sqrt(this->val[0]*this->val[0] + this->val[1]*this->val[1]);
        this->val[0] /= r, this->val[1] /= r, this->val[2] /= r;
    }
    /** @pre normalize */
    template <typename T2>
    T distanceToLine(const cv::Point_<T2> &p) {
        return this->val[0]*((T)p.x) + this->val[1]*((T)p.y) + this->val[2];
    }
    /** @pre normalize */
    template <typename T2>
    std::vector<T> distanceToLine(const std::vector <cv::Point_<T2> > &points) {
        std::vector<T> d(points.size());
        for (int i = 0; i < d.size(); i++)  d[i] = distanceToLine(points[i]);
        return d;
    }
    /** @pre normalize */
    template <typename T2>
    cv::Point_<T> nearestPointOnLine(const cv::Point_<T2> &p) {
        T d = distanceToLine(p);
        return cv::Point_<T>(p.x - d * A(), p.y - d * B());
    }
    cv::Point_<T> intersection( Line2D<T> &l) {
        cv::Vec<T,3> h = l.cross(*this);
        return cv::Point_<T>(h[0]/h[2],h[1]/h[2]);
    }
    cv::Vec<T,3> &vec() {
        return *this;
    }
    cv::Vec<T,2> normal() {
        return cv::Vec<T,2>(this->val[0], this->val[1]);
    }
    const cv::Vec<T,3> &vec() const {
        return *this;
    }
};
typedef Line2D<float> Line2Df;
typedef Line2D<double> Line2Dd;
#endif //V4R_LINE2D

#ifndef V4R_LINESEGMENT2D
#define V4R_LINESEGMENT2D
template <typename T>
class LineSegment2D : public cv::Vec<T,4> {
public:
    LineSegment2D() {};
    LineSegment2D(cv::Vec<T,4> &r) : cv::Vec<T,4>(r) {};
    template <typename T2>
    LineSegment2D(const cv::Point_<T2> &pt1, const cv::Point_<T2> &pt2) {
        p1() = pt1;
        p2() = pt2;
    };
    template <typename T2, typename T3>
    LineSegment2D(const cv::Vec<T2,3> &line, const cv::Rect_<T3> &rect) {
        x1() = rect.x;
        x2() = rect.x + rect.width-1;
        y1() = -(line.val[0] * x1() + line.val[2]) / line.val[1];
        y2() = -(line.val[0] * x2() + line.val[2]) / line.val[1];
    };
    template <typename T2, typename T3>
    LineSegment2D(const cv::Vec<T2,3> &line, const cv::Size_<T3> &size) {
        x1() = 0;
        x2() = size.width-1;
        y1() = -(line.val[2]) / line.val[1];
        y2() = -(line.val[0] * x2() + line.val[2]) / line.val[1];
    };
    T &x1() {
        return this->val[0];
    }
    const T &x1() const {
        return this->val[0];
    }
    T &y1() {
        return this->val[1];
    }
    const T &y1() const {
        return this->val[1];
    }
    T &x2() {
        return this->val[2];
    }
    const T &x2() const {
        return this->val[2];
    }
    T &y2() {
        return this->val[3];
    }
    const T &y2() const {
        return this->val[3];
    }
    cv::Point_<T> &p1() {
        return (cv::Point_<T> &) this->val[0];
    }
    const cv::Point_<T> &p1() const {
        return (cv::Point_<T> &) this->val[0];
    }
    cv::Point_<T> &p2() {
        return (cv::Point_<T> &) this->val[2];
    }
    const cv::Point_<T> &p2() const {
        return (cv::Point_<T> &) this->val[2];
    }
};
typedef LineSegment2D<float> LineSegment2Df;
typedef LineSegment2D<double> LineSegment2Dd;
#endif //V4R_LINESEGMENT2D

#ifndef V4R_POSE3D
#define V4R_POSE3D
template <typename T>
class Pose3D : public cv::Vec<T,6> {
public:
    ///constructor
    Pose3D<T>() {};
    ///constructor
    Pose3D<T>(const T& x, const T& y = 0, const T& z = 0, const T& wx = 0, const T& wy = 0, const T& wz = 0) {
        this->val[0] = x,  this->val[1] = y,  this->val[2] = z,  this->val[3] = wx,  this->val[4] = wy,  this->val[5] = wz;
    };
    ///constructor
    Pose3D<T>(const cv::Mat_<T> &rvec, const cv::Mat_<T> &tvec) {
        set(rvec, tvec);
    }
    ///constructor
    Pose3D<T>(const T *pRvec, const T *pTvec) {
        set(pRvec, pTvec);
    }
    Pose3D<T>(const Pose3D<T> &r) : cv::Vec<T,6>(r) {};
    cv::Vec<T,3> &translation() {
        return (cv::Vec<T,3> &) this->val[0];
    }
    const cv::Vec<T,3> &translation() const {
        return (cv::Vec<T,3> &) this->val[0];
    }
    cv::Vec<T,3> &rotation() {
        return (cv::Vec<T,3> &) this->val[3];
    }
    const cv::Vec<T,3> &rotation() const {
        return (cv::Vec<T,3> &) this->val[3];
    }
    /// location
    cv::Point3_<T> &location() {
        return (cv::Point3_<T> &) this->val[0];
    }
    /** returns a copy of the loaction
    * @param scale
    **/
    cv::Point3_<T> location(double scale) {
        return cv::Point3_<T>(this->val[0]* scale, this->val[1]*scale, this->val[2] *scale);
    }
    /// translation X
    T& tx() {
        return this->val[0];
    }
    const T& tx() const {
        return this->val[0];
    }
    /// translation Y
    T& ty() {
        return this->val[1];
    }
    const T& ty() const {
        return this->val[1];
    }
    /// translation Z
    T& tz() {
        return this->val[2];
    }
    const T& tz() const {
        return this->val[2];
    }
    /// rotation X
    T& wx() {
        return this->val[3];
    }
    const T& wx() const {
        return this->val[3];
    }
    /// rotation Y
    T& wy() {
        return this->val[4];
    }
    const T& wy() const {
        return this->val[4];
    }
    /// rotation Z
    T& wz() {
        return this->val[5];
    }
    const T& wz() const {
        return this->val[5];
    }
    /**
     * double pointer to the parameters
     * tx, ty, tz, wx, wy, wz
     * @param p
     */
    T *ptr() {
        return (T*) this;
    }
    /**
     * double pointer to the parameters
     * tx, ty, tz
     * @param p
     */
    T *ptrT() {
        return this->val;
    }
    /**
     * double pointer to the parameters
     * wx, wy, wz
     * @param p
     */
    T *ptrR() {
        return this->val+3;
    }
    /**
     * sets Translation and Rotation form array
     * @param pRvec = wx, wy, wz
     * @param pTvec = tx, ty, tz
     */
    void set(double *pRvec, double *pTvec) {
        this->val[0] = (T) *pTvec++;
        this->val[1] = (T) *pTvec++;
        this->val[2] = (T) *pTvec++;
        this->val[3] = (T) *pRvec++;
        this->val[4] = (T) *pRvec++;
        this->val[5] = (T) *pRvec++;
    }

    /**
     * sets Translation and Rotation form array
     * @param pRvec = wx, wy, wz
     * @param pTvec = tx, ty, tz
     */
    template <typename T2>
    void set(const cv::Vec<T2,3> &rvec, const cv::Vec<T2,3> &tvec) {
        this->val[0] = (T) tvec[0];
        this->val[1] = (T) tvec[1];
        this->val[2] = (T) tvec[2];
        this->val[3] = (T) rvec[0];
        this->val[4] = (T) rvec[1];
        this->val[5] = (T) rvec[2];
    }
    /**
     * sets Translation and Rotation form array
     * @param p = tx, ty, tz, wx, wy, wz
     */
    void set(double *p) {
        this->val[0] = (T) *p++;
        this->val[1] = (T) *p++;
        this->val[2] = (T) *p++;
        this->val[3] = (T) *p++;
        this->val[4] = (T) *p++;
        this->val[5] = (T) *p++;
    }
    /**
     * sets Translation and Rotation form array
     * @param p = tx, ty, tz, wx, wy, wz
     */
    void set(float *p) {
        this->val[0] = (T) *p++;
        this->val[1] = (T) *p++;
        this->val[2] = (T) *p++;
        this->val[3] = (T) *p++;
        this->val[4] = (T) *p++;
        this->val[5] = (T) *p++;
    }
    /**
     * sets Translation and Rotation form opencv Mat
     * @param rvec
     * @param tvec
     */
    void set(const cv::Mat_<T> &rvec, const cv::Mat_<T> &tvec) {
        setR(rvec);
        setT(tvec);
    }
    /**
     * sets Rotation
     * @param rvec
     */
    void setR(const cv::Mat_<T> &rvec) {
        this->val[3] = rvec(0,1), this->val[4] = rvec(0,1), this->val[5] = rvec(0,2);
    }
    /**
     * sets Rotation
     * @param wx
     * @param wy
     * @param wz
     */
    void setR(T wx, T wy, T wz) {
        this->val[3] = wx, this->val[4] = wy, this->val[5] = wz;
    }
    /**
     * sets Rotation
     * @param r with wx, wy, wz
     */
    void setR(const cv::Vec<T,3>  &r) {
        rotation() = r;
    }
    /**
     * sets Translation
     * @param tx
     * @param ty
     * @param tz
     */
    void setT(T tx, T ty, T tz) {
        this->val[0] = tx,  this->val[1] = ty,  this->val[2] = tz;
    }
    /**
     * sets Translation
     * @param tx
     * @param ty
     * @param tz
     * @param scale
     */
    void setT(T tx, T ty, T tz, double scale) {
        this->val[0] = tx * scale,  this->val[1] = ty * scale,  this->val[2] = tz * scale;
    }
    /**
     * sets Translation
     * @param tvec
     */
    void setT(const cv::Mat_<T> &tvec) {
        this->val[0] = tvec(0,0), this->val[1] = tvec(0,1), this->val[2] = tvec(0,2);
    }
    /**
     * sets Translation
     * @param t width tx, ty, tz
     * @param scale
     */
    void setT(const cv::Vec<T,3>  &t, double scale) {
        translation() = t*scale;
    }
    /**
     * sets Translation
     * @param t width tx, ty, tz
     */
    void setT(const cv::Vec<T,3>  &t) {
        translation() = t;
    }
    void mat4x4(cv::Mat_<T> &M, double scale = 1) {
        M = cv::Mat_<T>::eye(4,4);
        cv::Mat_<T> R = matR3x3();
        M(0,3) = tx() * scale, M(1,3) = ty() * scale, M(2,3) = tz() * scale;
        M(0,0) = R(0,0), M(0,1) = R(0,1), M(0,2) = R(0,2);
        M(1,0) = R(1,0), M(1,1) = R(1,1), M(1,2) = R(1,2);
        M(2,0) = R(2,0), M(2,1) = R(2,1), M(2,2) = R(2,2);
    }
    cv::Mat_<T> mat4x4(double scale = 1) {
        cv::Mat_<T> M;
        mat4x4(M, scale);
        return M;
    }
    /// mat translation
    cv::Mat_<T> matT() {
        return cv::Mat_<T>(3,1, ptrT());
    }
    /// mat translation
    cv::Mat_<T> matR() {
        return cv::Mat_<T>(3,1, ptrR());
    }
    /// mat translation
    cv::Mat_<T> matR3x3() {
        cv::Mat_<T> r = matR();
        cv::Mat_<T> R( 3, 3);
        cv::Rodrigues(r, R);
        return R;
    }
    void roationAxis(cv::Vec<T,3> &axis, T &phi) {
        T vecZ[] = {0, 0, 1};
        cv::Mat_<T> r = matR().clone();
        r(2,0) = 0;
        cv::Mat_<T> R( 3, 3);
        cv::Mat_<T> a( 3, 1, axis[0]);
        cv::Rodrigues(r, R);
        a = R * cv::Mat_<T>(3,1,vecZ);
        //axis = cv::Vec<T,3>(0,0,1);
        phi = wz();
    }
    bool operator != (const V4R::Pose3D<T> &r) {
        return ((tx() != r.tx()) || (ty() != r.ty()) || (tz() != r.tz()) || (wx() != r.wx()) || (wy() != r.wy()) || (wz() != r.wz()));
    }
    bool operator == (const V4R::Pose3D<T> &r) {
        return ((tx() == r.tx()) && (ty() == r.ty()) && (tz() == r.tz()) && (wx() == r.wx()) && (wy() == r.wy()) && (wz() == r.wz()));
    }

    /**
    * produces a human readable string
    * @return string t = %-8.3f, %-8.3f, %-8.3f; r = %-5.4f, %-5.4f, %-5.4f;
    **/
    std::string string_human_readable() const {
        char pText[0xFF];
        sprintf(pText, "t = %-8.3f, %-8.3f, %-8.3f; r = %-5.4f, %-5.4f, %-5.4f;",
                tx(), ty(), tz(), wx(), wy(), wz());
        return std::string(pText);
    }
    //friend std::ostream &operator<<(std::ostream &stream, const Pose3D<T> &r);
};
typedef Pose3D<float> Pose3Df;
typedef Pose3D<double> Pose3Dd;
#endif //V4R_POSE3D



#ifndef V4R_TCAMERAPARAMETER
#define V4R_TCAMERAPARAMETER
template <typename T>
class TCameraParameter : public cv::Vec<T,4> {
public:
    TCameraParameter() {};
    TCameraParameter(const cv::Vec<T,4> &r)
            : cv::Vec<T,4>(r) {}
    const T &fx() const {
        return this->val[0];
    }
    const T &fy() const {
        return this->val[1];
    }
    const T &cx() const {
        return this->val[2];
    }
    const T &cy() const {
        return this->val[3];
    }
    T &fx() {
        return this->val[0];
    }
    T &fy() {
        return this->val[1];
    }
    T &cx() {
        return this->val[2];
    }
    T &cy() {
        return this->val[3];
    }
    T* ptr() {
        return (T*) this;
    }
    void set(T fx, T fy, T cx, T cy) {
        this->val[0] = fx;
        this->val[1] = fy;
        this->val[2] = cx;
        this->val[3] = cy;
    }
    void set(const cv::Mat_<T> &distCoeff) {
        this->val[0] = distCoeff(0,0);
        this->val[1] = distCoeff(1,1);
        this->val[2] = distCoeff(0,2);
        this->val[3] = distCoeff(1,2);
    }
    cv::Mat_<T> mat4x1() {
        return cv::Mat_<T>(4,1, ptr());
    }
    cv::Mat_<T> mat3x3() {
        cv::Mat_<T> M = cv::Mat_<T>::eye(3,3);
        M(0,0) = this->val[0], M(1,1) = this->val[1], M(0,2) = this->val[2];
        M(1,2) = this->val[3];
        return M;
    }
    cv::Mat_<T> mat4x4() {
        cv::Mat_<T> M = cv::Mat_<T>::eye(4,4);
        M(0,0) = this->val[0], M(1,1) = this->val[1], M(0,2) = this->val[2];
        M(1,2) = this->val[3];
        return M;
    }
    bool operator != (const TCameraParameter<T> &r) {
        return (this->val[0] != r[0]) || (this->val[1] != r[1]) || (this->val[2] != r[2]) || (this->val[3] != r[3]);
    }
    bool operator == (const TCameraParameter<T> &r) {
        return  (this->val[0] == r[0]) && (this->val[1] == r[1]) && (this->val[2] == r[2]) && (this->val[3] == r[3]);
    }
    /**
     * produces a human readable string
     * @return string [ k = %-8.3f, %-8.3f, %-8.3f; p = %-8.3f, %-8.3f]
     **/
    std::string string_human_readable() const {
        char pText[0xFF];
        sprintf(pText, "[ f = %-8.3f, %-8.3f, c = %-5.4f, %-5.4f]",
                fx(), fy(), cx(), cy());
        return std::string(pText);
    }
    friend std::ostream& operator << ( std::ostream &os, const V4R::TCameraParameter<T> &r) {
        return os << r.string_human_readable();
    };
};
#endif //V4R_TCAMERAPARAMETER

#ifndef V4R_TCAMRADISTORTION
#define V4R_TCAMRADISTORTION
template <typename T>
class TCameraDistortion  : public cv::Vec<T,5> {
public:
    TCameraDistortion() {};
    TCameraDistortion(const cv::Vec<T,5> &r)
            : cv::Vec<T,5>(r) {};
    T &k1() {
        return this->val[0];
    }
    T &k2() {
        return this->val[1];
    }
    T &p1() {
        return this->val[2];
    }
    T &p2() {
        return this->val[3];
    }
    T &k3() {
        return this->val[4];
    }
    const T &k1() const {
        return this->val[0];
    }
    const T &k2() const {
        return this->val[1];
    }
    const T &p1() const {
        return this->val[2];
    }
    const T &p2() const {
        return this->val[3];
    }
    const T &k3() const {
        return this->val[4];
    }
    T &fx() {
        return this->val[0];
    }
    T* ptr() {
        return (T*) this;
    }
    void set(double *p, int size = 4) {
        this->val[0] = *p++;
        this->val[1] = *p++;
        this->val[2] = *p++;
        this->val[3] = *p++;
        if (size > 4) {
            this->val[4] = *p++;
        }
    }
    void set(T k1, T k2, T p1, T p2, T k3 = 0) {
        this->val[0] = k1;
        this->val[1] = k2;
        this->val[2] = p1;
        this->val[3] = p2;
        this->val[4] = k3;
    }
    void set(const cv::Mat_<T> &distCoeff) {
        this->val[0] = distCoeff(0,0);
        this->val[1] = distCoeff(0,1);
        this->val[2] = distCoeff(0,2);
        this->val[3] = distCoeff(0,3);
        if ((distCoeff.rows > 4) && (distCoeff.cols > 4)) {
            this->val[4] = distCoeff(0,4);
        }
    }
    cv::Mat_<T> mat() {
        if (k3() == 0) {
            return cv::Mat_<T>(4, 1, ptr());
        } else {
            return cv::Mat_<T>(5, 1, ptr());
        }
    }
    bool operator != (const TCameraDistortion<T> &r) {
        return (this->val[0] != r[0]) || (this->val[1] != r[1]) || (this->val[2] != r[2]) || (this->val[3] != r[3]) || (this->val[4] != r[4]);
    }
    bool operator == (const TCameraDistortion<T> &r) {
        return (this->val[0] == r[0]) && (this->val[1] == r[1]) && (this->val[2] == r[2]) && (this->val[3] == r[3]) && (this->val[4] == r[4]);
    }

    /**
     * produces a human readable string
     * @return string [ k = %-8.3f, %-8.3f, %-8.3f; p = %-8.3f, %-8.3f]
     **/
    std::string string_human_readable() const {
        char pText[0xFF];
        sprintf(pText, "[ k = %-8.3f, %-8.3f, %-8.3f; p = %-5.4f, %-5.4f]",
                k1(), k2(), k3(), p1(), p1());
        return std::string(pText);
    }

    friend std::ostream& operator << ( std::ostream &os, const V4R::TCameraDistortion<T> &r) {
        return os << r.string_human_readable();
    }
};
#endif //V4R_TCAMRADISTORTION


#ifndef V4R_TPLANE
#define V4R_TPLANE
template <typename T>
class Plane {
public:
    Plane()
            : p(0,0,0), n(0,0,0), eq(0,0,0) {}
    Plane(const Plane<T> &r)
            : p(r.p), n(r.n), eq(r.eq) {}
    cv::Vec3d p;
    cv::Vec3d n;
    cv::Vec3d eq;
    friend std::ostream& operator << ( std::ostream &os, const V4R::Plane<T> &r) {
        return os << "- p: " << r.p << "\n - n: "  << r.n << "\n - eq: " << r.eq;
    };

};
#endif //V4R_TPLANE

#ifndef V4R_TOBJECT
#define V4R_TOBJECT
template <typename T>
class TObject : public Pose3D<T> {
public:
    static const unsigned int SHAPE_NA = 0;
    static const unsigned int SHAPE_BOX = 1;
    static const unsigned int SHAPE_CYLINDER = 1;
    TObject() {
        init();
    };
    TObject(const TObject<T> &r)
            : Pose3D<T>(r)
            , mID(r.mID)
            , mShapeID(r.mShapeID)
            , mDimensions(r.mDimensions) {};
    TObject(const cv::Vec<T,6> &r)
            : Pose3D<T>(r) {};
    unsigned int& id() {
        return mID;
    }
    const unsigned int& id() const {
        return mID;
    }
    unsigned int& shape() {
        return mShapeID;
    }
    const unsigned int& shape() const {
        return mShapeID;
    }
    cv::Vec<T, 3>& dimensions() {
        return mDimensions;
    }
    const cv::Vec<T, 3>& dimensions() const {
        return mDimensions;
    }
    T& length() {
        return mDimensions[0];
    }
    const T& length() const {
        return mDimensions[0];
    }
    T& width() {
        return mDimensions[1];
    }
    const T& width() const {
        return mDimensions[1];
    }
    T& height() {
        return mDimensions[2];
    }
    const T& height() const {
        return mDimensions[2];
    }
    void init(const cv::Vec<T, 6> pose = Pose3D<T>(0.0), unsigned int id = 0, unsigned int shape = SHAPE_NA, const cv::Vec<T, 3> &dimensions  = cv::Vec<T, 3>(0,0,0)) {
        *((cv::Vec<T, 6>*) this) = pose;
        mID = id;
        mShapeID = shape;
        mDimensions  = dimensions;

    }
protected:
    unsigned int mID;
    unsigned int mShapeID;
    cv::Vec<T, 3> mDimensions;
};
#endif //V4R_TOBJECT

#ifndef V4R_TPARTICLE
#define V4R_TPARTICLE
template <typename T>
class TParticle : public T {
public:
    TParticle() {};
    TParticle(const T &r) : T(r) {};
    virtual void update() {
        std::cerr << "TParticle::update not implementet!\n";
    };
    T& state() {
        return (T&) this->val[0];
    }
protected:
};
#endif //V4R_TPARTICLE

};



template <typename T>
inline std::ostream& operator << ( std::ostream &os, const V4R::Pose3D<T> &r) {
    return os << r.translation() << " -- " << r.rotation();
};





#endif //V4R_STRUCTS_H

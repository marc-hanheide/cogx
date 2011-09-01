/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) <2010>  <Markus Bader>

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

#ifndef V4R_CAMERAGEOMETRYBASE_H
#define V4R_CAMERAGEOMETRYBASE_H

#include <opencv2/opencv.hpp>
// #include <v4r/print/print_cv.h>

namespace V4R
{

template <typename T>
class CameraGeometryBase
{
protected:
    cv::Size mImgSize;
    cv::Mat_<T> mCamPara; /// Intrinsic in 4x1 format fx, fy, cx, cy
    cv::Mat_<T> mDistCoeff;  /// Intrinsic in 5x1 format k1, k2, p1, p2, k3
    cv::Mat_<T> mRVec; /// Rotation in 3x1 format wx, wy, wz
    cv::Mat_<T> mTVec; /// Translation in 3x1 format x, y, z
public:

    /// constructor
    CameraGeometryBase()
            : mImgSize ( 0,0 )
            , mCamPara (  4,1 ) 
            , mDistCoeff (  5,1 ) 
            , mRVec (  3,1 )
            , mTVec (  3,1 ) {
	      clear();              
            }
    /// copy constructor
    CameraGeometryBase ( const CameraGeometryBase &r )
            : mImgSize ( r.mImgSize )
            , mCamPara ( r.mCamPara )
            , mDistCoeff ( r.mDistCoeff )
            , mRVec ( r.mRVec )
            , mTVec ( r.mTVec )
    {
    }
    /// clears all entries
    void clear()
    {
        mImgSize = cv::Size ( 0,0 );
        mCamPara = cv::Mat_<T>::zeros ( 4,1 );
        mDistCoeff = cv::Mat_<T>::zeros ( 5,1 );
        mRVec = cv::Mat_<T>::zeros ( 3,1 );
        mTVec = cv::Mat_<T>::zeros ( 3,1 );
    }
    /// returns ture if the Geometry has valid size and camera parameter
    bool valid()
    {
        return ( ( mImgSize.height > 0 ) && ( mImgSize.width > 0 )  && ( cv::norm(mCamPara) > 0 ));
    }
    /// returns image size
    cv::Size &imgSize()
    {
        return mImgSize;
    }
    /// returns image size
    const cv::Size &imgSize() const
    {
        return mImgSize;
    }
    /// returns image width
    const int &width() const
    {
        return mImgSize.width;
    }
    /// returns image height
    const int &height() const
    {
        return mImgSize.height;
    }
    /** sets image size
     * @param w width
     * @param h height
     **/
    void setImgSize ( int w, int h )
    {
        mImgSize.width = w;
        mImgSize.height = h;
    }
    /** sets image size
     * @param s size
     **/
    void setImgSize ( const cv::Size &s )
    {
        mImgSize = s;
    }
    /// returns image width
    int &width()
    {
        return mImgSize.width;
    }
    /// returns image height
    int &height()
    {
        return mImgSize.height;
    }
    /** sets camera parameter
     * @param fx
     * @param fy
     * @param cx
     * @param cy
     **/
    void setCameraParameter ( T fx, T fy, T cx, T cy )
    {
        fx() = fx;
        fy() = fy;
        cx() = cx;
        cy() = cy;
    }
    /** sets camera parameter
     * @param cp fx, fy, cx, cy
     **/
    template <typename T2> void setCameraParameter ( const cv::Vec<T2,4> &cp )
    {
        fx() = ( T ) cp[0];
        fy() = ( T ) cp[1];
        cx() = ( T ) cp[2];
        cy() = ( T ) cp[3];
    }
    /** sets camera parameter
     * @param m can be a 4x1, 1x4 or at least 3x3
     **/
    void setCameraParameter ( const cv::Mat &m )
    {
        if ( ( m.rows == 1 ) || ( m.cols == 1 ) )
        {
            if ( m.depth() == CV_32F )
            {
                fx() = ( T ) m.at<float> ( 0,0 );
                fy() = ( T ) m.at<float> ( 0,1 );
                cx() = ( T ) m.at<float> ( 0,2 );
                cy() = ( T ) m.at<float> ( 0,3 );
            }
            else if ( m.depth() == CV_64F )
            {
                fx() = ( T ) m.at<double> ( 0,0 );
                fy() = ( T ) m.at<double> ( 0,1 );
                cx() = ( T ) m.at<double> ( 0,2 );
                cy() = ( T ) m.at<double> ( 0,3 );
            }
            else
            {
                CV_Error ( CV_StsUnsupportedFormat, "setCameraParameter" );
            }
        }
        else
        {
            if ( ( m.rows < 3 ) && ( m.cols < 3 ) ) CV_Error ( CV_StsUnsupportedFormat, "setCameraParameter" );
            if ( m.depth() == CV_32F )
            {
                fx() = ( T ) m.at<float> ( 0,0 );
                fy() = ( T ) m.at<float> ( 1,1 );
                cx() = ( T ) m.at<float> ( 0,2 );
                cy() = ( T ) m.at<float> ( 1,2 );
            }
            else if ( m.depth() == CV_64F )
            {
                fx() = ( T ) m.at<double> ( 0,0 );
                cy() = ( T ) m.at<double> ( 1,1 );
                cx() = ( T ) m.at<double> ( 0,2 );
                cy() = ( T ) m.at<double> ( 1,2 );
            }
            else
            {
                CV_Error ( CV_StsUnsupportedFormat, "setCameraParameter" );
            }
        }
    }
    /** sets camera parameter
     * @param m can be a 4x1, 1x4 or at least 3x3
     **/
    void setCameraParameter ( const cv::Mat_<T> &m )
    {
        mCamPara.create ( 4,1 );
        if ( ( m.rows == 1 ) || ( m.cols == 1 ) )
        {
            fx() = m ( 0,0 );
            fy() = m ( 1,0 );
            cx() = m ( 2,0 );
            cy() = m ( 3,0 );
        }
        else
        {
            if ( ( m.rows < 3 ) && ( m.cols < 3 ) ) CV_Error ( CV_StsUnsupportedFormat, "setCameraParameter" );
            fx() = m ( 0,0 );
            fy() = m ( 1,1 );
            cx() = m ( 0,2 );
            cy() = m ( 1,2 );
        }
    }
    /** computes a camera matrix 3x3 based on the camera parameter vector 4x1
     * @param m matrix to fill with the 3x3 camera matrix
     **/
    void computeCameraMatrix3x3 ( cv::Mat_<T> &m )
    {
        m.create ( 3,3 );
        m = cv::Mat_<T>::eye ( 3,3 );
        m ( 0,0 ) = fx(), m ( 1,1 ) = fy(),  m ( 0,2 ) = cx(),  m ( 1,2 ) = cy();
    }
    /** computes a camera matrix 4x4 based on the camera parameter vector 4x1
     * @param m matrix to fill with the 4x4 camera matrix
     **/
    void computeCameraMatrix4x4 ( cv::Mat_<T> &m )
    {
        m.create ( 4,4 );
        m = cv::Mat_<T>::eye ( 4,4 );
        m ( 0,0 ) = fx(), m ( 1,1 ) = fy(),  m ( 0,2 ) = cx(),  m ( 1,2 ) = cy();
    }
    /** returns camera parameter vector 
     * @return camera parameter vector 4x1 with fx, fy, cx, cy
     **/
    cv::Mat_<T> &cameraParameter()
    {
        return mCamPara;
    }
    /** returns camera parameter vector 
     * @return camera parameter vector 4x1 with fx, fy, cx, cy
     **/
    const cv::Mat_<T> &cameraParameter() const
    {
        return mCamPara;
    }
    /// @return focal length fx
    T &fx()
    {
        return mCamPara ( 0,0 );
    }
    /// @return focal length fy
    T &fy()
    {
        return mCamPara ( 1,0 );
    }
    /// @return camera center cx
    T &cx()
    {
        return mCamPara ( 2,0 );
    }
    /// @return camera center cy
    T &cy()
    {
        return mCamPara ( 3,0 );
    }
    /// @return focal length fx
    const T &fx() const
    {
        return mCamPara ( 0,0 );
    }
    /// @return focal length fx
    const T &fy() const
    {
        return mCamPara ( 1,0 );
    }
    /// @return camera center cx
    const T &cx() const
    {
        return mCamPara ( 2,0 );
    }
    /// @return camera center cy
    const T &cy() const
    {
        return mCamPara ( 3,0 );
    }
    void setDistortionCoefficient ( T k1, T k2, T p1, T p2, T k3 = 0 )
    {
        k1() = k1, k2() = k2, p1() = p1, p2() = p2, k3() = k3;
    }
    /// @param dc k1, k2, p1, p2, k3
    /// @param _k3 on ture it uses all five entries
    void setDistortionCoefficient ( float *dc, bool _k3 = true )
    {
        k1() = ( T ) *dc++;
        k2() = ( T ) *dc++;
        p1() = ( T ) *dc++;
        p2() = ( T ) *dc++;
        if ( _k3 ) k3() = ( T ) *dc++;
    }
    /// @param dc k1, k2, p1, p2, k1
    /// @param _k3 on ture it uses all five entries
    void setDistortionCoefficient ( double *dc, bool _k3 = true )
    {
        k1() = ( T ) *dc++;
        k2() = ( T ) *dc++;
        p1() = ( T ) *dc++;
        p2() = ( T ) *dc++;
        if ( _k3 ) k3() = ( T ) *dc++;
    }
    /** sets distortion coefficient
     * @param dc k1, k2, p1, p2, k1
     **/
    template <typename T2> void setDistortionCoefficient ( const cv::Vec<T2,5> &dc )
    {
        k1() = ( T ) dc[0], k2() = ( T ) dc[1], p1() = ( T ) dc[2], p2() = ( T ) dc[3], k3() = ( T ) dc[4];
    }
    /** sets distortion coefficient
     * @param dc k1, k2, p1, p2, k1
     **/
    template <typename T2> void setDistortionCoefficient ( const cv::Vec<T2,4> &dc )
    {
        k1() = ( T ) dc[0], k2() = ( T ) dc[1], p1() = ( T ) dc[2], p2() = ( T ) dc[3], k3() = 0.0;
    }
    /** sets distortion coefficient
     * @param m can be a 4x1, 1x4, 5x1, 1x5 [k1, k2, p1, p2, k1]
     **/
    void setDistortionCoefficient ( const cv::Mat &m )
    {
        if ( m.rows == 1 )
        {
            if ( m.cols < 4 ) CV_Error ( CV_StsUnsupportedFormat, "setDistortionCoefficient" );
            if ( m.depth() == CV_32F )
            {
                k1() = ( T ) m.at<float> ( 0,0 );
                k2() = ( T ) m.at<float> ( 0,1 );
                p1() = ( T ) m.at<float> ( 0,2 );
                p2() = ( T ) m.at<float> ( 0,3 );
                if ( m.cols > 4 ) k3() = ( T ) m.at<float> ( 0,4 );
                else k3() = 0.0;
            }
            else if ( m.depth() == CV_64F )
            {
                k1() = ( T ) m.at<double> ( 0,0 );
                k2() = ( T ) m.at<double> ( 0,1 );
                p1() = ( T ) m.at<double> ( 0,2 );
                p2() = ( T ) m.at<double> ( 0,3 );
                if ( m.cols > 4 ) k3() = ( T ) m.at<double> ( 0,4 );
                else k3() = 0.0;
            }
            else
            {
                CV_Error ( CV_StsUnsupportedFormat, "setDistortionCoefficient" );
            }
        }
        else if ( m.cols == 1 )
        {
            if ( m.rows < 4 ) CV_Error ( CV_StsUnsupportedFormat, "setDistortionCoefficient" );
            if ( m.depth() == CV_32F )
            {
                k1() = ( T ) m.at<float> ( 0,0 );
                k2() = ( T ) m.at<float> ( 1,0 );
                p1() = ( T ) m.at<float> ( 2,0 );
                p2() = ( T ) m.at<float> ( 3,0 );
                if ( m.cols > 4 ) k3() = ( T ) m.at<float> ( 4,0 );
                else k3() = ( T ) 0.0;
            }
            else if ( m.depth() == CV_64F )
            {
                k1() = ( T ) m.at<double> ( 0,0 );
                k2() = ( T ) m.at<double> ( 1,0 );
                p1() = ( T ) m.at<double> ( 2,0 );
                p2() = ( T ) m.at<double> ( 3,0 );
                if ( m.cols > 4 ) k3() = ( T ) m.at<double> ( 4,0 );
                else k3() = ( T ) 0.0;
            }
            else
            {
                CV_Error ( CV_StsUnsupportedFormat, "setDistortionCoefficient" );
            }
        }
        else
        {
            CV_Error ( CV_StsUnsupportedFormat, "setDistortionCoefficient" );
        }
    }
    /** sets distortion coefficient
     * @param m can be a 4x1, 1x4, 5x1, 1x5 [k1, k2, p1, p2, k1]
     **/
    void setDistortionCoefficient ( const cv::Mat_<T> &m )
    {
        if ( m.rows == 1 )
        {
            if ( m.cols < 4 ) CV_Error ( CV_StsUnsupportedFormat, "setDistortionCoefficient" );
            k1() = m ( 0,0 );
            k2() = m ( 0,1 );
            p1() = m ( 0,2 );
            p2() = m ( 0,3 );
            if ( m.cols > 4 ) k3() = ( T ) m ( 0,4 );
            else k3() = 0.0;
        }
        else if ( m.cols == 1 )
        {
            if ( m.rows < 4 ) CV_Error ( CV_StsUnsupportedFormat, "setDistortionCoefficient" );
            k1() = m ( 0,0 );
            k2() = m ( 1,0 );
            p1() = m ( 2,0 );
            p2() = m ( 3,0 );
            if ( m.cols > 4 ) k3() = m ( 4,0 );
            else k3() = 0.0;
        }
        else
        {
            CV_Error ( CV_StsUnsupportedFormat, "setDistortionCoefficient" );
        }
    }
    /// @return distortion coefficient k1, k2, p1, p2, k1
    cv::Mat_<T>  &distortionCoefficient()
    {
        return mDistCoeff;
    }
    /// @return distortion coefficient k1, k2, p1, p2, k1
    const cv::Mat_<T>  &distortionCoefficient() const
    {
        return mDistCoeff;
    }
    /// @return distortion coefficient  k1
    T &k1()
    {
        return mDistCoeff ( 0,0 );
    }
    /// @return distortion coefficient  k2
    T &k2()
    {
        return mDistCoeff ( 1,0 );
    }
    /// @return distortion coefficient  p1
    T &p1()
    {
        return mDistCoeff ( 2,0 );
    }
    /// @return distortion coefficient  p2
    T &p2()
    {
        return mDistCoeff ( 3,0 );
    }
    /// @return distortion coefficient  k3
    T &k3()
    {
        return mDistCoeff ( 4,0 );
    }
    /// @return distortion coefficient  k1
    const T &k1() const
    {
        return mDistCoeff ( 0,0 );
    }
    /// @return distortion coefficient  k2
    const T &k2() const
    {
        return mDistCoeff ( 1,0 );
    }
    /// @return distortion coefficient  p1
    const T &p1() const
    {
        return mDistCoeff ( 2,0 );
    }
    /// @return distortion coefficient  p2
    const T &p2() const
    {
        return mDistCoeff ( 3,0 );
    }
    /// @return distortion coefficient  k3
    const T &k3() const
    {
        return mDistCoeff ( 4,0 );
    }
    /// @return translation vector 3x1
    cv::Mat_<T>  &translationVector()
    {
        return mTVec;
    }
    /// @return translation vector 3x1
    const cv::Mat_<T> &translationVector() const
    {
        return mTVec;
    }
    void setTanslationVector ( T x, T y, T z )
    {
        tx() = x;
        ty() = y;
        tz() = z;
    }
    /// @param t x, y, z
    void setTanslationVector ( double *t )
    {
        tx() = *t++;
        ty() = *t++;
        tz() = *t++;
    }
    /// @param t x, y, z
    void setTanslationVector ( float *t )
    {
        tx() = *t++;
        ty() = *t++;
        tz() = *t++;
    }
    /// @param t x, y, z
    template <typename T2> void setTanslationVector ( const cv::Vec<T2,3> &t )
    {
        tx() = ( T ) t[0];
        ty() = ( T ) t[1];
        tz() = ( T ) t[2];
    }
    /// @param t x, y, z
    void setTanslationVector ( const cv::Mat &m )
    {

        if ( m.rows == 1 )
        {
            if ( m.cols < 4 ) CV_Error ( CV_StsUnsupportedFormat, "setTanslationVector" );
            if ( m.depth() == CV_32F )
            {
                tx() = ( T ) m.at<float> ( 0,0 );
                ty() = ( T ) m.at<float> ( 0,1 );
                tz() = ( T ) m.at<float> ( 0,2 );
            }
            else if ( m.depth() == CV_64F )
            {
                tx() = ( T ) m.at<double> ( 0,0 );
                ty() = ( T ) m.at<double> ( 0,1 );
                tz() = ( T ) m.at<double> ( 0,2 );
            }
            else
            {
                CV_Error ( CV_StsUnsupportedFormat, "setTanslationVector" );
            }
        }
        else if ( m.cols == 1 )
        {
            if ( m.rows < 4 ) CV_Error ( CV_StsUnsupportedFormat, "setTanslationVector" );
            if ( m.depth() == CV_32F )
            {
                tx() = ( T ) m.at<float> ( 0,0 );
                ty() = ( T ) m.at<float> ( 1,0 );
                tz() = ( T ) m.at<float> ( 2,0 );
            }
            else if ( m.depth() == CV_64F )
            {
                tx() = ( T ) m.at<double> ( 0,0 );
                ty() = ( T ) m.at<double> ( 1,0 );
                tz() = ( T ) m.at<double> ( 2,0 );
            }
            else
            {
                CV_Error ( CV_StsUnsupportedFormat, "setTanslationVector" );
            }
        }
        else
        {
            CV_Error ( CV_StsUnsupportedFormat, "setTanslationVector" );
        }
    }
    /// @param t x, y, z
    void setTanslationVector ( const cv::Mat_<T> &m )
    {
        if ( m.rows == 1 )
        {
            if ( m.cols < 3 ) CV_Error ( CV_StsUnsupportedFormat, "setTanslationVector" );
            tx() = m ( 0,0 );
            ty() = m ( 0,1 );
            tz() = m ( 0,2 );
        }
        else if ( m.cols == 1 )
        {
            if ( m.rows < 3 ) CV_Error ( CV_StsUnsupportedFormat, "setTanslationVector" );
            tx() = m ( 0,0 );
            tz() = m ( 1,0 );
            ty() = m ( 2,0 );
        }
        else
        {
            CV_Error ( CV_StsUnsupportedFormat, "setTanslationVector" );
        }
    }
    cv::Mat_<T> &getTanslationVector ( cv::Mat_<T> &m )
    {
        m = cv::Mat_<T> ( 1,3, &mTVec[0] );
        return m;
    }
    T& tx()
    {
        return mTVec ( 0,0 );
    }
    T& ty()
    {
        return mTVec ( 1,0 );
    }
    T& tz()
    {
        return mTVec ( 2,0 );
    }
    const T& tx() const
    {
        return mTVec ( 0,0 );
    }
    const T& ty() const
    {
        return mTVec ( 1,0 );
    }
    const T& tz() const
    {
        return mTVec ( 2,0 );
    }
    void setRotationVector ( T wx, T wy, T wz )
    {
        wx() = wx,  wy() = wy, wz() = wz;
    }
    /// @param t wx, wy, wz
    void setRotationVector ( double *t )
    {
        wx() = *t++;
        wy() = *t++;
        wz() = *t++;
    }
    /// @param t wx, wy, wz
    void setRotationVector ( float *t )
    {
        wx() = *t++;
        wy() = *t++;
        wz() = *t++;
    }
    /// @param t wx, wy, wz
    template <typename T2> void setRotationVector ( const cv::Vec<T2,3> &t )
    {
        wx() = ( T ) t[0], wy() = ( T ) t[1], wz() = ( T ) t[2];
    }
    /// @param t wx, wy, wz
    void setRotationVector ( const cv::Mat &m )
    {

        if ( m.rows == 1 )
        {
            if ( m.cols < 4 ) CV_Error ( CV_StsUnsupportedFormat, "setRotationVector" );
            if ( m.depth() == CV_32F )
            {
                wx() = ( T ) m.at<float> ( 0,0 );
                wy() = ( T ) m.at<float> ( 0,1 );
                wz() = ( T ) m.at<float> ( 0,2 );
            }
            else if ( m.depth() == CV_64F )
            {
                wx() = ( T ) m.at<double> ( 0,0 );
                wy() = ( T ) m.at<double> ( 0,1 );
                wz() = ( T ) m.at<double> ( 0,2 );
            }
            else
            {
                CV_Error ( CV_StsUnsupportedFormat, "setRotationVector" );
            }
        }
        else if ( m.cols == 1 )
        {
            if ( m.rows < 4 ) CV_Error ( CV_StsUnsupportedFormat, "setRotationVector" );
            if ( m.depth() == CV_32F )
            {
                wx() = ( T ) m.at<float> ( 0,0 );
                wy() = ( T ) m.at<float> ( 1,0 );
                wz() = ( T ) m.at<float> ( 2,0 );
            }
            else if ( m.depth() == CV_64F )
            {
                wx() = ( T ) m.at<double> ( 0,0 );
                wy() = ( T ) m.at<double> ( 1,0 );
                wz() = ( T ) m.at<double> ( 2,0 );
            }
            else
            {
                CV_Error ( CV_StsUnsupportedFormat, "setRotationVector" );
            }
        }
        else
        {
            CV_Error ( CV_StsUnsupportedFormat, "setRotationVector" );
        }
    }
    /// @param t wx, wy, wz
    void setRotationVector ( const cv::Mat_<T> &m )
    {
        if ( m.rows == 1 )
        {
            if ( m.cols < 3 ) CV_Error ( CV_StsUnsupportedFormat, "setRotationVector" );
            wx() = m ( 0,0 );
            wy() = m ( 0,1 );
            wz() = m ( 0,2 );
        }
        else if ( m.cols == 1 )
        {
            if ( m.rows < 3 ) CV_Error ( CV_StsUnsupportedFormat, "setRotationVector" );
            wx() = m ( 0,0 );
            wy() = m ( 1,0 );
            wz() = m ( 2,0 );
        }
        else
        {
            CV_Error ( CV_StsUnsupportedFormat, "setRotationVector" );
        }
    }
    /// @param R must be at least 3x3
    void setRoationVectorFromMatrix ( cv::Mat &M )
    {
        cv::Mat_<T> RT;
        M.convertTo ( RT, RT.type() );
        cv::Mat_<T> R ( RT, cv::Rect ( 0, 0, 3, 3 ) );
        cv::Rodrigues ( R, mRVec );
    }
    cv::Mat_<T> computeRoationMatrix ( cv::Mat_<T> &m )
    {
        getRoationVector ( mRVec );
        cv::Rodrigues ( mRVec, m );
        return m;
    }
    cv::Mat_<T>  &rotationVector()
    {
        return mRVec;
    }
    const cv::Mat_<T>  &rotationVector() const
    {
        return mRVec;
    }
    T& wx()
    {
        return mRVec ( 0,0 );
    }
    T& wy()
    {
        return mRVec ( 1,0 );
    }
    T& wz()
    {
        return mRVec ( 2,0 );
    }
    const T& wx() const
    {
        return mRVec ( 0,0 );
    }
    const T& wy() const
    {
        return mRVec ( 1,0 );
    }
    const T& wz() const
    {
        return mRVec ( 2,0 );
    }
    void computePoseMatrix4x4 ( cv::Mat_<T> &RT )
    {
        RT.create ( 4,4 );
        RT = cv::Mat_<T>::eye ( 4,4 );
        RT ( 0,3 ) = tx(), RT ( 1,3 ) = ty(), RT ( 2,3 ) = tz();
        cv::Mat_<T> R ( RT, cv::Rect ( 0, 0, 3, 3 ) );
        cv::Rodrigues ( mRVec, R );
    }
    bool operator != ( const CameraGeometryBase<T> &r )
    {
        return ( mImgSize != r.mImgSize ) || ( mCamPara != r.mCamPara ) || ( mDistCoeff != r.mDistCoeff ) || ( mTVec != r.mTVec ) || ( mRVec != r.mRVec );
    }
    bool operator == ( const CameraGeometryBase<T> &r )
    {
        return ( mImgSize == r.mImgSize ) && ( mCamPara == r.mCamPara ) && ( mDistCoeff == r.mDistCoeff ) && ( mTVec == r.mTVec ) && ( mRVec == r.mRVec );
    }
    /**
     * produces a human readable string of the camera pose
     * @return string
     **/
    std::string human_readable_pose() const
    {
        char pText[0xFF];
        sprintf ( pText, "t = [%-8.3f, %-8.3f, %-8.3f]; w = [%-8.3f, %-8.3f, %-8.3f]; %p", tx(), ty(), tz(), wx(), wy(), wz(), mTVec.data );
        return std::string ( pText );
    }
    /**
     * produces a human readable string of the camera distorions
     * @return string
     **/
    std::string human_readable_distortions() const
    {
        char pText[0xFF];
        sprintf ( pText, "k = [%-8.3f, %-8.3f, %-8.3f]; p = [%-8.3f, %-8.3f];", k1(), k2(), k3(), p1(), p1() );
        return std::string ( pText );
    }
    /**
     * produces a human readable string of the camera parameters
     * @return string
     **/
    std::string human_readable_camera() const
    {
        char pText[0xFF];
        sprintf ( pText, "s = [%4i,%4i]; f = [%-8.3f, %-8.3f]; c = [%-8.3f, %-8.3f];", width(), height(), fx(), fy(), cx(), cy() );
        return std::string ( pText );
    }
    /**
     * produces a human readable string of all entries
     * @return string
     **/
    std::string human_readable() const
    {
        std::stringstream ss;
        ss << human_readable_camera() << std::endl;
        ss << human_readable_distortions() << std::endl;
        ss << human_readable_pose() << std::endl;
        return ss.str();
    }

    bool isEqual ( const cv::Mat_<T> &A, const cv::Mat_<T> &B ) const
    {
        for ( int r = 0; r < A.rows; r++ )
            for ( int c = 0; c < A.cols; c++ )
                if ( A ( r,c ) != B ( r,c ) ) return false;
        return true;
    }
};
};
#endif // V4R_CAMERAGEOMETRYBASE_H

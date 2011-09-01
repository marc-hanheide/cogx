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

#ifndef V4R_CAMERAGEOMETRY_H
#define V4R_CAMERAGEOMETRY_H

#include <v4r/geometry/camera/camerageometrybase.h>

namespace cv {
/**
 * triangulatePoints function HZ, R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, Cambridge Univ. Press, 2003.
* @param projMatr1 Int*Ext  4x3 Matrix
* @param projMatr2 Int*Ext  4x3 Matrix
* @param projPoints1 image points 2xn Matrix
* @param projPoints2 image points 2xn Matrix
* @param points4D 3D points 4xn or 3xn if 4xn the 4th part will be the scale
**/
void triangulatePoints(const Mat &projMatr1, const Mat &projMatr2, const Mat &projPoints1, const Mat &projPoints2, Mat &pointsDes);
void triangulatePoint(const Mat_<double> &projMatr1, const Mat_<double> &projMatr2, const Point_<double> &pointA, const Point_<double>  &pointB, cv::Point3_<double> &pointDes);


};

namespace V4R {
cv::Mat_<double> convertPointsHomogeneous(const std::vector< cv::Point_<double> > &rPoints);

class CameraGeometry : public CameraGeometryBase<double>
{
public:
    /**
     * Constructor
    **/
    CameraGeometry();
    /**
     * Copy Constructor
     * @param r
    **/
    CameraGeometry(const CameraGeometryBase<double> &r);
    /**
     * Destructor
    **/
    ~CameraGeometry();
    /**
     * Updates the mInt4x1, mInt4x4, mInt3x3 from camPara
    **/
    void updateMatrixIntrinsic();
    /**
     * Updates the mExt, mInvExt, mIntExt from camPose
    	 * @pre updateMatrixIntrinsic
    **/
    void updateMatrixExtrinsic();
    /**
     * Updates the mUndistMap from camPara and distCoeff
    	 * @pre updateMatrixIntrinsic updateDistortionCoeff or updateDistortionMap
    **/
    void updateDistortionMap();
    /**
     * Updates the mInt4x1, mInt4x4, mInt3x3, mExt, mInvExt, mIntExt, mDistCoeff if something changed
     * @param bUpdateDistorMap on true it will update the mUndistMap as well
    **/
    void update(const CameraGeometryBase<double> &r, bool bUpdateDistorMap = false);
    /**
     * Undistors image
     * @param pImgSrc
     * @param pImgDes
    **/
    void undistor(IplImage *pImgSrc, IplImage *pImgDes);
    /**
     * Undistors single points using the opencv function undistortPoints
     * @param rPtnSrc
     * @param rPtnDes
    **/
    void undistortPoints(const std::vector < cv::Point_<double> > &rPtnSrc, std::vector < cv::Point_<double> > &rPtnDes);
    /**
    * Undistors single points using a pre computet lookup table
    * @param rPtnSrc
    * @param rPtnDes
    * @todo not yet finished
    **/
    void undistort(const std::vector < cv::Point_<double> > &rPtnSrc, std::vector < cv::Point_<double> > &rPtnDes);
    /**
     * Projects a 3D Point
     * @param rPtnSrc
     * @param rPtnDes
    	 **/
    void project(const cv::Point3_<double> &rPtnSrc, cv::Point_<double> &rPtnDes);
    /**
    * Projects a 3D Point
    * @param rPtnSrc
     * @return image point
     **/
    cv::Point_<double>  project(const cv::Point3_<double> &rPtnSrc);
    /**
     * Projects a 3D Point and distores the point if needed
     * @param rPtnSrc
     * @param rPtnDes
    	 **/
    void project(const std::vector < cv::Point3_<double> > &rPtnSrc, std::vector < cv::Point_<double> > &rPtnDes);

    /**
     * Computes extrinsic based on 3D world and 2D image points
     * @param imagePoints
     * @param worldPoints
    **/
    void computeExtrinsic (const std::vector<cv::Point2d> imagePoints, const std::vector<cv::Point3d> worldPoints, bool imgPointsAreDestort = true);

    /// Intrinsic in 4x1 format fx, fy, cx, cy
    const cv::Mat_<double> &MInt4x1() const {
        return mCamPara;
    };
    /// Intrinsic as 3x3 matrix
    const cv::Mat_<double> &MInt3x3() const {
        return mInt3x3;
    };
    /// Intrinsic as 4x4 matrix
    const cv::Mat_<double> &MInt4x4() const {
        return mInt4x4;
    };
    /// Extrinsic as 4x4 matrix
    const cv::Mat_<double> &MExt() const {
        return mExt;
    };
    /// Extrinsic as 3x4 matrix
    cv::Mat_<double> MExt3x4() const {
        return cv::Mat_<double>(mExt, cv::Range(0,3),cv::Range(0,4));
    };
    /// Inverse of Extrinsic as 4x4 matrix
    const cv::Mat_<double> &MInvExt() const {
        return mInvExt;
    };
    /// Intrinsic * Extrinsic as 4x4 matrix
    const cv::Mat_<double> &MIntExt() const {
        return mIntExt;
    };
    /// Intrinsic * Extrinsic as 3x4 matrix
    cv::Mat_<double> MIntExt3x4() const {
        return cv::Mat_<double>(mIntExt, cv::Range(0,3),cv::Range(0,4));
    };
    /// Distortion parameter k1, k2, p1, p2, [k3]
    const cv::Mat_<double> &MDistCoeff() const {
        return mDistCoeff;
    };
    
    
    /**
     * retuns the camera pose
     * @return pose of the camera in world cooridinates
     * @see updateMatrixExtrinsic
     **/
    cv::Point3_<double> cameraPose();
    /**
     * retuns the camera pose
     * @param tvec
     * @see updateMatrixExtrinsic
     **/
    void cameraPose(cv::Mat_<double> &tvec);
    
    /**
     * retuns the roation as quaterion
     * @param updateMatrixExtrinsic on true it updates the internal matrix for the computation first
     * @return quaterion 4x1
     **/
    cv::Mat_<double> quaterion(bool update = true);
    
    
    /**
     * computes the roation as quaterion
     * @param quat destiantion matrix 4x1
     * @param updateMatrixExtrinsic on true it updates the internal matrix for the computation first
     * @return zero on sugsess
     **/
    int quaterion(cv::Mat_<double> &quat, bool updateRT = true);
    
    /**
     * computes essential matrix
     **/
    static const cv::Mat_<double> computeEssential(const CameraGeometry &A, const CameraGeometry &B);
        
    /**
     * returns the 3D location of a image point on the image plane in world cooridinates
     * @param src image point
     * @param des 
     **/
    void point3DOnImagePlane(const cv::Point_<double> &src, cv::Point3_<double> &des);
    
    /**
     * triangulates point seen in two images
     * @param cgA 
     * @param cgB
     * @param pntA 
     * @param pntB 
     * @return point 
     **/
    static const cv::Point3_<double> triangulate(const CameraGeometry &cgA, const CameraGeometry &cgB, const cv::Point_<double> &pntA, const cv::Point_<double> &pntB);
 
    /**
     * valid2Project
     * @return true if all preprocessing steps are done to use the project functions 
     **/
    bool valid2Project();
private:
    cv::Mat_<double> mInt3x3;
    cv::Mat_<double> mInt4x4;
    cv::Mat_<double> mExt;
    cv::Mat_<double> mInvExt;
    cv::Mat_<double> mIntExt;
    cv::Mat_<double> mInvIntExt;
    cv::Mat mUndistMap[2];

};
};
#endif // V4R_CAMERAGEOMETRY_H

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

#include "camerageometry.h"
#include <v4r/cvextensions/print_cv.h>

using namespace V4R;

CameraGeometry::CameraGeometry() : CameraGeometryBase< double >()
{
}

CameraGeometry::CameraGeometry ( const CameraGeometryBase<double> &r )
        : CameraGeometryBase< double > ( r )
{
}

CameraGeometry::~CameraGeometry()
{
}


void CameraGeometry::update ( const CameraGeometryBase<double> &r, bool bUpdateDistorMap )
{
    if ( !isEqual ( mCamPara,r.cameraParameter() ) )
    {
        mCamPara = r.cameraParameter();
        updateMatrixIntrinsic();
    }
    if ( ( !isEqual ( mDistCoeff,r.distortionCoefficient() ) ) || ( mImgSize != r.imgSize() ) )
    {
        mDistCoeff = r.distortionCoefficient();
        mImgSize = r.imgSize();
        if ( bUpdateDistorMap )
        {
            updateDistortionMap();
        }
    }
    if ( ( !isEqual ( mRVec,r.rotationVector() ) ) || ( !isEqual ( mTVec, r.translationVector() ) ) )
    {
        mRVec = r.rotationVector();
        mTVec = r.translationVector();
        updateMatrixExtrinsic();
    }
}
void CameraGeometry::updateMatrixIntrinsic()
{
    computeCameraMatrix3x3 ( mInt3x3 );
    computeCameraMatrix4x4 ( mInt4x4 );
}
void CameraGeometry::updateMatrixExtrinsic()
{
    computePoseMatrix4x4 ( mExt );
    mIntExt = mInt4x4 * mExt;
    mInvExt = mExt.inv();
    mInvIntExt = mIntExt.inv();
}

cv::Mat_<double> CameraGeometry::quaterion ( bool update )
{
    cv::Mat_<double> quat;
    quaterion ( quat, update );
    return quat;
}

int CameraGeometry::quaterion ( cv::Mat_<double> &quat, bool update )
{
    quat.create ( 4,1 );
    if ( update ) computePoseMatrix4x4 ( mExt );
    double w = mExt ( 0,0 ) + mExt ( 1,1 ) + mExt ( 2,2 ) + 1;
    if ( w < 0.0 ) return 1;
    w = sqrt ( w );
    quat ( 0,0 ) = ( mExt ( 2,1 ) - mExt ( 1,2 ) ) / ( w*2.0 );
    quat ( 1,0 ) = ( mExt ( 0,2 ) - mExt ( 2,0 ) ) / ( w*2.0 );
    quat ( 2,0 ) = ( mExt ( 1,0 ) - mExt ( 0,1 ) ) / ( w*2.0 );
    quat ( 3,0 ) = w / 2.0;
    return 0;
}
void CameraGeometry::updateDistortionMap()
{
    cv::initUndistortRectifyMap ( mInt3x3, mDistCoeff, cv::Mat(), mInt3x3, mImgSize, CV_32FC1, mUndistMap[0], mUndistMap[1] );
}

void CameraGeometry::undistor ( IplImage *pImgSrc, IplImage *pImgDes )
{
    cv::Mat src ( pImgSrc );
    cv::Mat des ( pImgDes );
    cv::remap ( src, des, mUndistMap[0], mUndistMap[1], cv::INTER_NEAREST );
}

void CameraGeometry::undistort ( const std::vector < cv::Point_<double> > &rPtnSrc, std::vector < cv::Point_<double> > &rPtnDes )
{

}
cv::Point3_<double> CameraGeometry::cameraPose()
{
    return cv::Point3_<double> ( mInvExt ( 0,3 ), mInvExt ( 1,3 ), mInvExt ( 2,3 ) );
}

void CameraGeometry::cameraPose ( cv::Mat_<double> &tvec )
{
    tvec.create ( 3,1 );
    tvec ( 0,0 ) = mInvExt ( 0,3 );
    tvec ( 1,0 ) = mInvExt ( 1,3 );
    tvec ( 2,0 ) = mInvExt ( 2,3 );
}
void CameraGeometry::point3DOnImagePlane ( const cv::Point_<double> &src, cv::Point3_<double> &des )
{
    /// @ToDo undistor point first if needed
    double *p = ( double* ) mInvIntExt.data;
    des.x = p[0] * src.x + p[1] * src.y  + p[2]  + p[3];
    des.y = p[4] * src.x + p[5] * src.y  + p[6]  + p[7];
    des.z = p[8] * src.x + p[9] * src.y  + p[10] + p[11];
}

void CameraGeometry::undistortPoints ( const std::vector < cv::Point_<double> > &rPtnSrc, std::vector < cv::Point_<double> > &rPtnDes )
{
    rPtnDes.resize ( rPtnSrc.size() );
    cv::Mat_<double> src ( rPtnSrc.size(), 2, ( double* ) &rPtnSrc[0] );
    cv::Mat_<double> des ( rPtnDes.size(), 2, ( double* ) &rPtnDes[0] );
    cv::Mat_<double> F ( MInt3x3(), cv::Range ( 0,2 ), cv::Range ( 0,2 ) );
    cv::Mat_<double> C ( MInt3x3(), cv::Range ( 0,2 ), cv::Range ( 2,3 ) );
    //cv::undistortPoints(src, des, MInt3x3(), MDistCoeff());

    CvMat cameraMatrix = MInt3x3();
    CvMat distCoeffs = MDistCoeff();
    CvMat  srcP = cvMat ( rPtnSrc.size(),1, CV_64FC2, ( void* ) &rPtnSrc[0] );
    CvMat  desP = cvMat ( rPtnSrc.size(),1, CV_64FC2, ( void* ) &rPtnDes[0] );
    cvUndistortPoints ( &srcP, &desP, &cameraMatrix, &distCoeffs );
    for ( unsigned int i = 0; i < rPtnSrc.size(); i++ )
    {
        rPtnDes[i].x = rPtnDes[i].x * fx() +  cx(),   rPtnDes[i].y = rPtnDes[i].y *  fy() +  cy();
    }
}

bool CameraGeometry::valid2Project ( ){
  return !mIntExt.empty();
}

void CameraGeometry::project ( const cv::Point3_<double> &rPtnSrc, cv::Point_<double> &rPtnDes )
{
    double *p = ( double* ) mIntExt.data;
    double xc = p[0] * rPtnSrc.x + p[1] * rPtnSrc.y  + p[2] * rPtnSrc.z  + p[3];
    double yc = p[4] * rPtnSrc.x + p[5] * rPtnSrc.y  + p[6] * rPtnSrc.z  + p[7];
    double zc = p[8] * rPtnSrc.x + p[9] * rPtnSrc.y  + p[10] * rPtnSrc.z  + p[11];
    rPtnDes.x = xc/zc;
    rPtnDes.y = yc/zc;
}
cv::Point_<double> CameraGeometry::project ( const cv::Point3_<double> &rPtnSrc )
{
    cv::Point_<double> rPtnDes;
    project (rPtnSrc, rPtnDes);
    return rPtnDes;
}

void CameraGeometry::project ( const std::vector < cv::Point3_<double> > &rPtnSrc, std::vector < cv::Point_<double> > &rPtnDes )
{
    rPtnDes.resize ( rPtnSrc.size() );
    for ( unsigned int i = 0; i <rPtnSrc.size(); i++ )
    {
        project ( rPtnSrc[i], rPtnDes[i] );
    }
}

void CameraGeometry::computeExtrinsic ( const std::vector<cv::Point2d> imagePoints, const std::vector<cv::Point3d> worldPoints, bool imgPointsAreDestort )
{
    CvMat *pImagePoints = cvCreateMat ( imagePoints.size(), 2, CV_64F );
    CvMat *pWorldPoints = cvCreateMat ( worldPoints.size(), 3, CV_64F );
    for ( unsigned int i = 0; i < imagePoints.size(); i++ )
    {
        pImagePoints->data.db[i*2] = imagePoints[i].x, pImagePoints->data.db[i*2+1] = imagePoints[i].y;
        pWorldPoints->data.db[i*3] = worldPoints[i].x, pWorldPoints->data.db[i*3+1] = worldPoints[i].y, pWorldPoints->data.db[i*3+2] = worldPoints[i].z;
    }
    CvMat tRotVec = cvMat ( 3, 1, CV_64F, &mRVec );
    CvMat tTraVec = cvMat ( 3, 1, CV_64F, &mTVec );
    CvMat tIntrinsic = cvMat ( 3, 3, CV_64F, mInt3x3.ptr() );
    CvMat tDisCoeff = cvMat ( mDistCoeff.rows, 1, CV_64F, mDistCoeff.ptr() );
    if ( imgPointsAreDestort )
    {
        cvFindExtrinsicCameraParams2 ( pWorldPoints, pImagePoints, &tIntrinsic, &tDisCoeff, &tRotVec, &tTraVec );
    }
    else
    {
        cvFindExtrinsicCameraParams2 ( pWorldPoints, pImagePoints, &tIntrinsic, &tDisCoeff, &tRotVec, &tTraVec );
    }
}

const cv::Mat_<double> CameraGeometry::computeEssential ( const CameraGeometry &A, const CameraGeometry &B )
{
    cv::Mat_<double> M = B.MExt() * A.MInvExt();
    cv::Mat_<double> R = cv::Mat ( M, cv::Range ( 0,3 ), cv::Range ( 0,3 ) );
    cv::Mat_<double> invR = R.inv();
    cv::Mat_<double> b = cv::Mat ( M, cv::Range ( 0,3 ), cv::Range ( 3,4 ) );
    cv::Mat_<double> t = invR * b;
    cv::Mat_<double> S = ( cv::Mat_<double> ( 3,3 ) << 0.0, -t ( 0,2 ), t ( 0,1 ), t ( 0,2 ), 0.0, -t ( 0,0 ), -t ( 0,1 ), t ( 0,0 ), 0.0 );
    cv::Mat_<double> Ei = R*S;
    return B.MInt3x3().t().inv() * Ei * A.MInt3x3().inv();
}

const cv::Point3_<double> CameraGeometry::triangulate(const CameraGeometry &cgA, const CameraGeometry &cgB, const cv::Point_<double> &pntA, const cv::Point_<double> &pntB) {
    cv::Point3_<double> des;

    //V4R::print(cgA.mIntExt, "cgA.mIntExt()");
    //V4R::print(cgB.mIntExt, "cgB.mIntExt()");
    //V4R::print(pntA, "pntA");
    //V4R::print(pntB, "pntB");
    cv::triangulatePoint(cgA.mIntExt, cgB.mIntExt, pntA, pntB, des);
    //V4R::print(des, "des");
    return des;
}



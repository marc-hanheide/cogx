/***************************************************************************
 *   Copyright (C) 2010 by Markus Bader               *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#include "checkerboard.h"
#include <vector>
#include <algorithm>


using namespace V4R;

void Checkerboard::init(cv::Size patternSize, cv::Size_<double> boxSize, bool useSubPix) {
    mPatternSize = patternSize;
    mBoxSize = boxSize;
    mUseSubPix = useSubPix;
    mImagePoints.resize(mPatternSize.area());
    mObjectPoints.resize(mPatternSize.area());
    for (unsigned int j = 0; j < mObjectPoints.size(); j++ ) {
        mObjectPoints[j].x = ( j % patternSize.width ) * mBoxSize.height;
        mObjectPoints[j].y = ( j / patternSize.width ) * mBoxSize.width;
        mObjectPoints[j].z = 0;
    }
};
int Checkerboard::find(const cv::Mat &img, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Mat &rvec, cv::Mat &tvec) {
    mCameraMatrix = cameraMatrix(cv::Rect(0,0,3,3));
    distCoeffs.assignTo( mDistCoeffs, CV_64F);

    mDetectionOK = findChessboardCorners(img, mPatternSize, mImagePoints);
    if (mDetectionOK) {
        CvPoint2D32f *pCorners = (CvPoint2D32f *) &mImagePoints[0];
        IplImage ipl = img;
        if (mUseSubPix) {
            cvFindCornerSubPix ( &ipl, pCorners, mImagePoints.size(), cvSize ( 11,11 ), cvSize ( -1,-1 ), cvTermCriteria ( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ) );
        }
        cv::Mat objPoints(mObjectPoints.size(), 1, CV_32FC3, &mObjectPoints[0]);
        cv::Mat imgPoints(mImagePoints.size(), 1, CV_32FC2, &mImagePoints[0]);
        cv::solvePnP(objPoints, imgPoints, mCameraMatrix, mDistCoeffs, mRVec, mTVec);

        mRVec.convertTo(rvec, rvec.type());
        mTVec.convertTo(tvec, tvec.type());
    }
    return mDetectionOK;
}

void Checkerboard::drawBoard(cv::Mat &img) {
    if (mDetectionOK) {
        cv::Mat imgPoints(mImagePoints.size(), 1, CV_32FC2, &mImagePoints[0]);
        drawChessboardCorners(img, mPatternSize, imgPoints, mDetectionOK);
    }
}


void Checkerboard::drawSystem(cv::Mat &img, const cv::Mat_<double>& cameraMatrix, const cv::Mat_<double>& distCoeffs, const cv::Mat_<double> &rvec, const cv::Mat_<double> &tvec) {
    if (mDetectionOK) {

        cv::Mat_<double> Mint = cv::Mat_<double>::eye ( 4,4 );
        Mint(0,0) = cameraMatrix(0,0); //fx
        Mint(0,2) = cameraMatrix(0,2); //cx
        Mint(1,1) = cameraMatrix(1,1); //fy
        Mint(1,2) = cameraMatrix(1,2); //cy

        double crossSize = std::min(mBoxSize.width * mPatternSize.width, mBoxSize.height * mPatternSize.height);

        cv::Mat_<double> Mext = cv::Mat_<double>::eye ( 4,4 );
        Mext(0,3) = tvec.ptr<double>()[0];
        Mext(1,3) = tvec.ptr<double>()[1];
        Mext(2,3) = tvec.ptr<double>()[2];
        cv::Mat R(Mext, cv::Rect(0, 0, 3, 3));
        cv::Rodrigues(rvec, R);

        int font = cv::FONT_HERSHEY_SIMPLEX;
        cv::Mat_<double> Pw0 = ( cv::Mat_<double> ( 4,1 ) << 0, 0, 0, 1 );
        cv::Mat_<double> Pc0 = Mext * Pw0;
        cv::Mat_<double> Pi0 = Mint * Pc0;
        cv::Point2d pi0 ( Pi0 ( 0,0 ) / Pi0 ( 0,2 ), Pi0 ( 0,1 ) / Pi0 ( 0,2 ) );
        cv::circle ( img, pi0, 3, CV_RGB ( 255,255,255 ) );

        cv::Mat_<double> Pw1 = ( cv::Mat_<double> ( 4,1 ) << crossSize, 0, 0, 1 );
        cv::Mat_<double> Pc1 = Mext * Pw1;
        cv::Mat_<double> Pi1 = Mint * Pc1;
        cv::Point2d pi1 ( Pi1 ( 0,0 ) / Pi1 ( 0,2 ), Pi1 ( 0,1 ) / Pi1 ( 0,2 ) );
        cv::circle ( img, pi1, 3, CV_RGB ( 255,0,0 ) );
        putText ( img, "X", pi1, font, 2, CV_RGB ( 255,0,0 ), 4 );

        cv::Mat_<double> Pw2 = ( cv::Mat_<double> ( 4,1 ) << 0, crossSize, 0, 1 );
        cv::Mat_<double> Pc2 = Mext * Pw2;
        cv::Mat_<double> Pi2 = Mint * Pc2;
        cv::Point2d pi2 ( Pi2 ( 0,0 ) / Pi2 ( 0,2 ), Pi2 ( 0,1 ) / Pi2 ( 0,2 ) );
        cv::circle ( img, pi2, 3, CV_RGB ( 0,255,0 ) );
        putText ( img, "Y", pi2, font, 2, CV_RGB ( 0,255,0 ), 4  );

        cv::Mat_<double> Pw3 = ( cv::Mat_<double> ( 4,1 ) << 0, 0, crossSize, 1 );
        cv::Mat_<double> Pc3 = Mext * Pw3;
        cv::Mat_<double> Pi3 = Mint * Pc3;
        cv::Point2d pi3 ( Pi3 ( 0,0 ) / Pi3 ( 0,2 ), Pi3 ( 0,1 ) / Pi3 ( 0,2 ) );
        cv::circle ( img, pi3, 3, CV_RGB ( 0,0,255 ) );
        putText ( img, "Z", pi3, font, 2, CV_RGB ( 0,0,255 ) , 4 );

        cv::line ( img, pi0, pi1, CV_RGB ( 255,0,0 ), 5 );
        cv::line ( img, pi0, pi2, CV_RGB ( 0,255,0 ), 5 );
        cv::line ( img, pi0, pi3, CV_RGB ( 0,0,255 ), 5 );
    }
}

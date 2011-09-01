/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2009, Intel Corporation and others, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <opencv/cv.h>
namespace cv {

// cvCorrectMatches function is Copyright (C) 2009, Jostein Austvik Jacobsen.
// cvTriangulatePoints function is derived from icvReconstructPointsFor3View, originally by Valery Mosyagin.

// HZ, R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, Cambridge Univ. Press, 2003.



// This method is the same as icvReconstructPointsFor3View, with only a few numbers adjusted for two-view geometry

void triangulatePoints(const Mat &projMatr1, const Mat &projMatr2, const Mat &projPoints1, const Mat &projPoints2, Mat &pointsDes)
{
    int numPoints = projPoints1.cols;

    if ( numPoints < 1 )
        CV_Error( CV_StsOutOfRange, "Number of points must be more than zero" );

    if ( projPoints2.cols != numPoints || pointsDes.cols != numPoints )
        CV_Error( CV_StsUnmatchedSizes, "Number of points must be the same" );

    if ( projPoints1.rows != 2 || projPoints2.rows != 2)
        CV_Error( CV_StsUnmatchedSizes, "Number of proj points coordinates must be == 2" );

    if ( pointsDes.rows != 4 && pointsDes.rows != 3)
        CV_Error( CV_StsUnmatchedSizes, "Number of world points coordinates must be == 4 or 3" );

    if ( projMatr1.cols != 4 || projMatr1.rows != 3 ||
            projMatr2.cols != 4 || projMatr2.rows != 3)
        CV_Error( CV_StsUnmatchedSizes, "Size of projection matrices must be 3x4" );

    Mat_<double> matrA(6,4);

    Mat_<double> projPoints[2];
    Mat_<double> projMatrs[2];

    projPoints1.assignTo(projPoints[0], CV_64F);
    projPoints2.assignTo(projPoints[1], CV_64F);

    projMatr1.assignTo(projMatrs[0], CV_64F);
    projMatr2.assignTo(projMatrs[1], CV_64F);

    /* Solve system for each point */
    int i,j;
    for ( i = 0; i < numPoints; i++ )/* For each point */
    {

        /* Fill matrix for current point */
        for ( j = 0; j < 2; j++ )/* For each view */
        {
            double x,y;
            x = projPoints[j](0,i);
            y = projPoints[j](1,i);
            for ( int k = 0; k < 4; k++ )
            {
                matrA(j*3+0, k) = x * projMatrs[j](2,k) -     projMatrs[j](0,k);
                matrA(j*3+1, k) = y * projMatrs[j](2,k) -     projMatrs[j](1,k);
                matrA(j*3+2, k) = x * projMatrs[j](1,k) - y * projMatrs[j](0,k);
            }
        }

        /* Solve system for current point */
        {
            SVD svd(matrA, SVD::MODIFY_A);
            if (pointsDes.rows == 3) {
                double w = svd.vt.at<double>(3,3);
                if ( pointsDes.type() == CV_64F)
                {
                    pointsDes.at<double>(0,i) = svd.vt.at<double>(3,0)/w;/* X */
                    pointsDes.at<double>(1,i) = svd.vt.at<double>(3,1)/w;/* Y */
                    pointsDes.at<double>(2,i) = svd.vt.at<double>(3,2)/w;/* Z */
                } else if ( pointsDes.type() == CV_32F)
                {
                    pointsDes.at<float>(0,i) = svd.vt.at<double>(3,0)/w;/* X */
                    pointsDes.at<float>(1,i) = svd.vt.at<double>(3,1)/w;/* Y */
                    pointsDes.at<float>(2,i) = svd.vt.at<double>(3,2)/w;/* Z */
                } else {
                    CV_Error( CV_StsUnsupportedFormat, "Output pointsDes matrices must be CV_64F or CV_32F" );
                }
            } else {
                if ( pointsDes.type() == CV_64F)
                {
                    pointsDes.at<double>(0,i) = svd.vt.at<double>(3,0);/* X */
                    pointsDes.at<double>(1,i) = svd.vt.at<double>(3,1);/* Y */
                    pointsDes.at<double>(2,i) = svd.vt.at<double>(3,2);/* Z */
                    pointsDes.at<double>(3,i) = svd.vt.at<double>(3,3);/* W */
                } else if ( pointsDes.type() == CV_32F)
                {
                    pointsDes.at<float>(0,i) = svd.vt.at<double>(3,0);/* X */
                    pointsDes.at<float>(1,i) = svd.vt.at<double>(3,1);/* Y */
                    pointsDes.at<float>(2,i) = svd.vt.at<double>(3,2);/* Z */
                    pointsDes.at<float>(3,i) = svd.vt.at<double>(3,3);/* W */
                } else {
                    CV_Error( CV_StsUnsupportedFormat, "Output pointsDes matrices must be CV_64F or CV_32F" );
                }
            }
        }
    }
}

void triangulatePoint(const Mat_<double> &projMatr1, const Mat_<double> &projMatr2, const Point_<double> &pointA, const Point_<double>  &pointB, cv::Point3_<double> &pointDes)
{
    Mat_<double> matrA(6,4);
    for ( int k = 0; k < 4; k++ ){
        matrA(0, k) = pointA.x * projMatr1(2,k) -            projMatr1(0,k);
        matrA(1, k) = pointA.y * projMatr1(2,k) -            projMatr1(1,k);
        matrA(2, k) = pointA.x * projMatr1(1,k) - pointA.y * projMatr1(0,k);
        matrA(3, k) = pointB.x * projMatr2(2,k) -            projMatr2(0,k);
        matrA(4, k) = pointB.y * projMatr2(2,k) -            projMatr2(1,k);
        matrA(5, k) = pointB.x * projMatr2(1,k) - pointB.y * projMatr2(0,k);
    }
    SVD svd(matrA, SVD::MODIFY_A);
    double w = svd.vt.at<double>(3,3);
    pointDes.x = svd.vt.at<double>(3,0)/w;/* X */
    pointDes.y = svd.vt.at<double>(3,1)/w;/* Y */
    pointDes.z = svd.vt.at<double>(3,2)/w;/* Z */

}
};
/***************************************************************************
 *   Copyright (C) 2009 by Markus Bader                                    *
 *   bader@acin.tuwien.ac.at                                               *
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

/**
 * @file main_camcalib.h
 * @author Markus Bader
 * @brief  Camera Calibration programm \n
 * @version 2.0
 * @date 30th of August 2010
 */


#include <iostream>
#include <vector>
#include <string>
#include <cstdio>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#define CHESSBOARD_WIDTH 8
#define CHESSBOARD_HEIGHT 6
#define CHESSBOARD_BOXES_WIDTH 0.080
#define CHESSBOARD_BOXES_HEIGHT 0.080


/**
* @brief prints a cv matrix in a nice format with an info text/ or in matlab format
* @param pMatix
* @param pInfo
* @param bMatlab on true it print it in matlab format
**/
inline void cvPrint ( const CvMat *pMatix, const char *pInfo = NULL, bool bMatlab = false ) {
    int type = CV_MAT_TYPE ( pMatix->type );
    if ( pInfo != NULL ) {
        if ( bMatlab ) printf ( "%s = [ ", pInfo );
        else {
            printf ( "*** %s  [%i x %i] ***", pInfo, pMatix->rows, pMatix->cols );
            switch ( type ) {
            case CV_8U:
                printf ( " Type: CV_8U\n" );
                break;
            case CV_8S:
                printf ( " Type: CV_8S\n" );
                break;
            case CV_16U:
                printf ( " Type: CV_16U\n" );
                break;
            case CV_16S:
                printf ( " Type: CV_16S\n" );
                break;
            case CV_32S:
                printf ( " Type: CV_32S\n" );
                break;
            case CV_32F:
                printf ( " Type: CV_32F\n" );
                break;
            case CV_64F:
                printf ( " Type: CV_64F\n" );
                break;
            default:
                printf ( " Type: NA\n" );
            }

        }
    }
    for ( int row = 0; row < pMatix->rows; row++ ) {
        for ( int col = 0; col < pMatix->cols; col++ ) {
            if ( type < CV_32F ) {
                printf ( " %-12i", ( int ) cvGet2D ( pMatix, row, col ).val[0] );
            } else {
                printf ( " %-12.6f", cvGet2D ( pMatix, row, col ).val[0] );
            }
        }
        if ( bMatlab ) {
            if ( row < pMatix->rows-1 ) printf ( ";\n" );
            else  printf ( "]\n" );
        } else printf ( "\n" );
    }
}

/**
 * Return inverse pose Ri, ti
 * with (global) world point w, (local) object point o:
 * w = R o + t
 * o = R^T w - R^T t
 * thus:
 * Ri = R^T
 * ti = -R^T t
 */
static void invertPose(const CvMat *t, const CvMat *r, CvMat *ti, CvMat *ri)
{
    // inverse rotation matrix
    CvMat *Ri = cvCreateMat( 3, 3, CV_64FC1 );
    cvConvertScale(r, ri, -1.);
    cvRodrigues2(ri, Ri);
    cvGEMM(Ri, t, -1., NULL, 0., ti);
    cvReleaseMat(&Ri);
}

/**
 * returns Pd = P1 * P2
 * with P1, P2 poses given as translation and rotation vectors and Pd the
 * destination pose.
 * w = R1 ( R2 o + t2 ) + t1
 *   = R1 R2 o + R1 t2 + t1
 * Rd = R1 R2
 * td = R1 t2 + t1
 */
static void multPose(const CvMat *t1, const CvMat *r1, const CvMat *t2, const CvMat *r2,
    CvMat *td, CvMat *rd)
{
    // rotation matrices
    CvMat *R1 = cvCreateMat( 3, 3, CV_64FC1 );
    CvMat *R2 = cvCreateMat( 3, 3, CV_64FC1 );
    CvMat *Rd = cvCreateMat( 3, 3, CV_64FC1 );
    cvRodrigues2(r1, R1);
    cvRodrigues2(r2, R2);
    cvGEMM(R1, R2, 1., NULL, 0., Rd);
    cvGEMM(R1, t2, 1., t1, 1., td);
    cvRodrigues2(Rd, rd);
    cvReleaseMat(&R1);
    cvReleaseMat(&R2);
    cvReleaseMat(&Rd);
}

static void storeOpenCvPose(cv::FileStorage &file, CvMat *tvec, CvMat *rvec) {
	CvMat *R = cvCreateMat( 3, 3, CV_64FC1 );
	cvRodrigues2(rvec, R);
	file.writeObj( "tvec", tvec );
	file.writeObj( "rvec", rvec );
	file.writeObj( "rmat", R );
	cvReleaseMat(&R);
}

int main ( int argc, char *argv[] ) {

    CvSize pattern_size = cvSize ( CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT );
    float fBoardBoxWidth = CHESSBOARD_BOXES_WIDTH;
    float fBoardBoxHeight = CHESSBOARD_BOXES_HEIGHT;
    bool bShowImages = true;
    bool bShowCalibration = true;
    bool bInvertPoses = false;

    char *pImageFile = NULL;
    char *pCalibFile = NULL;
    int width = 0, height = 0;
    CvMat *pTraVecPattern2World = 0;
    CvMat *pRotVecPattern2World = 0;

    if ( argc < 3 ) {
        printf ( "usage: %s <image> <camera calibration> [ -c -o -i -m -v]\n", argv[0] );
        printf ( "sample Linux  : %s img.png camcal.xml -c 8 6 80 80 -o offset.xml\n", argv[0] );
        printf ( "sample Windows: %s img.gif camcal.xml -c 8 6 80 80 -o offset.xml\n", argv[0] );
        printf ( "-c	<width> <height> <boxwidth> <boxheight> default is 8 x 6 boxes size of 80 mm\n" );
        printf ( "-o	<pose> pose offset file of calibration pattern w.r.t. world\n" );
        printf ( "-m	display the calibration off\n" );
        printf ( "-i	display images off \n" );
        printf ( "-v	invert poses: return pose of cam w.r.t. world instead of "
            "world w.r.t. cam (which is the OpenCV convention)\n");
        exit ( 1 );
    }
    pImageFile = argv[1];
    pCalibFile = argv[2];

    for ( int i = 3; i < argc; i++ ) {
        if ( strcmp ( argv[i], "-c" ) == 0 ) {
            if ( i + 4 >= argc ) {
                printf ( "Four parameters needed for %s, aborting.\n", argv[i] );
                exit ( 1 );
            }
            pattern_size = cvSize ( atoi ( argv[i + 1] ), atoi ( argv[i + 2] ) );
            // note: user gives pattern size in mm but we use m
            fBoardBoxWidth = atof ( argv[i + 3] ) / 1000.;
            fBoardBoxHeight = atof ( argv[i + 4] ) / 1000.;
            if(fBoardBoxWidth <= 0.0f || fBoardBoxHeight <= 0.0f){
                printf ( "Parameter <boxwidth> <boxheight> must be greater than zero, aborting.\n" );
                exit ( 1 );
            }
        }
        if ( strcmp ( argv[i], "-i" ) == 0 ) {
            bShowImages = false;
        }
        if ( strcmp ( argv[i], "-v" ) == 0 ) {
            bInvertPoses = true;
        }
        if ( strcmp ( argv[i], "-m" ) == 0 ) {
            bShowCalibration = false;
        }
        if ( strcmp ( argv[i], "-o" ) == 0 ) {
            if ( i + 1 >= argc ) {
                printf ( "One parameter needed for %s, aborting.\n", argv[i] );
                exit ( 1 );
            }
            cv::FileStorage poseFile( argv[i + 1], cv::FileStorage::READ );
            pTraVecPattern2World = cvCloneMat((CvMat*)poseFile["tvec"].readObj());
            pRotVecPattern2World = cvCloneMat((CvMat*)poseFile["rvec"].readObj());
        }
    }

    if ( pTraVecPattern2World == 0 )
    {
        pTraVecPattern2World = cvCreateMat( 3, 1, CV_64FC1 );
        pRotVecPattern2World = cvCreateMat( 3, 1, CV_64FC1 );
        cvSet(pTraVecPattern2World, cvScalar(0));
        cvSet(pRotVecPattern2World, cvScalar(0));
    }

    int iNrOfCorners = pattern_size.width * pattern_size.height;

    IplImage *pImg = cvLoadImage ( pImageFile, 1 );
    IplImage *pImgGray = cvCreateImage ( cvGetSize ( pImg ), IPL_DEPTH_8U, 1 );
    cvReleaseImage ( &pImg );

    int pattern_was_found = 0;
    int corner_count = 0;
    cvNamedWindow ( "Camera", 1 );
    cvNamedWindow ( "CameraSubPix", 1 );
    std::vector <CvPoint3D64f> object_points;
    std::vector <CvPoint2D64f> image_points;
    std::vector <int> count_points;

    pImg = cvLoadImage ( pImageFile, 1 );
    width = pImg->width;
    height = pImg->height;
    cvCvtColor ( pImg, pImgGray, CV_BGR2GRAY );
    if ( ( pImgGray->width != pImg->width ) || ( pImgGray->height != pImg->height ) ) {
        std::cout << " images has a different size\n";
        exit ( 1 );
    }
    CvPoint2D32f *pImageCorners = ( CvPoint2D32f* ) malloc ( sizeof ( CvPoint2D32f ) * iNrOfCorners );
    pattern_was_found = cvFindChessboardCorners ( pImg, pattern_size, pImageCorners, &corner_count, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS );
    cvDrawChessboardCorners ( pImg, pattern_size, pImageCorners, corner_count, pattern_was_found );
    if ( pattern_was_found ) {
        std::cout << " pattern OK\n";
        cvFindCornerSubPix ( pImgGray, pImageCorners, corner_count, cvSize ( 11,11 ), cvSize ( -1,-1 ), cvTermCriteria ( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ) );
        for ( int j = 0; j < corner_count; j++ ) {
            CvPoint2D64f imageCorner = cvPoint2D64f(pImageCorners[j].x,pImageCorners[j].y);
            image_points.push_back ( imageCorner );
            CvPoint3D64f worldPoint;
            worldPoint.x = ( j % pattern_size.width ) * fBoardBoxHeight;
            worldPoint.y = ( j / pattern_size.width ) * fBoardBoxWidth;
            worldPoint.z = 0;
            object_points.push_back ( worldPoint );
        }
        count_points.push_back ( iNrOfCorners );

        cvDrawChessboardCorners ( pImgGray, pattern_size, pImageCorners, corner_count, pattern_was_found );
        cvShowImage ( "CameraSubPix", pImgGray );
    }

    cvShowImage ( "Camera", pImg );
    cvWaitKey ( 0 );
    free ( pImageCorners );

    CvMat *pImagePoints = cvCreateMat ( image_points.size(),2,CV_64FC1 );
    CvMat *pObjectPoints = cvCreateMat ( object_points.size(),3,CV_64FC1 );
    CvMat *pTraVecPattern2Cam = cvCreateMat( 3, 1, CV_64FC1 );
    CvMat *pRotVecPattern2Cam = cvCreateMat( 3, 1, CV_64FC1 );
    CvMat *pTraVecCam2Pattern = cvCreateMat( 3, 1, CV_64FC1 );
    CvMat *pRotVecCam2Pattern = cvCreateMat( 3, 1, CV_64FC1 );
    CvMat *pTraVecCam2World = cvCreateMat( 3, 1, CV_64FC1 );
    CvMat *pRotVecCam2World = cvCreateMat( 3, 1, CV_64FC1 );
    CvMat *pIntrinsic = 0;
    CvMat *pDistortions = 0;

    for ( unsigned int i = 0; i < image_points.size(); i++ ) {
        CV_MAT_ELEM ( *pImagePoints, double, i, 0 ) = image_points[i].x;
        CV_MAT_ELEM ( *pImagePoints, double, i, 1 ) = image_points[i].y;
        CV_MAT_ELEM ( *pObjectPoints, double, i, 0 ) = object_points[i].x;
        CV_MAT_ELEM ( *pObjectPoints, double, i, 1 ) = object_points[i].y;
        CV_MAT_ELEM ( *pObjectPoints, double, i, 2 ) = object_points[i].z;
    }

    cv::FileStorage calibFile( pCalibFile, cv::FileStorage::READ );
    pIntrinsic = (CvMat*)calibFile["intrinsic"].readObj();
    pDistortions = (CvMat*)calibFile["distortion"].readObj();

    printf("estimating pose, please wait ...\n");
    cvFindExtrinsicCameraParams2( pObjectPoints, pImagePoints, pIntrinsic, pDistortions,
        pRotVecPattern2Cam, pTraVecPattern2Cam );

    // calculate the pose of the camera w.r.t. the world, which is the inverse
    // of the OpenCV convention
    invertPose(pTraVecPattern2Cam, pRotVecPattern2Cam, pTraVecCam2Pattern, pRotVecCam2Pattern);
    multPose(pTraVecPattern2World, pRotVecPattern2World, pTraVecCam2Pattern, pRotVecCam2Pattern,
        pTraVecCam2World, pRotVecCam2World);

    cv::FileStorage poseFile( "campose.xml", cv::FileStorage::WRITE );
    if ( bInvertPoses ) {
        storeOpenCvPose(poseFile, pTraVecCam2World, pRotVecCam2World);
    } else {
        // if we do not want the inverse, i.e. we want world w.r.t. camera
        CvMat *pTraVecWorld2Cam = cvCreateMat( 3, 1, CV_64FC1 );
        CvMat *pRotVecWorld2Cam = cvCreateMat( 3, 1, CV_64FC1 );
        invertPose(pTraVecCam2World, pRotVecCam2World, pTraVecWorld2Cam,
            pRotVecWorld2Cam);
        storeOpenCvPose(poseFile, pTraVecWorld2Cam, pRotVecWorld2Cam);
    }

    printf("\nparameters saved to file: %s\n", "campose.xml");

    /*double o[6] = {0., 0., 0., 0., 0., 0.}, oi[4];
    CvMat omat = cvMat(1, 6, CV_64FC1, o);
    CvMat oimat = cvMat(1, 4, CV_64FC1, oi);
    cvProjectPoints2(&omat, pRotVecCam2World, pTraVecCam2World, pIntrinsic, pDistortions, &oimat);
    cvCircle(pImg, cvPoint(oi[0], oi[1]), 2, cvScalar(0, 255, 0));
    cvShowImage ( "Camera", pImg );
    cvWaitKey ( 0 );*/

    // HACK
    /*{
    double R[9] = {0, -1,  0,
                  -1,  0,  0,
                   0,  0,  -1};
    double r[3] = {0, 0, 0};
    CvMat Rmat = cvMat(3, 3, CV_64FC1, R);
    CvMat rvec = cvMat(3, 1, CV_64FC1, r);
    cvRodrigues2(&Rmat, &rvec);
    cvPrint ( &rvec, "rvec" );
    }*/

    cvReleaseMat ( &pImagePoints );
    cvReleaseMat ( &pObjectPoints );
    cvReleaseMat ( &pIntrinsic );
    cvReleaseMat ( &pDistortions );
    cvReleaseMat ( &pTraVecPattern2World );
    cvReleaseMat ( &pRotVecPattern2World );
    cvReleaseMat ( &pTraVecPattern2Cam );
    cvReleaseMat ( &pRotVecPattern2Cam );
    cvReleaseMat ( &pTraVecCam2Pattern );
    cvReleaseMat ( &pRotVecCam2Pattern );
    cvReleaseMat ( &pTraVecCam2World );
    cvReleaseMat ( &pRotVecCam2World );
    cvReleaseImage ( &pImg );
    cvReleaseImage ( &pImgGray );
    cvDestroyWindow ( "Camera" );
    cvDestroyWindow ( "cvUndistortPoints" );
    cvDestroyWindow ( "CameraSubPix" );
    return 0;
}


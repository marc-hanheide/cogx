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

#include <boost/regex.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#define CHESSBOARD_WIDTH 8
#define CHESSBOARD_HEIGHT 6
#define CHESSBOARD_BOXES_WIDTH 0.030
#define CHESSBOARD_BOXES_HEIGHT 0.030


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
                printf ( " %-12.4f", cvGet2D ( pMatix, row, col ).val[0] );
            }
        }
        if ( bMatlab ) {
            if ( row < pMatix->rows-1 ) printf ( ";\n" );
            else  printf ( "]\n" );
        } else printf ( "\n" );
    }
}

/**
* @brief Undistorts a point
* @param pIntrinsicDistort	Intrinsic Matrinx of the distort image (camera image) in [3x3] matrix
* @param pDistortion	DistortionCoeff pointer to an array [k1, k2, p1, p2]
* @param pSrc	source point [x, y]
* @param pDes	destination point [x, y]
**/
inline bool undistort ( const double *pIntrinsicDistort, const double *pDistortion, const double *pSrc, double *pDes, double maxError = 0.1 ) {

  CvMat intr = cvMat(3,3, CV_64FC1, (void*) pIntrinsicDistort);
  CvMat dist = cvMat(1,4, CV_64FC1, (void*) pDistortion);
  CvMat src = cvMat(1,1, CV_64FC2, (void*) pSrc);
  CvMat des = cvMat(1,1, CV_64FC2, (void*) pDes);
  cvUndistortPoints(&src, &des, &intr, &dist);
  pDes[0] = pDes[0]*pIntrinsicDistort[0] + pIntrinsicDistort[2];
  pDes[1] = pDes[1]*pIntrinsicDistort[4] + pIntrinsicDistort[5];
  return true;
  
}

/** compares to strings
* @param A
* @param B
* @return return false on a match
*/
inline bool compareCaseInsensitive( const std::string &left, const std::string &right ) {
    for ( std::string::const_iterator lit = left.begin(), rit = right.begin(); lit != left.end() && rit != right.end(); ++lit, ++rit ) {
        if ( tolower( *lit ) < tolower( *rit ) ) {
            return true;
        } else if ( tolower( *lit ) > tolower( *rit ) ) {
            return false;
        }
    }
    if ( left.size() < right.size() ) {
        return true;
    }
    return false;
}

/** Returns a the name of files in a folder </br>
* '(.*)bmp'
* @param rFolder
* @param rFiles
* @param regExpressions examples "(.*)bmp",  "(.*)$"
* @return Number of files
*/
int getFilesInFolder ( const std::string &rFolder,  std::vector<std::string> &rFiles, const std::string regx = "(.*)$" ) {
    using namespace boost::filesystem;
    path fullPath = system_complete ( path ( rFolder.c_str(), native ) );

    if ( !exists ( fullPath ) ) {
        std::cerr << "Error: the directory " << fullPath.string( ) << " does not exist.\n";
        return ( -1 );
    }
    if ( !is_directory ( fullPath ) ) {
        std::cout << fullPath.string( ) << " is not a directory!\n";
        return ( -1 );
    }

    static const boost::regex expression ( regx );
    int nrOfFiles = 0;
    directory_iterator end;
    for ( directory_iterator it ( fullPath ); it != end; ++it ) {
        std::string filename = it->filename();
        if ( !is_directory ( *it ) && boost::regex_match ( filename, expression ) ) {
            std::string fileNameFull = it->string();
            rFiles.push_back ( fileNameFull );
            std::cout << it->filename() << std::endl;
            nrOfFiles++;
        }
    }
    sort( rFiles.begin(), rFiles.end(), compareCaseInsensitive );
    return nrOfFiles;
}

int main ( int argc, char *argv[] ) {

    CvSize pattern_size = cvSize ( CHESSBOARD_WIDTH, CHESSBOARD_HEIGHT );
    float fBoardBoxWidth = CHESSBOARD_BOXES_WIDTH;
    float fBoardBoxHeight = CHESSBOARD_BOXES_HEIGHT;
    bool bShowImages = true;
    bool bShowCalibration = true;

    char *pImageFolder = NULL;
    char *pRegx = NULL;
    int width = 0, height = 0;
    if ( argc < 3 ) {
        printf ( "usage: %s < folder regx > [ -c -i -m -p] \n", argv[0] );
        printf ( "sample Linux  : %s /images/calib '(.*)bmp' -c 8 6 30 30\n", argv[0] );
        printf ( "sample Windows: %s \\images\\calib '(.*)bmp' -c 8 6 30 30\n", argv[0] );
        printf ( "-c	<width> <height> <boxwidth> <boxheight> default is 8 x 6 boxes size of 30 mm\n" );
        printf ( "-m	display the calibration off\n" );
        printf ( "-i	display images off \n" );
        exit ( 1 );
    }
    pImageFolder = argv[1];
    pRegx = argv[2];

    for ( int i = 3; i < argc; i++ ) {
        if ( strcmp ( argv[i], "-c" ) == 0 ) {
            if ( i + 4 >= argc ) {
                printf ( "Four parameters needed for %s, aborting.\n", argv[i] );
                exit ( 1 );
            }
            pattern_size = cvSize ( atoi ( argv[i + 1] ), atoi ( argv[i + 2] ) );
            fBoardBoxWidth = atof ( argv[i + 3] ) / 1000.;
            fBoardBoxHeight = atof ( argv[i + 4] ) / 1000.;
            if(fBoardBoxWidth <= 0.0f || fBoardBoxHeight <= 0.0f){
                printf ( "Parameter <boxwidth> <boxheight> must be greater than zero, aborting.\n" );
                exit ( 1 );
            }
        }
        if ( strcmp ( argv[i], "-p" ) == 0 ) {
            if ( i + 1 >= argc ) {
                printf ( "One parameter needed for %s, aborting.\n", argv[i] );
                exit ( 1 );
            };
            
        }
        if ( strcmp ( argv[i], "-i" ) == 0 ) {
            bShowImages = false;
        }
        if ( strcmp ( argv[i], "-m" ) == 0 ) {
            bShowCalibration = false;
        }
    }

    int iNrOfCorners = pattern_size.width * pattern_size.height;

    // The program

    std::vector<std::string> files;
    if ( getFilesInFolder ( pImageFolder,files, pRegx ) < 1 ) {
        printf ( "no images detected\n" );
        exit ( 1 );
    }
    IplImage *pImg = cvLoadImage ( files[0].c_str(), 1 );
    IplImage *pImgGray = cvCreateImage ( cvGetSize ( pImg ), IPL_DEPTH_8U, 1 );
    cvReleaseImage ( &pImg );

    std::vector<std::string> usedFiles;
    int pattern_was_found = 0;
    int corner_count = 0;
    cvNamedWindow ( "Camera", 1 );
    cvNamedWindow ( "CameraSubPix", 1 );
    std::vector <CvPoint3D64f> object_points;
    std::vector <CvPoint2D64f> image_points;
    std::vector <int> count_points;
    using namespace boost::filesystem;
    for ( unsigned int i = 0; i < files.size(); i++ ) {
        path filePath ( files[i].c_str() );
        std::cout << "File: " << filePath.filename();
        pImg = cvLoadImage ( files[i].c_str(), 1 );
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
            std::cout << " pattern OK ";
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
            usedFiles.push_back ( files[i] );

            cvDrawChessboardCorners ( pImgGray, pattern_size, pImageCorners, corner_count, pattern_was_found );
            cvShowImage ( "CameraSubPix", pImgGray );
        }
        std::cout << std::endl;

        cvShowImage ( "Camera", pImg );
        cvWaitKey ( 10 );
        cvReleaseImage ( &pImg );
        free ( pImageCorners );
    }

    CvMat *pImagePoints = cvCreateMat ( image_points.size(),2,CV_64FC1 );
    CvMat *pObjectPoints = cvCreateMat ( object_points.size(),3,CV_64FC1 );
    CvMat *pPointsCounts = cvCreateMat ( count_points.size(),1,CV_32SC1 );
    CvMat *pRotVecs = cvCreateMat( count_points.size(), 3, CV_64FC1 );
    CvMat *pTraVecs = cvCreateMat( count_points.size(), 3, CV_64FC1 );
    CvMat *pIntrinsic = cvCreateMat ( 3,3,CV_64FC1 );
    CvMat *pDistortions = cvCreateMat ( 4,1,CV_64FC1 );

    for ( unsigned int i = 0; i < image_points.size(); i++ ) {
        CV_MAT_ELEM ( *pImagePoints, double, i, 0 ) = image_points[i].x;
        CV_MAT_ELEM ( *pImagePoints, double, i, 1 ) = image_points[i].y;
        CV_MAT_ELEM ( *pObjectPoints, double, i, 0 ) = object_points[i].x;
        CV_MAT_ELEM ( *pObjectPoints, double, i, 1 ) = object_points[i].y;
        CV_MAT_ELEM ( *pObjectPoints, double, i, 2 ) = object_points[i].z;
    }
    for ( unsigned int i = 0; i < count_points.size(); i++ ) {
        CV_MAT_ELEM ( *pPointsCounts, int , i, 0 ) = count_points[i];
    }
    CV_MAT_ELEM ( *pIntrinsic, double, 0, 0 ) = 1.0f;
    CV_MAT_ELEM ( *pIntrinsic, double, 1, 1 ) = 1.0f;

    printf("calibrating camera, please wait ...\n");
    cvCalibrateCamera2 ( pObjectPoints, pImagePoints, pPointsCounts, cvGetSize ( pImgGray ), pIntrinsic, pDistortions, NULL, NULL, 0 );

    CvMat *pImgSize = cvCreateMat ( 2,1,CV_64FC1 );
    cvSetReal1D(pImgSize, 0, width);
    cvSetReal1D(pImgSize, 1, height);

    // TODO: make the file name a command line option
    cv::FileStorage calibFile( "camcalib.xml", cv::FileStorage::WRITE );
    calibFile.writeObj( "imgsize", pImgSize );
    calibFile.writeObj( "intrinsic", pIntrinsic );
    calibFile.writeObj( "distortion", pDistortions );
    calibFile.writeObj( "tvecs", pTraVecs );
    calibFile.writeObj( "rvecs", pRotVecs );
    printf("\nparameters saved to file: %s\n", "camcalib.xml");

    cvReleaseMat ( &pImagePoints );
    cvReleaseMat ( &pObjectPoints );
    cvReleaseMat ( &pPointsCounts );
    cvReleaseMat ( &pIntrinsic );
    cvReleaseMat ( &pDistortions );
    cvReleaseImage ( &pImgGray );
    cvDestroyWindow ( "Camera" );
    cvDestroyWindow ( "cvUndistortPoints" );
    cvDestroyWindow ( "CameraSubPix" );
    return 0;
}


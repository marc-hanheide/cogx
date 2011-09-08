/*
 * 1394-Based Digital Camera Control Library
 *
 * Written by Damien Douxchamps <ddouxchamps@users.sf.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <cstdio>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cctype>
#include <boost/regex.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
#include <iomanip>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <boost/concept_check.hpp>


#define CVPRINT( A ) print( A, #A"");
/**
* @brief prints a cv matrix in a nice format with an info text/ or in matlab format
* @param rMatrix
* @param pInfo
**/
inline void print ( const cv::Mat &rMatix, const char *pInfo = NULL) {
    int type = CV_MAT_TYPE(rMatix.type());
    if ( pInfo != NULL ) {
        printf ( "%s  [%i x %i] ***", pInfo, rMatix.rows, rMatix.cols );
        switch (type) {
        case CV_8U:
            printf ( " Type: CV_8U\n");
            break;
        case CV_8S:
            printf ( " Type: CV_8S\n");
            break;
        case CV_16U:
            printf ( " Type: CV_16U\n");
            break;
        case CV_16S:
            printf ( " Type: CV_16S\n");
            break;
        case CV_32S:
            printf ( " Type: CV_32S\n");
            break;
        case CV_32F:
            printf ( " Type: CV_32F\n");
            break;
        case CV_32FC2:
            printf ( " Type: CV_32FC2\n");
            break;
        case CV_64F:
            printf ( " Type: CV_64F\n");
            break;
        case CV_64FC2:
            printf ( " Type: CV_64FC2\n");
            break;
        default:
            printf ( " Type: NA\n");
        }

    }

    printf( "[");
    for ( int row = 0; row < rMatix.rows; row++ ) {
        for ( int col = 0; col < rMatix.cols; col++ ) {
            switch (type) {
            case CV_8U:
                printf ( " %-12i",   rMatix.at<uchar>(row, col) );
                break;
            case CV_8S:
                printf ( " %-12i",   rMatix.at<char>(row, col) );
                break;
            case CV_16U:
                printf ( " %-12i",   rMatix.at<uint16_t>(row, col) );
                break;
            case CV_16S:
                printf ( " %-12i",   rMatix.at<int16_t>(row, col) );
                break;
            case CV_32S:
                printf ( " %-12i",   rMatix.at<int32_t>(row, col) );
                break;
            case CV_32F:
                printf ( " %-12.4f",   rMatix.at<float>(row, col) );
                break;
            case CV_32FC2:
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2) );
                printf ( " %-12.4f",   rMatix.at<float>(row, col*2+1) );
                break;
            case CV_64F:
                printf ( " %-12.4f",   rMatix.at<double>(row, col) );
                break;
            case CV_64FC2:
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2) );
                printf ( " %-12.4f",   rMatix.at<double>(row, col*2+1) );
                break;
            default:
                printf ( " Type: NA\n");
            }
        }
        if ( row < rMatix.rows-1) printf ( ";\n");
        else  printf( "]\n");
    }
}

void createFolder ( const std::string &rFolder ) {
    boost::filesystem::path dir_path = boost::filesystem::complete ( boost::filesystem::path ( rFolder, boost::filesystem::native ) );
    if ( !boost::filesystem::exists ( dir_path ) || !boost::filesystem::is_directory ( dir_path ) ) {
        boost::filesystem::create_directory ( dir_path );
    }
}

bool compareCaseInsensitive( const std::string &left, const std::string &right ) {
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

bool existsFolder ( const std::string &rFolder ) {
    boost::filesystem::path dir_path = boost::filesystem::complete ( boost::filesystem::path ( rFolder, boost::filesystem::native ) );
    if ( boost::filesystem::exists ( dir_path ) && boost::filesystem::is_directory(dir_path)) {
        return true;
    } else {
        return false;
    }
}

std::vector < std::vector <cv::Point2f> > undistortPoints(const std::vector < std::vector <cv::Point2f> > src, const cv::Mat_<double>& cameraMatrix, const cv::Mat_<double>& distCoeffs) {
    std::vector < std::vector <cv::Point2f> > des(src.size());
    for (unsigned int j = 0; j < src.size(); j++) {
        des[j].resize(src[j].size());
        cv::Mat s(src[j]);
        cv::Mat d(des[j]);
        cv::undistortPoints(s, d, cameraMatrix, distCoeffs);
        for (unsigned int i = 0; i < des[j].size(); i++) {
            des[j][i].x = des[j][i].x * cameraMatrix(0,0) +  cameraMatrix(0,2);
            des[j][i].y = des[j][i].y * cameraMatrix(1,1) +  cameraMatrix(1,2);
        }
    }
    return des;
}

int getFilesInFolder ( const std::string &rFolder,  std::vector<std::string> &rFiles, const std::string regx) {
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
            //cout << it->filename() << endl;
            nrOfFiles++;
        }
    }
    sort( rFiles.begin(), rFiles.end(), compareCaseInsensitive );
    return nrOfFiles;
}

std::string getFileNameOfPathToFile ( const std::string &rPathToFile ) {
    int start = rPathToFile.rfind("/");
    if (start == -1) start = 0;
    else start += 1;
    std::string filename = rPathToFile.substr(start);
    return filename;
}

namespace po = boost::program_options;

int main ( int argc, char *argv[] ) {
    // declare which options we want to read
    bool error = false;
    int pattern_rows, pattern_columns, every;
    double pattern_boxheight, pattern_boxwidth;
    std::string nameConfigFile, nameFolderA, nameFolderB, strRegExA, strRegExB, strOut;
    std::vector<std::string> nameFilesA, nameFilesB;
    std::string nameIntrLeft, nameIntrRight;
    po::options_description desc("Allowed Parameters");
    desc.add_options()
    ("help", "get this help message")
    ("configfile,c", po::value<std::string>(&nameConfigFile), "Config file")
    ("pattern_rows,r", po::value<int>(&pattern_rows)->default_value(8), "Pattern number of rows, default 8")
    ("pattern_columns,p", po::value<int>(&pattern_columns)->default_value(6), "Pattern number of columns, default 6")
    ("pattern_boxheight,h", po::value<double>(&pattern_boxheight)->default_value(30), "Pattern box height, default 30 [mm]")
    ("pattern_boxwidth,w", po::value<double>(&pattern_boxwidth)->default_value(30), "Pattern box width, default 30 [mm]")
    ("folderA,A", po::value<std::string>(&nameFolderA)->default_value("./00"), "Image folderA")
    ("folderB,B", po::value<std::string>(&nameFolderB)->default_value("./01"), "Image folderB")
    ("every,e", po::value<int>(&every)->default_value(1), "If set it uses only every x file")
    ("out,o", po::value<std::string>(&strOut)->default_value(""), "output folder")
    ("regexA,a", po::value<std::string>(&strRegExA)->default_value("(.*)bmp"), "Regular expression (.*)bmp")
    ("regexB,b", po::value<std::string>(&strRegExB)->default_value("(.*)bmp"), "Regular expression (.*)bmp")
    ("leftintr,y", po::value<std::string>(&nameIntrLeft)->default_value(""), "left intrinsic paramters")
    ("rightintr,z", po::value<std::string>(&nameIntrRight)->default_value(""), "right intrinsic paramters");

    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (const std::exception& ex) {
        std::cout << desc << "\n";
        return 1;
    }
    po::notify(vm);

    if (vm.count("help"))  {
        std::cout << desc << "\n";
        return 1;
    }

    if (!nameConfigFile.empty())  {
        printf("configfile:    %s\n", nameConfigFile.c_str());
        std::ifstream file(nameConfigFile.c_str(), std::ifstream::in);
        if (file.is_open()) {
            try {
                po::store(po::parse_config_file(file, desc), vm);
                po::notify(vm);
            } catch (const std::exception& ex) {
                std::cout << "Error reading config file: " << ex.what()  << std::endl;
                return 1;
            }
        } else {
            std::cout << "Error opening config file " << nameConfigFile << std::endl;
            return 1;
        }
    }


    if (!nameFolderA.empty()) {
        if (!existsFolder(nameFolderA)) {
            std::cout << "ERROR:  folderA: " << nameFolderA << " does not exist " << std::endl;
            error = true;
        } else {
            printf("folderA:       %s  %s\n", nameFolderA.c_str(), strRegExA.c_str());
            getFilesInFolder(nameFolderA, nameFilesA, strRegExA);
        }
    } else {
        std::cout << "Missing option: folderA " << std::endl;
        error = true;
    }

    if (!nameFolderB.empty()) {
        if (!existsFolder(nameFolderB)) {
            std::cout << "ERROR:  folderB: " << nameFolderB << " does not exist " << std::endl;
            error = true;
        } else {
            printf("folderB:       %s  %s\n", nameFolderB.c_str(), strRegExB.c_str());
            getFilesInFolder(nameFolderB, nameFilesB, strRegExB);
        }
    } else {
        std::cout << "Missing option: folderB " << std::endl;
        error = true;
    }
    if (error) {
        return 1;
    }


    /// Look for the minimum of files
    unsigned int nrOfImages = nameFilesA.size() < nameFilesB.size() ? nameFilesA.size() : nameFilesB.size();


    cv::Mat imgA, imgB, imgDM;
    cv::Mat imgADebug, imgBDebug;
    std::vector < std::vector <cv::Point3f> > objectPoints;
    std::vector < std::vector <cv::Point2f> > imagePoints1, imagePoints2;
    std::vector<cv::Mat> rvecs1(3), rvecs2(3);
    std::vector<cv::Mat> tvecs1(3), tvecs2(3);
    cv::Mat cameraMatrix1(3,3, CV_32F), distCoeffs1(1,5, CV_32F);
    cv::Mat cameraMatrix2(3,3, CV_32F), distCoeffs2(1,5, CV_32F);
    cv::Size imageSize;
    cv::Mat R, T, E, F;
    std::string wndA("imgA");
    std::string wndB("imgB");
    cv::Size patternSize(pattern_rows, pattern_columns);
    std::vector <cv::Point3f> patternPoints(patternSize.area());
    for (unsigned int j = 0; j < patternPoints.size(); j++ ) {
        patternPoints[j].x = ( j % patternSize.width ) * pattern_boxheight;
        patternPoints[j].y = ( j / patternSize.width ) * pattern_boxwidth;
        patternPoints[j].z = 0;
    }

    for(unsigned i = 0; i<nameFilesA.size(); i++)
      std::cout << nameFilesA[i] << std::endl;
    
    for (unsigned int i = 0; (i < nrOfImages) && (i < nameFilesA.size()) && (i < nameFilesB.size()); i+=every) {
        std::string fileA = getFileNameOfPathToFile(nameFilesA[i]);
        std::string fileB = getFileNameOfPathToFile(nameFilesB[i]);
        std::cout << std::setw(3) << i  <<  "  " << fileA << std::setw(60-fileA.length()) << fileB;


        imgA = cv::imread(nameFilesA[i], 0);
        imgB = cv::imread(nameFilesB[i], 0);
        imageSize.width = imgA.cols;
        imageSize.height = imgA.rows;
        cv::cvtColor(imgA, imgADebug, CV_GRAY2RGB);
        cv::cvtColor(imgB, imgBDebug, CV_GRAY2RGB);

        std::vector<cv::Point2f> cornersA, cornersB;
        bool cornersAOK = cv::findChessboardCorners(imgA, patternSize, cornersA);
        bool cornersBOK = cv::findChessboardCorners(imgB, patternSize, cornersB);

        if (cornersAOK) {
            IplImage img = imgA;
            CvPoint2D32f *pCorners = (CvPoint2D32f *) &cornersA[0];
            cvFindCornerSubPix ( &img, pCorners, cornersA.size(), cvSize ( 11,11 ), cvSize ( -1,-1 ), cvTermCriteria ( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ) );
            drawChessboardCorners(imgADebug, patternSize, cv::Mat(cornersA), cornersAOK);
        }
        if (cornersBOK) {
            IplImage img = imgB;
            CvPoint2D32f *pCorners = (CvPoint2D32f *) &cornersB[0];
            cvFindCornerSubPix ( &img, pCorners, cornersB.size(), cvSize ( 11,11 ), cvSize ( -1,-1 ), cvTermCriteria ( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ) );
            drawChessboardCorners(imgBDebug, patternSize, cv::Mat(cornersB), cornersBOK);
        }
        if (cornersAOK && cornersBOK)  {
            objectPoints.push_back(patternPoints);
            imagePoints1.push_back(cornersA);
            imagePoints2.push_back(cornersB);
            std::cout << " OK";
        }
        std::cout << std::endl;
        cv::imshow(wndA, imgADebug);
        cv::imshow(wndB, imgBDebug);
        cv::waitKey(10);
    }
    if (objectPoints.size() > 0)  {
        if (nameIntrLeft.empty() && nameIntrRight.empty()) {
            cv::calibrateCamera(objectPoints, imagePoints1, imageSize, cameraMatrix1, distCoeffs1, rvecs1, tvecs1, 0);
            cv::calibrateCamera(objectPoints, imagePoints2, imageSize, cameraMatrix2, distCoeffs2, rvecs2, tvecs2, 0);
        } else {
            CvMat *intr, *dist;
            printf("left calibration file:    %s\n", nameIntrLeft.c_str());
            printf("right calibration file:    %s\n", nameIntrRight.c_str());

            cv::FileStorage leftFile(nameIntrLeft, cv::FileStorage::READ);
            if (leftFile.isOpened()) {
                intr = (CvMat*)leftFile["intrinsic"].readObj();
                dist = (CvMat*)leftFile["distortion"].readObj();
                cameraMatrix1 = intr;
                distCoeffs1 = dist;
            } else {
                throw runtime_error("failed to read left calibration file");
            }
            
            cv::FileStorage rightFile(nameIntrRight, cv::FileStorage::READ);
            if (rightFile.isOpened()) {
                intr = (CvMat*)rightFile["intrinsic"].readObj();
                dist = (CvMat*)rightFile["distortion"].readObj();
                cameraMatrix2 = intr;
                distCoeffs2 = dist;
            } else {
                throw runtime_error("failed to read left calibration file");
            }
        }
        CVPRINT(cameraMatrix1);
        CVPRINT(distCoeffs1);
        CVPRINT(cameraMatrix2);
        CVPRINT(distCoeffs2);
        fflush(stdout);
        std::vector < std::vector <cv::Point2f> > imagePoints1U = undistortPoints(imagePoints1, cameraMatrix1, distCoeffs1);
        std::vector < std::vector <cv::Point2f> > imagePoints2U = undistortPoints(imagePoints2, cameraMatrix2, distCoeffs2);

        if (imagePoints2.size() == imagePoints1.size()) {
            if (!strOut.empty()) {
                createFolder(strOut);
            }
            cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT +  cv::TermCriteria::EPS, 30, 1e-6);
            int flags = CV_CALIB_USE_INTRINSIC_GUESS; //cv::CALIB_FIX_INTRINSIC;
            if (!nameIntrLeft.empty() && !nameIntrRight.empty())
                flags += cv::CALIB_FIX_INTRINSIC;
            cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, E, F, term_crit, flags);
            CVPRINT(cameraMatrix1);
            CVPRINT(distCoeffs1);
            CVPRINT(cameraMatrix2);
            CVPRINT(distCoeffs2);
            CVPRINT(R);
            CVPRINT(T);
            CVPRINT(E);
            CVPRINT(F);
            cv::FileStorage fs;
            std::string strTargetFolder;
            if (strOut.empty()) {
                strTargetFolder = std::string("distorted");
            } else {
                strTargetFolder = strOut + std::string("/distorted");
            }
            createFolder(strTargetFolder);
//             fs.open(strTargetFolder + std::string("/cameraMatrix1.xml"), cv::FileStorage::WRITE);
//             cv::write ( fs, cameraMatrix1 );
//             fs.open(strTargetFolder + std::string("/distCoeffs1.xml"), cv::FileStorage::WRITE);
//             cv::write  ( fs, distCoeffs1 );
//             fs.open(strTargetFolder + std::string("/cameraMatrix2.xml"), cv::FileStorage::WRITE);
//             cv::write ( fs,  cameraMatrix2 );
//             fs.open(strTargetFolder + std::string("/distCoeffs2.xml"), cv::FileStorage::WRITE);
//             cv::write ( fs, distCoeffs2 );
//             fs.open(strTargetFolder + std::string("/R.xml"), cv::FileStorage::WRITE);
//             cv::write  ( fs,  R );
//             fs.open(strTargetFolder + std::string("/T.xml"), cv::FileStorage::WRITE);
//             cv::write  ( fs,  T );
//             fs.open(strTargetFolder + std::string("/E.xml"), cv::FileStorage::WRITE);
//             cv::write  ( fs,  E );
//             fs.open(strTargetFolder + std::string("/F.xml"), cv::FileStorage::WRITE);
//             cv::write  ( fs,  F );

            /// Added code by ARI
            flags= cv::CALIB_ZERO_DISPARITY;
            cv::Mat R1(3,3, CV_32F), R2(3,3, CV_32F);   // rotation matrices
            cv::Mat P1(3,4, CV_32F), P2(3,4, CV_32F);   // projection matrices
            cv::Mat Q(4,4, CV_32F);
            cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q, flags);

            // Image size as cv::Mat
            cv::Mat size = cv::Mat_<double>(2, 1);
            size.at<double>(0, 0) = imageSize.width;
            size.at<double>(1, 0) = imageSize.height;
            
            // Translation and rotation for left camera (==0)
            cv::Mat Tl = cv::Mat_<double>::zeros(3, 1);
            cv::Mat Rl = cv::Mat_<double>::zeros(3, 3);
            Rl.at<double>(0, 0) = 1.;
            Rl.at<double>(1, 1) = 1.;
            Rl.at<double>(2, 2) = 1.;

            // Recalculate translation vector from [mm] to [m]
            T.at<double>(0, 0) = T.at<double>(0, 0)/1000.;
            T.at<double>(1, 0) = T.at<double>(1, 0)/1000.;
            T.at<double>(2, 0) = T.at<double>(2, 0)/1000.;
            
            fs.open(strTargetFolder + std::string("/camcalib-left.xml"), cv::FileStorage::WRITE);
            cv::write ( fs, "imgsize", size );
            cv::write ( fs, "intrinsic", cameraMatrix1 );
            cv::write ( fs, "distortion", distCoeffs1 );
            cv::write ( fs, "projection", P1 );
            cv::write ( fs, "rotation", R1 );
            cv::write ( fs, "tvec", Tl );
            cv::write ( fs, "rmat", Rl );
            
            fs.open(strTargetFolder + std::string("/camcalib-right.xml"), cv::FileStorage::WRITE);
            cv::write ( fs, "imgsize", size );
            cv::write ( fs, "intrinsic", cameraMatrix2 );
            cv::write ( fs, "distortion", distCoeffs2 );
            cv::write ( fs, "projection", P2 );
            cv::write ( fs, "rotation", R2 );
            cv::write ( fs, "tvec", T );
            cv::write ( fs, "rmat", R );
            
            fs.open(strTargetFolder + std::string("/campose-left.xml"), cv::FileStorage::WRITE);
            cv::write ( fs, "tvec", Tl );
            cv::write ( fs, "rmat", Rl );
            
            fs.open(strTargetFolder + std::string("/campose-right.xml"), cv::FileStorage::WRITE);
            cv::write ( fs, "tvec", T );
            cv::write ( fs, "rmat", R );
            /// Added by ARI end
            
            {
                std::cout << "---------------------- undistortPoints ---------------------- \n";
                cameraMatrix1 = cv::Mat_<double>::zeros(3,3);
                distCoeffs1 = cv::Mat_<double>::zeros(1,5);
                cameraMatrix2 = cv::Mat_<double>::zeros(3,3);
                distCoeffs2 = cv::Mat_<double>::zeros(1,5);
                cv::calibrateCamera(objectPoints, imagePoints1U, imageSize, cameraMatrix1, distCoeffs1, rvecs1, tvecs1, 0);
                CVPRINT(cameraMatrix1);
                CVPRINT(distCoeffs1);
                cv::calibrateCamera(objectPoints, imagePoints2U, imageSize, cameraMatrix2, distCoeffs2, rvecs2, tvecs2, 0);
                CVPRINT(cameraMatrix2);
                CVPRINT(distCoeffs2);
                fflush(stdout);
                cv::TermCriteria term_crit = cv::TermCriteria(cv::TermCriteria::COUNT +  cv::TermCriteria::EPS, 30, 1e-6);
                int flags = CV_CALIB_USE_INTRINSIC_GUESS; //cv::CALIB_FIX_INTRINSIC;
                cv::stereoCalibrate(objectPoints, imagePoints1U, imagePoints2U, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, E, F, term_crit, flags);
                CVPRINT(cameraMatrix1);
                CVPRINT(distCoeffs1);
                CVPRINT(cameraMatrix2);
                CVPRINT(distCoeffs2);
                CVPRINT(R);
                CVPRINT(T);
                CVPRINT(E);
                CVPRINT(F);
                cv::FileStorage fs;
                std::string strTargetFolder;
                if (strOut.empty()) {
                    strTargetFolder = std::string("undistorted");
                } else {
                    strTargetFolder = strOut + std::string("/undistorted");
                }
                createFolder(strTargetFolder);
//                 fs.open(strTargetFolder + std::string("/cameraMatrix1.xml"), cv::FileStorage::WRITE);
//                 cv::write ( fs, cameraMatrix1 );
//                 fs.open(strTargetFolder + std::string("/distCoeffs1.xml"), cv::FileStorage::WRITE);
//                 cv::write  ( fs, distCoeffs1 );
//                 fs.open(strTargetFolder + std::string("/cameraMatrix2.xml"), cv::FileStorage::WRITE);
//                 cv::write ( fs,  cameraMatrix2 );
//                 fs.open(strTargetFolder + std::string("/distCoeffs2.xml"), cv::FileStorage::WRITE);
//                 cv::write ( fs, distCoeffs2 );
//                 fs.open(strTargetFolder + std::string("/R.xml"), cv::FileStorage::WRITE);
//                 cv::write  ( fs,  R );
//                 fs.open(strTargetFolder + std::string("/T.xml"), cv::FileStorage::WRITE);
//                 cv::write  ( fs,  T );
//                 fs.open(strTargetFolder + std::string("/E.xml"), cv::FileStorage::WRITE);
//                 cv::write  ( fs,  E );
//                 fs.open(strTargetFolder + std::string("/F.xml"), cv::FileStorage::WRITE);
//                 cv::write  ( fs,  F );
                
                /// Added code by ARI
                flags= cv::CALIB_ZERO_DISPARITY;
                cv::stereoRectify(cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, R1, R2, P1, P2, Q, flags);
                
                // Recalculate translation vector from [mm] to [m]
                T.at<double>(0, 0) = T.at<double>(0, 0)/1000.;
                T.at<double>(1, 0) = T.at<double>(1, 0)/1000.;
                T.at<double>(2, 0) = T.at<double>(2, 0)/1000.;
                
                fs.open(strTargetFolder + std::string("/camcalib-left.xml"), cv::FileStorage::WRITE);
                cv::write ( fs, "imgsize", size );
                cv::write ( fs, "intrinsic", cameraMatrix1 );
                cv::write ( fs, "distortion", distCoeffs1 );
                cv::write ( fs, "projection", P1 );
                cv::write ( fs, "rotation", R1 );
                cv::write ( fs, "tvec", Tl );
                cv::write ( fs, "rmat", Rl );
                
                fs.open(strTargetFolder + std::string("/camcalib-right.xml"), cv::FileStorage::WRITE);
                cv::write ( fs, "imgsize", size );
                cv::write ( fs, "intrinsic", cameraMatrix2 );
                cv::write ( fs, "distortion", distCoeffs2 );
                cv::write ( fs, "projection", P2 );
                cv::write ( fs, "rotation", R2 );
                cv::write ( fs, "tvec", T );
                cv::write ( fs, "rmat", R );
                
                fs.open(strTargetFolder + std::string("/campose-left.xml"), cv::FileStorage::WRITE);
                cv::write ( fs, "tvec", Tl );
                cv::write ( fs, "rmat", Rl );
                
                fs.open(strTargetFolder + std::string("/campose-right.xml"), cv::FileStorage::WRITE);
                cv::write ( fs, "tvec", T );
                cv::write ( fs, "rmat", R );
                /// Added by ARI end
                
                cv::StereoBM stereo; //(0, 100, 3);
                stereo(imgA, imgB, imgDM);
                cv::imshow("DM", imgDM);
                cv::waitKey(10000);
            }
        }

    }
    return 0;

}
// kate: indent-mode cstyle; space-indent on; indent-width 0;

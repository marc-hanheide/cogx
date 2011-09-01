/**
 * @file main
 * @author Johann Prankl
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/

#include <vector>
#include <time.h>
#include <iostream>
#include <fstream>
#include <iostream>
#include <string>
#include <dirent.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "v4r/EllCalib/EllPatternDetector.hh"
#include "v4r/EllCalib/EllCalibrate.hh"


#define DEBUG

using namespace std;


void help();


/******************************** main *********************************/
int main(int argc, char** argv)
{
    P::EllCalibrate::Parameter param; 

    printf ( "usage: %s directory xEllipseDistance yEllipseDistance \n", argv[0] );
    printf ( "  directory: A directory only with images for camera calibration!");
    printf ( "  xEllipseDistance: Distance between ellipses in x direction");
    printf ( "  yEllipseDistance: Distance between ellipses in y direction");

    if (argc != 2 && argc !=4)
    {
      cout<<"Wrong parameters!"<<endl;
      return 0;
    }

    if (argc == 4)
    {
      param.xDist = atof(argv[2]);
      param.yDist = atof(argv[3]);
    }

    cv::Mat image, dbg, gray;
    cv::Mat intrinsic, distCoeffs, R, T;
    vector<cv::Mat> rvecs, tvecs;

    P::EllCalibrate calib(param);
    P::EllPatternDetector detector;

    cv::namedWindow("Image");

    DIR *dp;
    dp = opendir( argv[1] );
    struct dirent *dirp;
    struct stat filestat;
    string filepath;

    if (dp == NULL)
    {
      cout << "Error opening " << argv[1] << endl;
      return 0;
    }

    while ((dirp = readdir( dp )))
    {
        filepath = string(argv[1]) + "/" + dirp->d_name;

        if (stat( filepath.c_str(), &filestat )) continue;
        if (S_ISDIR( filestat.st_mode ))         continue;

        image = cv::imread(filepath, 1);

        if( image.empty() ) continue;
      
        cv::cvtColor(image, gray, CV_BGR2GRAY);
        image.copyTo(dbg);

        if (calib.Add(gray))
        {
          detector.Draw(dbg,*calib.patterns.back());
        }

        cv::imshow("Image",dbg); 
        cv::waitKey(100);
    }

    closedir( dp );

    //calibrate ...
    {
      cout<<"Calibrate...."<<endl;

      calib.Calibrate(intrinsic, distCoeffs, rvecs, tvecs, 0);

      cout<<"Camera parameter: "<<endl;
      cout<<"Intrinsic: "<<endl<<intrinsic<<endl;
      cout<<"Distortion: "<<endl<<distCoeffs<<endl;
 
      cv::FileStorage fs("intrinsic.yml", cv::FileStorage::WRITE);
      fs<<"intrinsic"<<intrinsic;
      fs.release();
      fs.open("distCoeffs.yml",cv::FileStorage::WRITE);
      fs<<"distCoeffs"<<distCoeffs;
      fs.release();     

      calib.GetPose(gray, intrinsic, distCoeffs, R, T);
      cout<<"Camera pose: "<<endl<<R<<endl<<T<<endl;

      fs.open("pose.yml",cv::FileStorage::WRITE);
      fs<<"R"<<R<<"T"<<T;
      fs.release();
    }

    cv::destroyWindow("Image");
    return 0;
}


/**************************** some (test) methods *************************/














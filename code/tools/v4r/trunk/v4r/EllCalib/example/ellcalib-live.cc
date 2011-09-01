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
#include <limits.h>
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
    cv::VideoCapture cap;

    printf ( "usage: %s cam_id xEllipseDistance yEllipseDistance \n", argv[0] );

    if( argc == 1)
      cap.open(0);
    else if( argc == 2 )
      cap.open(argv[1]);
    else if (argc == 4)
    {
      cap.open(argv[1]);
      param.xDist = atof(argv[2]);
      param.yDist = atof(argv[3]);
    }

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

    int mode=-1;
    cv::Mat image, dbg, gray;
    cv::Mat intrinsic, distCoeffs, R, T;
    vector<cv::Mat> rvecs, tvecs;

    P::EllCalibrate calib(param);
    P::EllPatternDetector detector;

    cv::namedWindow("Image");

    cap >> image;

    cout<<"Press:"<<endl;
    cout<<"  'd' .. to start/stop pattern detection"<<endl;
    cout<<"  'c' .. to calibrate the camera"<<endl;

    cv::imshow("Image",image); 
    char c = (char)cv::waitKey(0);


    for (;;) 
    {
        if( c == 27 )
            break;
        if( c == 'd' && mode==-1) mode = 1;
        else if (c == 'd' && mode==1) mode = -1;
        else if (c == 'c')
        {
          mode = 2;
          break;
        }

        cap >> image;

        if( image.empty() )
            continue;
      
        cv::cvtColor(image, gray, CV_BGR2GRAY);
        image.copyTo(dbg);

        if (mode==1)
        {
          if (calib.Add(gray))
          {
            detector.Draw(dbg,*calib.patterns.back());
          }
        }

        cv::imshow("Image",dbg); 
        c = (char)cv::waitKey(100);
    }


    //calibrate ...
    if (mode == 2 )
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














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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "v4r/elldetect/SMath.h"
#include "v4r/EllCalib/EllPatternDetector.hh"
#include "v4r/EllCalib/EllCalibrate.hh"


#define DEBUG

using namespace std;


void help();


/******************************** main *********************************/
int main(int argc, char** argv)
{
    cv::VideoCapture cap;

    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        cap.open(argc == 2 ? argv[1][0] - '0' : 0);
    else if( argc == 2 )
        cap.open(argv[1]);

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

    cv::namedWindow("Image");

    cout<<"Press:"<<endl;
    cout<<"  'a' .. to add detected patterns to a container"<<endl;
    cout<<"  'c' .. to calibrate the camera"<<endl;
    cout<<"  'p' .. to detecte the pose of the camera (only after calibration)"<<endl;
    cout<<"Press a key to start!"<<endl;
    cv::waitKey(0);


    int mode=-1;
    cv::Mat image, dbg, gray;
    cv::Mat intrinsic, distCoeffs, R, T;
    vector<cv::Mat> rvecs, tvecs;
    vector<cv::Ptr<P::EllPattern> > patterns;

    P::EllCalibrate calib;
    P::EllPatternDetector detector;


    for (;;) 
    {
    

        char c = (char)cv::waitKey(50);
        if( c == 27 )
            break;
        if( c == 's') mode = -1;
        else if (c == 'a') mode = 0;
        else if (c == 'c') mode = 1;
        else if (c == 'p') mode = 2;

        cap >> image;

        if( image.empty() )
            continue;
      
        cv::cvtColor(image, gray, CV_BGR2GRAY);
        image.copyTo(dbg);

        if (mode==0)
        {
          calib.dbg = dbg;
          calib.Add(gray);
        }
        else if (mode == 1 )
        {
          cout<<"Calibrate...."<<endl;

          calib.Calibrate(intrinsic, distCoeffs, rvecs, tvecs, 0);

          cout<<"Camera parameter: "<<endl;
          cout<<intrinsic<<distCoeffs<<endl;
          cv::waitKey(0);
          mode = -1;
        }
        else if (mode == 2 && !intrinsic.empty() && !distCoeffs.empty())
        {
          calib.GetPose(gray, intrinsic, distCoeffs, R, T);
          cout<<"Camera pose: "<<R<<T<<endl;
        }
        else if (mode==-1)
        {
          detector.dbg = dbg;
          detector.Detect(gray,patterns);
        }

        cv::imshow("Image",dbg); 
    }


    cv::destroyWindow("Image");
    return 0;
}


/**************************** some (test) methods *************************/

void help()
{
  cout <<
      "\nCamera calibration with ellipses\n"
      "Call:\n"
      "./ellCalib [image_name]\n" << endl;

}












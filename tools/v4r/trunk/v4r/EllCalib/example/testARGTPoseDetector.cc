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
#include "v4r/EllCalib/arGTPatternDetector.hh"
#include "v4r/EllCalib/arDetectGTPose.hh"


#define DEBUG

using namespace std;


void help();


/******************************** main *********************************/
int main(int argc, char** argv)
{
    P::arDetectGTPose::Parameter param1(9, 15,15, -5, 0, -1, 7, -4, 4, 10.);
    P::arDetectGTPose::Parameter param2(9, 15,15, 4 ,0, -6, 2, -4, 4, 20.);
    
    cv::VideoCapture cap;

    printf ( "usage: %s cam_id xEllipseDistance yEllipseDistance \n", argv[0] );

    if( argc == 1)
      cap.open(0);
    else if( argc == 2 )
      cap.open(argv[1]);
    else if (argc == 4)
    {
      cap.open(argv[1]);
      param1.xDist = atof(argv[2]);
      param1.yDist = atof(argv[3]);
      param2.xDist = atof(argv[2]);
      param2.yDist = atof(argv[3]);
    }

    if( !cap.isOpened() )
    {
        cout << "Could not initialize capturing...\n";
        return 0;
    }

    cv::Mat image, dbg, gray;
    cv::Mat_<double> intrinsic, distCoeffs, R, T;
    P::EllPattern pattern;

    intrinsic = cv::Mat(3,3,CV_64F);
    intrinsic(0,0) = 521.1082832157917; intrinsic(0,1) = 0; intrinsic(0,2) = 319.8730653241794;
    intrinsic(1,0) = 0; intrinsic(1,1) = 519.3812016050053; intrinsic(1,2) = 238.5306749361964;
    intrinsic(2,0) = 0; intrinsic(2,1) = 0; intrinsic(2,2) = 1;
    distCoeffs = cv::Mat::zeros(1,4,CV_64F);

    P::arDetectGTPose detector("/home/hannes/XWorks/v4r/v4r/EllCalib/pattern/1.pat","/home/hannes/XWorks/v4r/v4r/EllCalib/pattern/2.pat",param1,param2);

    cv::namedWindow("Image");

    cap >> image;

    cout<<"Show the pattern to detect the groundtruth pose and press a key!"<<endl;

    cv::imshow("Image",image); 
    char c = (char)cv::waitKey(0);
   

    for (;;) 
    {
        if( c == 27 )
            break;

        cap >> image;

        if( image.empty() )
            continue;
      
        cv::cvtColor(image, gray, CV_BGR2GRAY);
        image.copyTo(dbg);

        detector.dbg = dbg;
        if(detector.GetPose(image, intrinsic, distCoeffs, R, T, pattern))
        {
          cout<<"Pose: "<<endl;
          cout<<" "<<R<<endl;
          cout<<" "<<T<<endl;
        }
        

        cv::imshow("Image",dbg); 
        c = (char)cv::waitKey(100);
    }

    cv::destroyWindow("Image");
    return 0;
}


/**************************** some (test) methods *************************/














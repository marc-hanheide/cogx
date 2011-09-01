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

//#define LOG_IMAGE_NAME "../log/result.jpg"

#define DEBUG

using namespace std;


void help();


/******************************** main *********************************/
int main(int argc, char** argv)
{
    cv::Mat image, gray, dbg;
    vector<cv::Ptr<P::EllPattern> > patterns;

    P::EllPatternDetector patternDetector;

    // get the image
    char* filename = argc == 2 ? argv[1] : (char*)"test.jpg";

    image = cv::imread(filename, 1);
    if(image.empty())
    {
      help();
      return -1;
    }

    cv::cvtColor(image, gray, CV_BGR2GRAY);
    image.copyTo(dbg);

    // Create a window
    cv::namedWindow("Image", 1);


    // ---------------- do the hard work ----------------------
    struct timespec start1, end1;
    clock_gettime(CLOCK_REALTIME, &start1);

    patternDetector.dbg = dbg;
    patternDetector.Detect(gray, patterns);

    clock_gettime(CLOCK_REALTIME, &end1);
    cout<<"Time [s]: "<<RTE::timespec_diff(&end1, &start1)<<endl;    
    // ----------------- wow it's done ------------------------

    cv::imshow("Image", dbg);

    // save debug image
    #ifdef LOG_IMAGE_NAME
    cv::imwrite(LOG_IMAGE_NAME,dbg);
    #endif

    cv::waitKey(0);

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












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
#include <PNamespace.hh>


using namespace P;

int thresh = 5;
cv::Mat image, gray, thrGray, thrCol;
cv::Mat dist,distlap;
int distType = CV_DIST_L2;
int maskSize = CV_DIST_MASK_PRECISE;

void help();
void onTrackbar(int, void*);


/******************************** main *********************************/
int main(int argc, char** argv)
{
    char* filename = argc == 2 ? argv[1] : (char*)"fruits.jpg";

    image = cv::imread(filename, 1);
    if(image.empty())
    {
      help();
      return -1;
    }
    help();

    thrGray.create(image.size(), image.type());
    thrCol.create(image.size(), image.type());
    cv::cvtColor(image, gray, CV_BGR2GRAY);

    // Create a window
    cv::namedWindow("Segmentation", 1);
    cv::namedWindow("Image", 1);


    // create a toolbar
    cv::createTrackbar("Threshold", "Segmentation", &thresh, 255, onTrackbar);

    // Show the image
    cv::imshow("Image", image);
    onTrackbar(0, 0);

    // Wait for a key stroke; the same function arranges events processing
    cv::waitKey(0);

    return 0;
}


// define a trackbar callback
void onTrackbar(int, void*)
{
    //cv::blur(gray, thrGray, cv::Size(3,3));

    //cv::threshold(thrGray, thrGray, thresh, 255, cv::THRESH_BINARY_INV);
    cv::adaptiveThreshold(gray,thrGray, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV, 9, thresh);

    cv::distanceTransform( thrGray, dist, distType, maskSize );

    // Run the edge detector on grayscale
    //cv::Canny(thrGray, thrGray, thresh, thresh*3, 3);
    //thrCol = cv::Scalar::all(0);

    image.copyTo(thrCol, thrGray);
    cv::imshow("Segmentation", thrGray);

    //show skeleton
    /*cv::Mat dist32s, dist8u, dist8u1, dist8u2;
    dist*=-50;
    dist+=255;

    cv::Laplacian(dist,distlap, CV_32F, 1, 1, 0, cv::BORDER_DEFAULT);

    distlap.convertTo(dist8u,CV_8U);
    //cv::threshold(dist8u,dist8u,0,255,cv::THRESH_BINARY);

    cv::imshow("Segmentation", dist8u);*/
}


void help()
{
  cout <<
      "\nSegmentation\n"
      "Call:\n"
      "/.segm [image_name]\n" << endl;

}












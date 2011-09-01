#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <v4r/canny/canny.h>



int main(int argc, char *argv[]) {
    if ( argc != 2 ) {
        printf ( "usage: %s <image>\n", argv[0] );
        return 1;
    }
    printf ( "%s\n", argv[1] );

    cv::Mat imgGray = cv::imread(argv[1], 0);
    cv::Mat imgGauss, imgCanny, imgGradient, imgDirection;
    cv::namedWindow("img",1);
    cv::namedWindow("canny",1);    
    cv::namedWindow("gradient",1);    
    cv::namedWindow("direction",1);    
    do {
        cv::GaussianBlur(imgGray, imgGauss, cv::Size(7,7), 1.5, 1.5);
        V4R::Canny(imgGauss, imgCanny, imgGradient, imgDirection, 0, 30, 3);
        cv::imshow("img", imgGray);
        cv::imshow("canny", imgCanny);
        cv::imshow("gradient", (imgGradient*65335/360));
        cv::imshow("direction", (imgDirection*65335/360));
    } while (cv::waitKey(30) < 0) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}


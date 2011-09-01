#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <v4r/colors/colors.h>



int main(int argc, char *argv[]) {
    if ( argc != 2 ) {
        printf ( "usage: %s <image>\n", argv[0] );
        return 1;
    }
    printf ( "%s\n", argv[1] );

    cv::Mat imgBGR = cv::imread(argv[1]);
    cv::Mat imgYUV422, imgDES;
    cv::namedWindow("YUV422",1);   
    cv::namedWindow("DES",1);  
    cv::namedWindow("BGR",1);    
    V4R::Colors colors;
    do {
        colors.BGR24toYUV422(imgBGR, imgYUV422);
        colors.YUV422toBGR24(imgYUV422, imgDES);
        cv::imshow("BGR", imgBGR);
	//cv::imshow("YUV422", imgYUV422);
        cv::imshow("DES", imgDES);
    } while (cv::waitKey(30) < 0) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}


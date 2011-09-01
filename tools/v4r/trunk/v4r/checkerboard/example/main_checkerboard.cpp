#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <v4r/checkerboard/checkerboard.h>
#include <boost/concept_check.hpp>



int main(int argc, char *argv[]) {
    if ( argc != 2 ) {
        printf ( "usage: %s <image>\n", argv[0] );
        return 1;
    }
    printf ( "%s\n", argv[1] );

    cv::Mat imgGray = cv::imread(argv[1], 0);
    V4R::Checkerboard board;
    cv::Mat_<double> cameraMat = cv::Mat_<double>::eye(3,3);
    cameraMat(0,0) = 300; //fx
    cameraMat(0,2) = imgGray.cols/2; //cx
    cameraMat(1,1) = cameraMat(0,0); //fy
    cameraMat(2,2) = imgGray.cols/2; //cy
    cv::Mat_<double> distCoeffs = cv::Mat_<double>::zeros(4,1);

    do {
	board.init(cv::Size(6,8), cv::Size_<double>(0.028,0.028), true);
	cv::Mat_<double> R, T;
	board.find(imgGray,cameraMat, distCoeffs, R, T);
	board.drawBoard(imgGray);
	board.drawSystem(imgGray,cameraMat, distCoeffs, R, T);
        cv::imshow("imgGray", imgGray);
    } while (cv::waitKey(30) < 0) ;
    return 0;
}


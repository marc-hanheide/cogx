#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <stdio.h>



int main ( int argc, char** argv ) {
    if ( argc < 4 ) {
        printf ( "usage: %s <source file> <destimation file> <format> [default = bg2rgb]\n", argv[0]);
        printf ( " - format: bg2bgr, gb2BGR, rg2bgr, gr2bgr, bg2rgb, gb2BGR, rg2bgr, gr2bgr\n");
        printf ( " ---> option not yet impemented\n");
        if (( argc < 3 ) || ( argc > 4 )) {
            return 1;
        }
    }
    printf ( "source %s\n, destimation %s\n, ", argv[1], argv[2] );
    cv::Mat imageSrc = cv::imread(argv[1], 0 );
    cv::Mat imageBy = imageSrc;
    cv::Mat imageColor(imageSrc.cols, imageSrc.rows, CV_8UC3);
    cv::cvtColor(imageBy, imageColor, CV_BayerGB2RGB);
    cv::imwrite(argv[2], imageColor);
    cv::namedWindow ( "Image", 1 );
    do {
      cv::imshow("Image", imageColor);
    } while (cv::waitKey(10000) == -1);
    return 0;
}

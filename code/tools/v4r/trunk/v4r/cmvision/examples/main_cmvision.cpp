#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <v4r/colors/colors.h>
#include <v4r/cmvision/segmentation.h>

#define PIXEL_FORMAT V4L2_PIX_FMT_YUYV


int main(int argc, char *argv[]) {
    if ( argc != 3 ) {
        printf ( "usage: %s <image> <colors>\n", argv[0] );
        return 1;
    }
    printf ( "%s\n", argv[1] );

    cv::Mat imgSrc = cv::imread(argv[1], 1);
    cv::Mat imgYUV, imgDes;
    V4R::Colors colors;
    colors.BGR24toYUV422(imgSrc, imgYUV);
    colors.YUV422toBGR24(imgYUV, imgDes);
    cv::namedWindow("img",1);
    V4R::CMUSegmentation segmentation;

    segmentation.loadColors("/home/max/projects/v4r/v4r/cmvision/examples/colors.txt");


    V4R::CMUSegmentation::Colors c;
    c.setColor(255,0,0, "goal");
    c.setRange(67,175,140,180,70,100);
    c.setOption(0.5,2);
    segmentation.addColor(c);

    do {
        cv::imshow("img", imgDes);
        segmentation.perform(imgYUV);
        for (unsigned int c = 0; c < segmentation.getRegions().size(); c++) {
            for (unsigned int i = 0; i < segmentation.getRegions()[c].size(); i++) {
                cv::Point2i pt1(segmentation.getRegions()[c][i].bounding_box.x, segmentation.getRegions()[c][i].bounding_box.y);
                cv::Point2i pt2(segmentation.getRegions()[c][i].bounding_box.x+pt1.x, segmentation.getRegions()[c][i].bounding_box.y+pt1.y);
                cv::rectangle(imgDes, pt1, pt2, cv::Scalar(255,255,0));
            }
            printf ( "color: %i count %i\n", c,  segmentation.getRegions()[c].size());
        }
    } while (cv::waitKey(30) < 0) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}


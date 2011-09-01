#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <v4r/edgelinker/contour.h>
#include <v4r/linedetector/linedetector.h>



int main(int argc, char *argv[]) {
    if ( argc != 2 ) {
        printf ( "usage: %s <image>\n", argv[0] );
        return 1;
    }
    printf ( "%s\n", argv[1] );

    cv::Mat imgGray = cv::imread(argv[1], 0);
    cv::Mat imgGauss, imgCanny, imgLinkedEdges(imgGray.rows, imgGray.cols, CV_8UC3), imgLines(imgGray.rows, imgGray.cols, CV_8UC3);
    cv::namedWindow("img",1);
    cv::namedWindow("canny",1);
    cv::namedWindow("linked_edges",1);
    cv::namedWindow("lines",1);
    V4R::Contour contour;
    V4R::LineDetector linedetector;
    linedetector.init(20, 0, 0.5, 1.0);
    contour.Init(imgGray.cols, imgGray.rows);


    do {
        cv::GaussianBlur(imgGray, imgGauss, cv::Size(7,7), 1.5, 1.5);
        cv::Canny(imgGauss, imgCanny, 0, 30, 3);
        contour.Perform(imgCanny.data, V4R::Contour::MODE_CONTOUR);
        std::vector<cv::Point2i> edges;
        contour.GetEdgeListSplittedXY(edges) ;
        contour.Draw(imgLinkedEdges.data);
        std::vector<cv::Range> idxLines = linedetector.Perform ( edges, contour.getSegmentIndexes() );
	imgLines.setTo(0);
        for ( unsigned int i = 0; i < idxLines.size(); i++ ) {
            cv::Point_<int> A, B;
            cv::Scalar colorLine = CV_RGB ( rand() &255, rand() &255, rand() &255 );
            A = edges [idxLines[i].start ];
            B = edges [idxLines[i].end ];
            cv::line (imgLines, A, B, colorLine );
        }

        cv::imshow("img", imgGray);
        cv::imshow("canny", imgCanny);
        cv::imshow("linked_edges", imgLinkedEdges);
        cv::imshow("lines", imgLines);
    } while (cv::waitKey(30) < 0) ;
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}


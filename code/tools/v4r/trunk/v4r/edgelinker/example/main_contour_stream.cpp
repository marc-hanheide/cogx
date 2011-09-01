#include <cstdio>
#include <stdlib.h>
#include <iostream>
#include <string>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <v4r/edgelinker/contour.h>
#include <v4r/utils/filesystemhdl.h>



int main(int argc, char *argv[]) {
    if ( argc != 3 ) {
        printf ( "usage: %s <folder> <fileending>\n", argv[0] );
        return 1;
    }
    cv::namedWindow("img",1);
    cv::namedWindow("canny",1);
    cv::namedWindow("linked_edges",1);

    std::vector<std::string> files;
    V4R::Contour el;
    V4R::FS::getFilesInFolder (argv[1], files, std::string("(.*)") + std::string(argv[2]));
    for (unsigned int idxFile = 0; idxFile < files.size(); idxFile++) {
        cv::Mat imgGray = cv::imread(files[idxFile], 0);
        cv::Mat imgGauss, imgCanny, imgLinkedEdges(imgGray.rows, imgGray.cols, CV_8UC3);
	el.Init(imgGray.cols, imgGray.rows);

        cv::GaussianBlur(imgGray, imgGauss, cv::Size(7,7), 1.5, 1.5);
        cv::Canny(imgGauss, imgCanny, 0, 30, 3);
        el.Perform(imgCanny.data, V4R::Contour::MODE_CONTOUR);
        el.Perform(imgCanny.data, V4R::Contour::MODE_SIMPLE);
        el.Draw(imgLinkedEdges.data);
        cv::imshow("img", imgGray);
        cv::imshow("canny", imgCanny);
        cv::imshow("linked_edges", imgLinkedEdges);
        if ( idxFile ==  files.size() -1) idxFile = 0;
	int key = cv::waitKey(100);
        if (key > 0) {
            idxFile = files.size();
        }
    }
    return 0;
}


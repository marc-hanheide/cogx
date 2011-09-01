/**
 * @file edgelinker
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief
 *
 * @see
 **/
#include "edgelinker.h"
#include <iostream>
#include <stack>
#include <cstdio>
#include "opencv/highgui.h"

namespace V4R {

EdgeLinker::EdgeLinker()  {
    mEdges.push_back(cv::Point2i(1024/2,768/2));
    mSegments.push_back(cv::Range(0,0));
    mDirectionLookup[0] = cv::Point2i(  0, +1);
    mDirectionLookup[1] = cv::Point2i( -1, +1);
    mDirectionLookup[2] = cv::Point2i( -1,  0);
    mDirectionLookup[3] = cv::Point2i( -1, -1);
    mDirectionLookup[4] = cv::Point2i(  0, -1);
    mDirectionLookup[5] = cv::Point2i( +1, -1);
    mDirectionLookup[6] = cv::Point2i( +1,  0);
    mDirectionLookup[7] = cv::Point2i( +1, +1);
}

EdgeLinker::~EdgeLinker() {
}
/*
 * 3 4 5
 * 2   6
 * 1 0 7
 */
void EdgeLinker::trace (int r, int c, int last, cv::Range &segment) {
    mImgEdge(r, c) = 0;
    mEdges.push_back(cv::Point(c,r));
    segment.end++;

    int vote[] = {0, 0, 0, 0, 0, 0, 0, 0};
    int d = (mImgDirections(r,c) / 45) % 8;
    vote[d]++;
    vote[(d+4)%8]++;
    vote[last]=0;
    for ( int i = 0; i < 8; i++) {
        if (mImgEdge( mDirectionLookup[i].y + r, mDirectionLookup[i].x + c)) {
            vote[i]++;
        } else {
            vote[i] = 0;
        }
    }
    int best = last;
    for ( int i = 0; i < 8; i++) {
        if (vote[i] > vote[best]) best = i;
    }
    if (vote[best] > 0) {
        last = (last-4) % 8;
        r = mDirectionLookup[best].y + r;
        c = mDirectionLookup[best].x + c;
        trace (r, c, last, segment);
    }
}

void EdgeLinker::clearBoarders() {
    for (int r = 0; r < mImgEdge.rows; r++) {
        mImgEdge(r,0) = 0;
        mImgEdge(r,mImgEdge.cols-1) = 0;
    }
    for (int c = 0; c < mImgEdge.cols; c++) {
        mImgEdge(0,c) = 0;
        mImgEdge(mImgEdge.rows-1,c) = 0;
    }
}

void EdgeLinker::Perform ( const cv::Mat &rEdges, const cv::Mat &rGradients, const cv::Mat &rDirections) {
    rEdges.copyTo(mImgEdge);
    clearBoarders();
    mImgGradients = rGradients;
    mImgDirections = rDirections;

    // ToDo only process area of interest
    for (cv::Point2i edge(1,1); edge.y < mImgEdge.rows-1;  edge.y++ ) {
        for (edge.x = 0; edge.x < mImgEdge.cols-1;  edge.x++ ) {
            if (mImgEdge(edge) > 0) {
                mSegments.push_back(cv::Range(mEdges.size(), mEdges.size()-1));
                trace(edge.y, edge.x, 2, mSegments.back());
            }
        }
    }
}

void EdgeLinker::Draw (cv::Mat &rImg ) {
    rImg.create(mImgEdge.size(), CV_8UC3);
    rImg.setTo(0);
    for ( unsigned int i = 0; i < mSegments.size(); i++ ) {
        unsigned char pColor[] = {rand() / ( RAND_MAX/0xFF ), rand() / ( RAND_MAX/0xFF ), rand() / ( RAND_MAX/0xFF ) };
        for ( int idxEdge = mSegments[i].start; idxEdge <= mSegments[i].end; idxEdge++ ) {
            cv::Point2i edge = (mEdges)[idxEdge];
            unsigned char *p = &rImg.data[3 * ( rImg.cols * edge.y + edge.x )];
            p[0] = pColor[0],p[1] = pColor[1],p[2] = pColor[2];
        }
    }
}


} //namespace V4R

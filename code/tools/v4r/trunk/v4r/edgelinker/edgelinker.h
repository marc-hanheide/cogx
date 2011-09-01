/**
 * @file edgelinker
 * @author Markus Bader
 * @date Mo Jul 9 2007
 * @version 0.1
 * @brief Includes the header file for a class to link detected image edges
 *
 * @see
 **/
#ifndef EDGELINKER_H
#define EDGELINKER_H

#include <vector>
#include <valarray>

#include <opencv/cv.h>

namespace V4R {


/**
* @brief Class to link detected images edges a typical source can be a cvCanny image
**/
class EdgeLinker {
public:

    EdgeLinker();
    ~EdgeLinker();

    void Perform ( const cv::Mat &rEdges,  const cv::Mat &rGradients, const cv::Mat &rDirections);
    void Draw ( cv::Mat &rRGB);

private:
    void trace(int r, int c, int last, cv::Range &segment);
    void clearBoarders();

    cv::Mat_<uchar> mImgEdge;
    cv::Mat_<unsigned short> mImgGradients;
    cv::Mat_<unsigned short> mImgDirections;
    std::vector<cv::Point2i> mEdges;
    std::vector<unsigned short> mGradients;
    std::vector<unsigned short> mDirections;
    std::vector<cv::Range> mSegments;
    cv::Point2i mDirectionLookup[8];

};

}

#endif //CONTOUR_H
// kate: indent-mode cstyle; space-indent on; indent-width 4;

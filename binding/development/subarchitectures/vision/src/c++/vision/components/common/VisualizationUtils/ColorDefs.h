/** @file ColorDefs.h
 *  @brief OpenCV's color codes.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _COLOR_DEF_H_
#define _COLOR_DEF_H_

#include <opencv/cv.h>


namespace Color {
static const CvScalar c_cyan = CV_RGB(0,255,255);
static const CvScalar c_yellow = CV_RGB(255,215,0);
static const CvScalar c_goldenRod = CV_RGB(218,165,32);
static const CvScalar c_salmon = CV_RGB(250,128,114);
static const CvScalar c_orange = CV_RGB(255,165,0);
static const CvScalar c_red = CV_RGB(255,0,0);
static const CvScalar c_magenta = CV_RGB(0,255,255);
static const CvScalar c_darkViolet = CV_RGB(148,0,211);
static const CvScalar c_blue = CV_RGB(0,0,255);
static const CvScalar c_steelBlue = CV_RGB(70,130,180);
static const CvScalar c_green = CV_RGB(0,255,0);
static const CvScalar c_darkGreen = CV_RGB(0,100,0);
static const CvScalar c_limeGreen = CV_RGB(50,205,50);
}

#endif 

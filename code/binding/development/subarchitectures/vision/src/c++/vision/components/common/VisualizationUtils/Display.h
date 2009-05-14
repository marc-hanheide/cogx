/** @file Display.h
 *  @brief Functions for drawing on an image.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#ifndef _DISPLAY_H
#define _DISPLAY_H

#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>

namespace Display {
    enum DrawMode { Draw2D, Draw3D };
    
    CvScalar getRandomColor(int objIdx);
    void drawCvRect(IplImage *img,CvRect rect, CvScalar color=CV_RGB(0,255,255));
}


#endif


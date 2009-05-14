/** @file Display.cpp
 *  @brief Functions for drawing on an image.
 *
 *
 *  @author Somboon Hongeng
 *  @bug No known bugs.
 */
#include "Display.h"

CvScalar Display::getRandomColor(int num) {
    CvScalar col;
    
    switch (num) {
	case 0: 
	    col= CV_RGB(255,0,0); break; // red
	case 1: 
      col= CV_RGB(0,255,0); break; //green
	case 2: 
	    col= CV_RGB(0,0,255); break; //blue
	default: 
	    col = CV_RGB(0,255,0); break;
    }
    return col;  
}

void Display::drawCvRect(IplImage *img,CvRect rect, CvScalar color) {
    CvPoint pts[4];
    pts[0].x = rect.x; pts[0].y = rect.y;
    pts[1].x = rect.x + rect.width; pts[1].y = rect.y;
    pts[2].x = rect.x + rect.width; pts[2].y = rect.y + rect.height;
    pts[3].x = rect.x; pts[3].y = rect.y + rect.height;
    
    cvLine(img, pts[0], pts[1], color, 2);
    cvLine(img, pts[1], pts[2], color, 2);
    cvLine(img, pts[2], pts[3], color, 2);
    cvLine(img, pts[3], pts[0], color, 2);
}


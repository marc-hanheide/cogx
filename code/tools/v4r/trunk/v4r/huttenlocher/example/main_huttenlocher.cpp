/*
Copyright (C) 2006 Pedro Felzenszwalb

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
*/

#include <cstdio>
#include <cstdlib>
#include "opencv/cv.h"
#include "opencv/highgui.h"

#include <hl.h>

int main ( int argc, char **argv ) {


    if ( ( argc != 2 ) && ( argc != 5 ) ) {
        fprintf ( stderr, "usage: %s image sigma k min \n", argv[0] );
        return 1;
    }
    
    char *pFile = argv[1];
    float sigma = 2;
    float k = 500;
    int min_size = 20;

    if ( argc > 2 ) {
        sigma = atof ( argv[2] );
    }
    if ( argc > 3 ) {
        k = atof ( argv[3] );
    }
    if ( argc > 4 ) {
        min_size = atoi ( argv[4] );
    }



    IplImage *pImgSrc = cvLoadImage ( pFile, CV_LOAD_IMAGE_UNCHANGED );
    //IplImage *pImgSrc = cvLoadImage ( pFile, CV_LOAD_IMAGE_GRAYSCALE );

    IplImage *pImgHl = cvCreateImage(cvSize(pImgSrc->width, pImgSrc->height), pImgSrc->depth, 3);
    IplImage *pImgSmooth = cvCreateImage(cvSize(pImgSrc->width, pImgSrc->height), pImgSrc->depth, pImgSrc->nChannels);
		V4R::Huttenlocher hl;
		hl.init(sigma, k, min_size, true);
    hl.segment_image( pImgSrc, pImgHl);
		hl.getSmooth(pImgSmooth);
    std::vector<V4R::ColorRegion>  segments = hl.getSegments();
		for(unsigned int i = 0; i < segments.size(); i++){
			segments[i].draw(pImgSrc);
		}
		
    cvNamedWindow ( "Source", 1 );
    cvShowImage ( "Source", pImgSrc );
		
    cvNamedWindow ( "Smooth", 1 );
    cvShowImage ( "Smooth", pImgSmooth );
		
    cvNamedWindow ( "Huttenlocher", 1 );
    cvShowImage ( "Huttenlocher", pImgHl );
		
    cvWaitKey ( 100000 );
    cvReleaseImage ( &pImgSrc );
    cvDestroyWindow ( "Source" );
    cvReleaseImage ( &pImgSmooth );
    cvDestroyWindow ( "Smooth" );
    cvReleaseImage ( &pImgHl );
    cvDestroyWindow ( "Huttenlocher" );
    return 0;
}


/***************************************************************************
 *   Copyright (C) 2009 by Markus Bader                                    *
 *   bader@acin.tuwien.ac.at                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


#include <mex.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <hl.h>


void mexFunction ( int nlhs, mxArray *plhs[], int nrhs,  const mxArray *prhs[] ) {
    if  ((nrhs > 5) || (nlhs == 0)) {
        mexPrintf ( "usage: <image> <sigma> <k> <min_size> <random_color on=1 off=0>\n" );
        mexErrMsgTxt ( "Wrong command" );
    }
        
    char *pSrc = ( char* ) mxGetPr ( prhs[0] );
    int *pIdim= ( int * ) mxGetDimensions ( prhs[0] );
    int rows = ( int ) pIdim[0];  //h
    int cols = ( int ) pIdim[1];  //w
    cv::Mat cvSrc(cols, rows, CV_8UC3);
    unsigned char *p = ( unsigned char* )  mxGetPr ( prhs[0]);
    std::vector<cv::Mat> mv;
    mv.push_back(cv::Mat(cols, rows, CV_8UC1,  p+(cols*rows*0)));
    mv.push_back(cv::Mat(cols, rows, CV_8UC1,  p+(cols*rows*1)));
    mv.push_back(cv::Mat(cols, rows, CV_8UC1,  p+(cols*rows*2)));
    cv::merge(mv, cvSrc);

    int num_ccs;
    float sigma = 0.5;
    float k = 500;
    int min_size = 20;
    bool random_color = false;

    if ( nrhs > 0 )  sigma = ( float ) mxGetScalar ( prhs[1] ) ;
    if ( nrhs > 1 )  k = ( float ) mxGetScalar ( prhs[2] );
    if ( nrhs > 2 )  min_size = ( float ) mxGetScalar ( prhs[3] );
    if ( nrhs > 3 )  random_color = ( float ) mxGetScalar ( prhs[4] );

    cv::Mat cvDes(cols, rows, CV_8UC1);


    V4R::Huttenlocher hl;
    hl.init(sigma, k, min_size);
    IplImage imgSrc = IplImage(cvSrc) ;
    IplImage imgDes = IplImage(cvDes);
    hl.segment_image( &imgSrc, &imgDes);

    mwSize dims[] = {rows, cols };
    mxArray *pImg = mxCreateNumericArray ( 2, dims, mxUINT8_CLASS, ( mxComplexity ) 0 );
    cv::Mat maltabDes(cols, rows, CV_8UC1, mxGetPr ( pImg ));
    cvDes.copyTo(maltabDes);;
    
    plhs[0] = pImg;

    if (nlhs > 1) {
        std::vector<V4R::ColorRegion>  segments = hl.getSegments();
        unsigned nrOfSegments = segments.size();
        mxArray *pTopLeft = mxCreateDoubleMatrix (2,  nrOfSegments, mxREAL );
        double *pDesTopLeft = mxGetPr ( pTopLeft );
        mxArray *pBottomRight = mxCreateDoubleMatrix (2,  nrOfSegments, mxREAL );
        double *pDesBottomRight = mxGetPr ( pBottomRight );
        mxArray *pCenter = mxCreateDoubleMatrix (2,  nrOfSegments, mxREAL );
        double *pDesCenter = mxGetPr ( pCenter );
        mxArray *pArea = mxCreateDoubleMatrix (1,  nrOfSegments, mxREAL );
        double *pDesArea = mxGetPr ( pArea );
        mxArray *pColor = mxCreateDoubleMatrix (4,  nrOfSegments, mxREAL );
        double *pDesColor = mxGetPr ( pColor );
        mxArray *pColorHist = mxCreateDoubleMatrix (16,  nrOfSegments, mxREAL );
        double *pDesColorHist = mxGetPr ( pColorHist );
        for (unsigned int i = 0; i < segments.size(); i++) {
            *pDesTopLeft++ = segments[i].x1;
            *pDesTopLeft++ = segments[i].y1;
            *pDesBottomRight++ = segments[i].x2;
            *pDesBottomRight++ = segments[i].y2;
            *pDesCenter++ = segments[i].cen_x;
            *pDesCenter++ = segments[i].cen_y;
            *pDesArea++ = segments[i].area;
            for (unsigned int j = 0; j < 4; j++) {
                *pDesColor++ = segments[i].color[j];
            }
            for (unsigned int j = 0; j < 16; j++) {
                *pDesColorHist++ = segments[i].histogram[j];
            }
        }
        
      if (nlhs > 1) plhs[1] = pTopLeft;
      if (nlhs > 2) plhs[2] = pBottomRight;
      if (nlhs > 3) plhs[3] = pCenter;
      if (nlhs > 4) plhs[4] = pArea;
      if (nlhs > 5) plhs[5] = pColor;
      if (nlhs > 6) plhs[6] = pColorHist;
    }

}
// kate: indent-mode cstyle; space-indent on; indent-width 0; 

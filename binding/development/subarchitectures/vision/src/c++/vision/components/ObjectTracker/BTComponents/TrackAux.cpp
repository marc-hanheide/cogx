/** @file TrackAux.cpp
 *  @brief Data structures and utility functions for tracking. 
 *
 *  Provides interfaces to various utility structures and functions.
 *
 *  @author Somboon Hongeng
 *  @date march 2007
 *  @bug No known bugs.
 */
#include "TrackAux.h"
#include <vision/components/common/SystemUtils/Common.h>
#include <opencv/highgui.h>

using namespace std;
using namespace Common;

double TrackAux::urandom(void) {
  return (double) rand() / (double) RAND_MAX;
}

double TrackAux::grandom(void) {
  static int next_gaussian = 0;
  static double saved_gaussian_value;
  
  double fac, rsq, v1, v2;
  
  if (next_gaussian == 0) {
    do {
      v1 = 2.0*urandom()-1.0;
      v2 = 2.0*urandom()-1.0;
      rsq = v1*v1+v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0*log(rsq)/rsq);
    saved_gaussian_value=v1*fac;
    next_gaussian=1;
    return v2*fac;
  } else {
    next_gaussian=0;
    return saved_gaussian_value;
  }
}

double TrackAux::grandom(double val, double sigma) {
  static int next_gaussian = 0;
  static double saved_gaussian_value;
  
  double fac, rsq, v1, v2;
  
  if (next_gaussian == 0) {
    do {
      v1 = 2.0*urandom()-1.0;
      v2 = 2.0*urandom()-1.0;
      rsq = v1*v1+v2*v2;
    } while (rsq >= 1.0 || rsq == 0.0);
    fac = sqrt(-2.0*log(rsq)/rsq);
    saved_gaussian_value=v1*fac;
    next_gaussian=1;
    return sigma*(v2*fac)+val;
  } 
  else {
    next_gaussian=0;
    return sigma*(saved_gaussian_value)+val;
  }
}

void TrackAux::AccumulateProb(float prob[], float accprob[], int arraysize) {
  float acc = 0;		
  for (int i = 0; i < arraysize;i++ )  {
    acc += prob[i] ; 
    accprob[i] = acc;
  }	
}

int TrackAux::bsearch(float cumulWei[], int arraysize, float choice) {

    if (arraysize<=0)
	throw user_error(__HERE__, " negative array size");

    int low,middle,high;

    if (choice > cumulWei[arraysize-1])
	throw user_error(__HERE__, " out of bound data");

    if(choice<cumulWei[0]) 
	high = 0;
    else {
	low=0;
	high=arraysize-1;
	while(high>(low+1)) {
	    middle=(high+low)/2;
	    if(choice>cumulWei[middle])
		low=middle;
	    else
		high=middle;
	}
    }
    return high;
}


// TrackAux::ObjType TrackAux::StringToObjtype(string _str) {
//     if (_str == "TYPE_HAND") return TYPE_HAND;
//     else if (_str == "TYPE_SHAPE") return TYPE_SHAPE;
//     else if (_str == "TYPE_FLAG") return TYPE_FLAG;
//     else if (_str == "TYPE_CAR") return TYPE_CAR;
//     else if (_str == "TYPE_BOX") return TYPE_BOX;
//     else if (_str == "TYPE_BALL") return TYPE_BALL;
//     else return TYPE_UNDEF;
// }

TrackAux::ObjShape TrackAux::StringToShapetype(string _str) {
    if (_str == "SH_CIRCLE") return SH_CIRCLE;
    else if (_str == "SH_RECT") return SH_RECT;
    else return SH_UNDEF;
}

// std::string TrackAux::ObjtypeToString(ObjType _type) {
//   if (_type == TYPE_HAND) return string("TYPE_HAND");
//   else if (_type == TYPE_SHAPE) return string("TYPE_SHAPE");
//   else if (_type == TYPE_FLAG) return string("TYPE_FLAG");
//   else if (_type == TYPE_CAR) return string("TYPE_CAR");
//   else if (_type == TYPE_BOX) return string("TYPE_BOX");
//   else if (_type == TYPE_BALL) return string("TYPE_BALL");
//   else return string("TYPE_UNDEF");
// }

std::string TrackAux::ShapetypeToString(ObjShape _shape) {
    if (_shape == SH_CIRCLE) return string("SH_CIRCLE");
    else if (_shape == SH_RECT) return string("SH_RECT");
    else return string("SH_UNDEF");
}



void TrackAux::updateHist(CvHistogram *crt_hist, 
			  IplImage * histsrcimg, CvRect roirect, 
			  IplImage *mask, bool notclean)
{
    CvRect orig_roirect = cvGetImageROI(histsrcimg);
    cvSetImageROI(histsrcimg, roirect);
    CvSize rectimgsize = cvSize(roirect.width,roirect.height);   
    IplImage * hsvImg = cvCreateImage(rectimgsize,8,3);     
    cvCvtColor(histsrcimg, hsvImg, CV_BGR2HSV); 
    cvSetImageROI(histsrcimg, orig_roirect);

    IplImage * imgplane[3] = {cvCreateImage(rectimgsize,8,1),
			      cvCreateImage(rectimgsize,8,1),
			      cvCreateImage(rectimgsize,8,1)};
    cvCvtPixToPlane( hsvImg, imgplane[0],imgplane[1],imgplane[2], 0 );

    cvCalcHist(imgplane, crt_hist, notclean, mask) ; 

    float maxhistvalue = 0 ;
    cvGetMinMaxHistValue(crt_hist,NULL,&maxhistvalue) ; 
    cvScale(&crt_hist->mat, &crt_hist->mat, 255.0/maxhistvalue) ;     
    
    // ShowHistogram(*crt_hist);
//     static int id=0;
//     char fname[1024];
//     snprintf(fname, 1024, "orig-%d.jpg", id++);
//     cvSaveImage(fname, histsrcimg);


    cvReleaseImage(&hsvImg) ;
    for(int i=0;i<3;i++){cvReleaseImage(&imgplane[i]);}  ;
}

CvHistogram *TrackAux::createHist() 
{
    float h_ranges[2] = {0,255}; 
    float s_ranges[2] = {0,200}; 
    float* ranges[] = { h_ranges, s_ranges }; //memory leak problem ??
    int hist_size[] = {15,15}; 
    return (cvCreateHist(2, hist_size, CV_HIST_ARRAY, ranges));
}


void TrackAux::ShowHistogram(CvHistogram &hist)
{
    IplImage *histImage = cvCreateImage(cvSize(320,320),8,1);

    float min_value, max_value;
    int min_idx, max_idx;
    cvGetMinMaxHistValue(&hist,&min_value,&max_value,&min_idx,&max_idx);
    
    //scale the bin values so that they fit in the image representation
    //cvScale( hist->bins, hist->bins, ((double)histImage->height)/max_value, 0 );
        
    //set all histogram values to 0 (black)
    cvSet( histImage, cvScalarAll(0), 0 );

    //create a factor for scaling along the width
    int bin_w = cvRound((double)histImage->width/15);
    
    //create a factor for scaling along the width
    int bin_h = cvRound((double)histImage->height/15);
    
    for( int i = 0; i < 15; i++ ) {  // h-value
	for ( int j = 0; j < 15; j++ ) { // s-value

	    //draw the histogram data onto the histogram image
	    cvRectangle( histImage, 
			 cvPoint(i*bin_w, j*bin_h),
			 cvPoint((i+1)*bin_w, (j+1)*bin_h),
			 cvScalarAll(cvGetReal2D(hist.bins,i,j)),
			 -1, 8, 0 );

	    //get the value at the current histogram bucket
	    //float* bins = cvGetHistValue_1D(hist,i);
	    //increment the mean value
	    //mean += bins[0];
	}
    }

    //cvNamedWindow( "H-S Histogram", 1 );
    //cvShowImage( "H-S Histogram", histImage );
    //cvWaitKey();
    
    static int i=0;
    char fname[1024];
    snprintf(fname, 1024, "hist-%d.jpg", i++);
    cvSaveImage(fname, histImage);

    cvReleaseImage(&histImage);
}

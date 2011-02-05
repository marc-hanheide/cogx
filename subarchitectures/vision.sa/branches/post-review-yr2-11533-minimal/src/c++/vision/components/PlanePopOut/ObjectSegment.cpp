/**
 * @author Kai ZHOU
 * @date Dec 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "ObjectSegment.h"


// Segmentation costants


using namespace std;
using namespace cdl;
using namespace VisionData;
using namespace Video;

	/* Constructor */
ObjectSegment::ObjectSegment()
{
	leftimg = NULL;
	disparityimg = NULL;
	sobelimg = NULL;
	cannyimg = NULL;
	LPimg = NULL;
	doDisplay = false;
}

	/* Destructor */
ObjectSegment::~ObjectSegment()
{
	cvWaitKey(0);
	if(doDisplay)
	{
		cvDestroyWindow("left");
		cvDestroyWindow("canny");
		cvDestroyWindow("sobel");
		cvDestroyWindow("LP");
		cvDestroyWindow("disparity");
	}

	cvReleaseImage(&leftimg);
	cvReleaseImage(&disparityimg);
	cvReleaseImage(&sobelimg);
	cvReleaseImage(&cannyimg);
	cvReleaseImage(&LPimg);
}

void ObjectSegment::doSegment()
{
	GetSobelCannyImg(leftimg, &sobelimg, &cannyimg);
}

void ObjectSegment::GetSobelCannyImg(IplImage* srcImg, IplImage** sobelImg, IplImage** cannyImg)
{
	IplImage * pImage8uGray=NULL;
	IplImage * pImage8uSmooth=NULL;
	IplImage * pImage16uGraySobel=NULL;
	IplImage * pImage8uGraySobelShow=NULL;
	IplImage * pImage8uGrayCanny=NULL;

	pImage8uGray=cvCreateImage(cvGetSize(srcImg),IPL_DEPTH_8U,1);
	pImage8uSmooth=cvCreateImage(cvGetSize(srcImg),IPL_DEPTH_8U,1);
	pImage8uGraySobelShow=cvCreateImage(cvGetSize(srcImg),IPL_DEPTH_8U,1);
	pImage8uGrayCanny=cvCreateImage(cvGetSize(srcImg),IPL_DEPTH_8U,1);
	//to gray
	cvCvtColor(srcImg,pImage8uGray,CV_BGR2GRAY);
	//Gaussian filter
	cvSmooth(pImage8uGray,pImage8uSmooth,CV_GAUSSIAN,3,0,0);
	//cvSobel need IPL_DEPTH_16S
	pImage16uGraySobel=cvCreateImage(cvGetSize(srcImg),IPL_DEPTH_16S,1);
	//cal sobel img, x-orientation
	cvSobel(pImage8uSmooth,pImage16uGraySobel,0,1,3);
	//convert back for display
	cvConvertScaleAbs(pImage16uGraySobel,pImage8uGraySobelShow,1,0);
	//canny
	cvCanny(pImage8uSmooth,pImage8uGrayCanny,100,200,3);

	*sobelImg = pImage8uGraySobelShow;
	*cannyImg = pImage8uGrayCanny;

/*	if (doDisplay)
	{
		cvNamedWindow("sobel",CV_WINDOW_AUTOSIZE);
		cvShowImage("sobel",pImage8uGraySobelShow);
		cvNamedWindow("canny",CV_WINDOW_AUTOSIZE);
		cvShowImage("canny",pImage8uGrayCanny);
	}
*/
	cvReleaseImage(pImage8uGray);
	cvReleaseImage(pImage8uSmooth);
	cvReleaseImage(pImage16uGraySobel);
	cvReleaseImage(pImage8uGraySobelShow);
	cvReleaseImage(pImage8uGrayCanny);
}

void Display()
{
	cvNamedWindow("sobel",CV_WINDOW_AUTOSIZE);
	cvShowImage("sobel",sobelimg);
	cvNamedWindow("canny",CV_WINDOW_AUTOSIZE);
	cvShowImage("canny",cannyimg);
	cvNamedWindow("LP",CV_WINDOW_AUTOSIZE);
	cvShowImage("LP",LPimg);
}



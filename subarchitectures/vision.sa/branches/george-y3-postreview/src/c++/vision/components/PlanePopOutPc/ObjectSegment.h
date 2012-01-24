/**
 * @author Kai ZHOU
 * @date Dec 2009
 *
 * A component segment the objects.
 */

#ifndef OBJECT_SEGMENT_H
#define OBJECT_SEGMENT_H

#include <vector>
#include <string>
#include <queue>
#include <map>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <cast/architecture/ManagedComponent.hpp>
#include <VideoClient.h>
#include <StereoClient.h>
#include <../../VisionUtils.h>

#include <VisionData.hpp>



class ObjectSegment
{
public:

	/* Constructor */
	ObjectSegment();

	/* Destructor */
	~ObjectSegment();

	void doSegment();

private:
	
	IplImage* leftimg;
	IplImage* disparityimg;
	IplImage* sobelimg;
	IplImage* cannyimg;
	IplImage* LPimg;

	bool doDisplay;

	void GetSobelCannyImg(IplImage* srcImg, IplImage** sobelImg, IplImage** cannyImg);
	void Display();


}

#endif


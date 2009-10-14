#ifndef CENSUSGPU_H 
#define CENSUSGPU_H

#define USE_OPENCV 1

#ifdef USE_OPENCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#endif


#include <stdio.h>
#include <iostream>
#include <fstream>
#include <time.h>

#include "census.h"
#include "align.h"

using namespace std;

class CensusGPU {
private:
	bool async;
	int  asyncImgLoadNr, asyncImgMatchNr;
	int  dispMin, dispMax;

	unsigned int iWidth, iHeight, iOrgWidth;

	unsigned char *d_left,
		*d_right,
		*d_disparityMap;

public:
	CensusGPU(int _dispMax = 50);
	~CensusGPU();

	//void SetImages(IplImage* _Left, IplImage* _Right, IplImage* _DM, IplImage *_GT = 0, IplImage *_EM = 0, float _bad_match_thresh = 0.0);
#ifdef USE_OPENCV
	void setImages(IplImage* left, IplImage* right);
	void getDisparityMap(IplImage *dm);
#endif
	void setImages(unsigned char *left, unsigned char *right, unsigned int iWidth, unsigned int iHeight);

	void match();

	void printTiming();

	template <class T> void writeArrayToFile(T *a, string file, int width, int height, int startX = 0, int startY = 0, int stopX = -1, int stopY = -1);
	template <class T> void writeDeviceDataToFile(T *d_a, string file, int width, int height);

};

#endif

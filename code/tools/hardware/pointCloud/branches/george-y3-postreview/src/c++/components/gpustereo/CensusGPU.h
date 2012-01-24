#ifndef CENSUSGPU_H
#define CENSUSGPU_H

#define USE_OPENCV 1

#ifdef USE_OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
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

	unsigned int iWidth, iHeight, iOrgWidth;

	unsigned int disp_min;
	unsigned int disp_max;
	unsigned int disp_step;
	bool sparse;
	unsigned int blockSize;
	int dmScale;

	unsigned char *d_left,
		*d_right,
		*d_disparityMap;

public:
	CensusGPU();
	~CensusGPU();

	//void SetImages(IplImage* _Left, IplImage* _Right, IplImage* _DM, IplImage *_GT = 0, IplImage *_EM = 0, float _bad_match_thresh = 0.0);
#ifdef USE_OPENCV
	void setImages(IplImage* left, IplImage* right);
	void getDisparityMap(IplImage *dm);
	void getConfidenceMap(IplImage *cm);
	void getTexture(IplImage *tex);
	void getDepthMap(IplImage *dm);
#endif
	void setImages(unsigned char *left, unsigned char *right, unsigned int iWidth, unsigned int iHeight);
	void setOptions(unsigned int disp_min=0, unsigned int disp_max=59, unsigned int disp_step=1, bool sparse = true, unsigned int blockSize = 5, int dmScale = 4);

	void match();

	void printTiming();

	template <typename T> void writeArrayToFile(T *a, string file, int width, int height, int startX = 0, int startY = 0, int stopX = -1, int stopY = -1);
	template <typename T> void writeDeviceDataToFile(T *d_a, string file, int width, int height);

};

#endif

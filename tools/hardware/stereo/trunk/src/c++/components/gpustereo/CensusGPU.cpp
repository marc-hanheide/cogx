#include <cstring>
#include "CensusGPU.h"



CensusGPU::CensusGPU(int _dispMax) {
	iWidth = 0;
	iHeight = 0;
	async = false;
	asyncImgLoadNr = 0;
	asyncImgMatchNr = 0;
  // always assume min disparity of 0
	dispMin = 0;
	dispMax = _dispMax;
}

CensusGPU::~CensusGPU() {
	gpuCensusImageCleanup();
}

#ifdef USE_OPENCV
void CensusGPU::setImages(IplImage* left, IplImage* right) {
	unsigned char *cLeft, *cRight;
	int i, j;
	int modWidth;

  assert(left->nChannels == 1 && right->nChannels == 1);

	iOrgWidth = left->width;
	modWidth = iAlignUp(left->width, 16);


	if (async) {
    assert(0);
		if (this->iWidth != modWidth || this->iHeight != left->height) {
			if (this->iWidth || this->iHeight)
				gpuCensusImageCleanup();

			gpuCensusImageSetup(modWidth, left->height, dispMin, dispMax);

			this->iWidth = modWidth;
			this->iHeight = left->height;
		}

		cLeft = gpuGetLeftImageBuffer(asyncImgLoadNr);
		cRight = gpuGetRightImageBuffer(asyncImgLoadNr);

		asyncImgLoadNr = 1 - asyncImgLoadNr;
	}
	else {
		cLeft  = (unsigned char*)malloc(modWidth*left->height*sizeof(unsigned char));
		cRight = (unsigned char*)malloc(modWidth*right->height*sizeof(unsigned char));
	}

	for (i=0; i < left->height; i++) {
		for (j=0; j < left->width; j++) {
			cLeft[i * modWidth + j] = (unsigned char)cvGetReal2D(left, i, j);
			cRight[i * modWidth + j] = (unsigned char)cvGetReal2D(right, i, j);
		}
		for (j=left->width; j < modWidth; j++) {
			cLeft[i * modWidth + j] = 0;
			cRight[i * modWidth + j] = 0;
		}
	}


	if (async) {
		gpuCensusLoadImages(asyncImgLoadNr);
	}
	else {
		setImages(cLeft, cRight, modWidth, left->height);

		free(cLeft);
		free(cRight);
	}
}

void CensusGPU::getDisparityMap(IplImage *dm) {
	float *fDM;
	float f;

	fDM = (float*)malloc(this->iWidth*this->iHeight*sizeof(float));
	gpuGetDisparityMap(fDM);

	for (unsigned int y=0; y < this->iHeight; y++) {
		for (unsigned int x=0; x < this->iOrgWidth; x++) {
			f = fDM[y*this->iWidth+x];

			cvSet2D(dm, y, x, cvScalar(f, f, f));
		}
	}

	free(fDM);
}
#endif

void CensusGPU::setImages(unsigned char *left, unsigned char *right, unsigned int iWidth, unsigned int iHeight) {

	if (this->iWidth != iWidth || this->iHeight != iHeight) {
		if (this->iWidth || this->iHeight)
			gpuCensusImageCleanup();

		gpuCensusImageSetup(iWidth, iHeight, dispMin, dispMax);
	}

	this->iWidth = iWidth;
	this->iHeight = iHeight;

	if (async) {
		unsigned char *h_left, *h_right;
		h_left = gpuGetLeftImageBuffer(asyncImgLoadNr);
		h_right = gpuGetRightImageBuffer(asyncImgLoadNr);

		memcpy(h_left, left, iWidth*iHeight*sizeof(unsigned char));
		memcpy(h_right, right, iWidth*iHeight*sizeof(unsigned char));

		gpuCensusLoadImages(asyncImgLoadNr);
		asyncImgLoadNr = 1 - asyncImgLoadNr;
	}
	else {
		gpuCensusSetImages(left, right);
	}
}

void CensusGPU::match() {
	//LARGE_INTEGER freq;
	//LARGE_INTEGER t1, t2;
	//double elapsed;

	//QueryPerformanceFrequency(&freq);
	//QueryPerformanceCounter(&t1);

	if (async) {
		gpuCensusSetAsyncImageNr(asyncImgMatchNr);
		asyncImgMatchNr = 1 - asyncImgMatchNr;
	}

	gpuCensusTransform();

	gpuCalcDSI();

	gpuAggregateCosts();

	gpuRefineSubPixel();

	gpuCompareDisps();

	gpuRoundAndScaleDisparities();

	//QueryPerformanceCounter(&t2);
	//elapsed = (t2.QuadPart - t1.QuadPart) * 1000.0 / freq.QuadPart;
	//printf("\n\nOverall: %.3f ms / %.1f fps\n", elapsed, 1000/elapsed);
}

void CensusGPU::printTiming() {
	printf("========================================\n");
	printf("   Census Transform............. %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n", 
		getCensusTiming(eCensusTransform),
		getCensusMemory(eCensusTransform)/(getCensusTiming(eCensusTransform)*1000000),
		getCensusFLOP(eCensusTransform)/(getCensusTiming(eCensusTransform)*1000000));

	printf("   Calc DSI..................... %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n",
		getCensusTiming(eCalcDSI),
		getCensusMemory(eCalcDSI)/(getCensusTiming(eCalcDSI)*1000000),
		getCensusFLOP(eCalcDSI)/(getCensusTiming(eCalcDSI)*1000000));

	printf("   Aggregate Costs.............. %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n", 
		getCensusTiming(eAggregateCosts),
		getCensusMemory(eAggregateCosts)/(getCensusTiming(eAggregateCosts)*1000000),
		getCensusFLOP(eAggregateCosts)/(getCensusTiming(eAggregateCosts)*1000000));

	printf("   Refine Subpixel.............. %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n", 
		getCensusTiming(eRefineSubPixel),
		getCensusMemory(eRefineSubPixel)/(getCensusTiming(eRefineSubPixel)*1000000),
		getCensusFLOP(eRefineSubPixel)/(getCensusTiming(eRefineSubPixel)*1000000));

	printf("   Compare Disparities.......... %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n", 
		getCensusTiming(eCompareDisps),
		getCensusMemory(eCompareDisps)/(getCensusTiming(eCompareDisps)*1000000),
		getCensusFLOP(eCompareDisps)/(getCensusTiming(eCompareDisps)*1000000));

	printf("   Round + Scale Disparities.... %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n", 
		getCensusTiming(eRoundAndScaleDisparities),
		getCensusMemory(eRoundAndScaleDisparities)/(getCensusTiming(eRoundAndScaleDisparities)*1000000),
		getCensusFLOP(eRoundAndScaleDisparities)/(getCensusTiming(eRoundAndScaleDisparities)*1000000));

	printf("\n");
}

template <typename T>
void CensusGPU::writeArrayToFile(T *a, string file, int width, int height, int startX, int startY, int stopX, int stopY) {
	int x, y;

	if (stopX < 1) stopX = width;
	if (stopY < 1) stopY = height;

	ofstream hFile(file.c_str());

	for (y = startY; y < stopY; y++) {
		for (x = startX; x < stopX; x++) {
			hFile << a[y*width+x] << ";";
		}
		hFile << endl;
	}

	hFile.close();
}

#include "CensusGPU.h"



CensusGPU::CensusGPU() {
	iWidth = 0;
	iHeight = 0;
	async = false;
	asyncImgLoadNr = 0;
	asyncImgMatchNr = 0;

	disp_min = 0;
	disp_max = 32;  // some default
	disp_step = 1;
	sparse = true;
	blockSize = 5;
	dmScale = 1;
}

CensusGPU::~CensusGPU() {
	gpuCensusImageCleanup();
}

#ifdef USE_OPENCV
void CensusGPU::setImages(IplImage* left, IplImage* right) {
	unsigned char *cLeft, *cRight;
	int i, j;
	int modWidth;

	if (left->nChannels != 1)
	{
		IplImage *tmp = cvCreateImage(cvSize(left->width, left->height), IPL_DEPTH_8U, 1);
		cvCvtColor(right, tmp, CV_RGB2GRAY);
		right = cvCloneImage(tmp);
		cvCvtColor(left, tmp, CV_RGB2GRAY);
		left = cvCloneImage(tmp);
		cvReleaseImage(&tmp);
	}


	iOrgWidth = left->width;
	modWidth = iAlignUp(left->width, 16);


	if (async) {
		if (this->iWidth != modWidth || this->iHeight != left->height) {
			if (this->iWidth || this->iHeight)
				gpuCensusImageCleanup();

			gpuCensusImageSetup(modWidth, left->height, disp_min, disp_max, disp_step, sparse, blockSize, dmScale);

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

void CensusGPU::getConfidenceMap(IplImage *cm) {
	int *CM;
	int v;

	CM = (int*)malloc(this->iWidth*this->iHeight*sizeof(int));
	gpuGetConfidenceMap(CM);

	for (unsigned int y=0; y < this->iHeight; y++) {
		for (unsigned int x=0; x < this->iOrgWidth; x++) {
			v = CM[y*this->iWidth+x];

			cvSet2D(cm, y, x, cvScalar(v, v, v));
		}
	}

	free(CM);
}

void CensusGPU::getTexture(IplImage *tex) {
	int *ttex;
	int v;

	ttex = (int*)malloc(this->iWidth*this->iHeight*sizeof(int));
	gpuGetTexture(ttex);

	for (unsigned int y=0; y < this->iHeight; y++) {
		for (unsigned int x=0; x < this->iOrgWidth; x++) {
			v = ttex[y*this->iWidth+x];

			cvSet2D(tex, y, x, cvScalar(v, v, v));
		}
	}

	free(ttex);
}


void CensusGPU::getDepthMap(IplImage *dm) {
	int *DM;
	int v;

	DM = (int*)malloc(this->iWidth*this->iHeight*sizeof(int));
	gpuGetDepthMap(DM);

	for (unsigned int y=0; y < this->iHeight; y++) {
		for (unsigned int x=0; x < this->iOrgWidth; x++) {
			v = DM[y*this->iWidth+x];

			cvSet2D(dm, y, x, cvScalar(v, v, v));
		}
	}

	free(DM);
}
#endif

void CensusGPU::setOptions(unsigned int disp_min, unsigned int disp_max, unsigned int disp_step, bool sparse, unsigned int blockSize, int dmScale) {
	this->disp_min = disp_min;
	this->disp_max = disp_max;
	this->disp_step = disp_step;
	this->sparse = sparse;
	this->blockSize = blockSize;
	this->dmScale = dmScale;
}

void CensusGPU::setImages(unsigned char *left, unsigned char *right, unsigned int iWidth, unsigned int iHeight) {
	if (this->iWidth != iWidth || this->iHeight != iHeight) {
		if (this->iWidth || this->iHeight)
			gpuCensusImageCleanup();

		printf("width %d height %d",iWidth,iHeight);
		gpuCensusImageSetup(iWidth, iHeight, disp_min, disp_max, disp_step, sparse, blockSize, dmScale);
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

	//gpuCalcTextureMap();

	gpuCalcDSI();

	gpuAggregateCosts();

	gpuRefineSubPixel();

	gpuCompareDisps();

	//gpuThresholdConfidence();

	/*

	gpuThresholdTexture();

	gpuRoundAndScaleDisparities();
	*/

	//gpuCalcDepthMap(600.f, 0.01f);

	//gpuRoundScaleThresholdDisparities();

	//QueryPerformanceCounter(&t2);
	//elapsed = (t2.QuadPart - t1.QuadPart) * 1000.0 / freq.QuadPart;
	//printf("\n\nOverall: %.3f ms / %.1f fps\n", elapsed, 1000/elapsed);
}

void CensusGPU::printTiming() {
	printf("====================================================================\n");
	printf("   Census Transform............. %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n",
		getCensusTiming(eCensusTransform),
		getCensusMemory(eCensusTransform)/(getCensusTiming(eCensusTransform)*1000000),
		getCensusFLOP(eCensusTransform)/(getCensusTiming(eCensusTransform)*1000000));

	printf("   Calc DSI..................... %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n",
		getCensusTiming(eCalcDSI),
		getCensusMemory(eCalcDSI)/(getCensusTiming(eCalcDSI)*1000000),
		getCensusFLOP(eCalcDSI)/(getCensusTiming(eCalcDSI)*1000000));

	printf("   Calc Texture Map............. %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n",
		getCensusTiming(eCalcTextureMap),
		getCensusMemory(eCalcTextureMap)/(getCensusTiming(eCalcTextureMap)*1000000),
		getCensusFLOP(eCalcTextureMap)/(getCensusTiming(eCalcTextureMap)*1000000));

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

	printf("   Threshold Confidence......... %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n",
		getCensusTiming(eThresholdConfidence),
		getCensusMemory(eThresholdConfidence)/(getCensusTiming(eThresholdConfidence)*1000000),
		getCensusFLOP(eThresholdConfidence)/(getCensusTiming(eThresholdConfidence)*1000000));

	printf("   Calculate Depth Map......... %.2f ms - %5.1f GB/s - %5.1f GFLOPS\n",
		getCensusTiming(eCalcDepthMap),
		getCensusMemory(eCalcDepthMap)/(getCensusTiming(eCalcDepthMap)*1000000),
		getCensusFLOP(eCalcDepthMap)/(getCensusTiming(eCalcDepthMap)*1000000));

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

enum CensusStep {
	eCensusTransform,
	eCalcDSI,
	eAggregateCosts,
	eRefineSubPixel,
	eCompareDisps,
	eRoundAndScaleDisparities
};

extern "C" float getCensusTiming(CensusStep s);
extern "C" unsigned int getCensusMemory(CensusStep s);
extern "C" unsigned int getCensusFLOP(CensusStep s);

// values by Michael Weber:
//disp_min=0, unsigned int disp_max=50, unsigned int disp_step=1, bool sparse = true, unsigned int blockSize = 5
extern "C" void gpuCensusImageSetup(unsigned int w, unsigned int h, unsigned int disp_min=0, unsigned int disp_max=100, unsigned int disp_step=1, bool sparse = true, unsigned int blockSize = 5);

extern "C" void gpuCensusSetImages(unsigned char *left, unsigned char *right);

extern "C" unsigned char* gpuGetLeftImageBuffer(int nr);
extern "C" unsigned char* gpuGetRightImageBuffer(int nr);
extern "C" void gpuCensusSetAsyncImageNr(int nr);
extern "C" void gpuCensusLoadImages(int nr);

extern "C" void gpuCensusImageCleanup();

extern "C" void gpuCensusTransform();

extern "C" void gpuCensusTransformSparse();

extern "C" void gpuCalcDSI();

extern "C" void gpuAggregateCosts();

extern "C" void gpuRefineSubPixel();

extern "C" void gpuCompareDisps();

extern "C" void gpuRoundAndScaleDisparities();

extern "C" void gpuGetDisparityMap(float *h_data);

extern "C" void debugGetCensusLeft(int *h_data);
extern "C" void debugGetCensusRight(int *h_data);
extern "C" void debugGetDSI(int *h_data, int d);
extern "C" void debugGetDMI_LR(float *h_data);
extern "C" void debugGetDMI_RL(float *h_data);

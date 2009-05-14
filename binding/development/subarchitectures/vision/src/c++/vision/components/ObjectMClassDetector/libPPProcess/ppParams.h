#ifndef PPPARAMS_H_
#define PPPARAMS_H_

struct preAndPostProcessParams{
	unsigned int minWidth;	
	unsigned int minHeight;
	unsigned int maxWidth;
	unsigned int maxHeight;
	float borderfactor;
	unsigned int scaledWidth;
	unsigned int scaledHeight;
	unsigned int blurKernelWidth;
	unsigned int extraBoundary;
	int jitterX;
	int jitterY;
	int localContextL;
	int localContextR;
	int localContextT;
	int localContextB;
	float smoothingBandWidthX;
	float smoothingBandWidthY;
	float smoothingBandWidthScale;
	int minDetectionsPerMode;
	float minNMSScore;
	int scoreMode;
	int mirror;	
	bool ahd_interpolate;
	int sharpenPercentTrain;
	int sharpenPercentTest;
	
	preAndPostProcessParams() : minWidth(40), minHeight(40), maxWidth(0), maxHeight(0), borderfactor(1.0), scaledWidth(128), scaledHeight(128), blurKernelWidth(2), extraBoundary(2), jitterX(0), jitterY(0), localContextL(0), localContextR(0), localContextT(0), localContextB(0), smoothingBandWidthX(4) , smoothingBandWidthY(8) , smoothingBandWidthScale(1.0), minDetectionsPerMode(4), minNMSScore(0), scoreMode(0), mirror(0), ahd_interpolate(false), sharpenPercentTrain(0), sharpenPercentTest(0) {};
};

#endif /*PPPARAMS_H_*/

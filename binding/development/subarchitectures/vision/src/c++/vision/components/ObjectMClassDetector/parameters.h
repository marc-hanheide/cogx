#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include <libInifile/inifile.h>
#include <libPPProcess/ppParams.h>

enum classifierMethod {SVM, AdaBoost};

struct SW_Params{
	int windowWidth;
	int windowHeight;
	int strideX;
	int strideY;
	float scaleStep;
	std::string features;
	int normalizeCues;
	
	classifierMethod classifier;
	
	SW_Params() : windowWidth(64), windowHeight(128), strideX(8), strideY(8), scaleStep(1.05), features(""),  normalizeCues(2), classifier(SVM) {};
};


struct LearningParams{
	std::string modelFile;
	std::string sampleFile;
	unsigned int noRandomPerNegative;
	float negativeScaleStep;
	int noNegativeScales;
	float minDetectionMargin;		
	unsigned int maxRetrainingSamples;
		
	int svm_kernelType;
	bool svm_maxNormalize;
	float svm_kernel_sigma;
	float svm_costFactor;
	float svm_slack;
	float svm_min_slack_C;
	float svm_max_slack_C;
	float svm_step_C;
	float svm_min_kernel_sigma;
	float svm_max_kernel_sigma;
	float svm_step_kernel_sigma;	
	
	bool svm_slackCV;
	bool svm_trainProbSigmoid;
	bool svm_autoRange;
	float svm_autoRangeSubset;
	
	
	int folds;
	int used_folds;
	
	int boosting_rounds;
	
	LearningParams() : modelFile("svmHOG.model"), sampleFile("trainingSamples.svm"), noRandomPerNegative(10), negativeScaleStep(1.2), noNegativeScales(1), minDetectionMargin(0), maxRetrainingSamples(0), svm_kernelType(0), svm_maxNormalize(false), svm_kernel_sigma(1), svm_costFactor(3), svm_slack(0.01), svm_min_slack_C(1e-6), svm_max_slack_C(1e6), svm_step_C(10), svm_min_kernel_sigma(1e-6), svm_max_kernel_sigma(1e6), svm_step_kernel_sigma(10), svm_slackCV(false), svm_trainProbSigmoid(false), svm_autoRange(false), svm_autoRangeSubset(0.1), folds(5), used_folds(5), boosting_rounds(100) {};
};



inifile::IniFile parseConfFile(const char* fileName, SW_Params& swParams, preAndPostProcessParams &ppParams, LearningParams &svmParams);
bool fileExists (const char * fileName);
int annoCount(const AnnotationList& annotations);

#endif /*PARAMETERS_H_*/

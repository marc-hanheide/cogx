#include <sys/stat.h>
#include <libInifile/inifile.h>
#include <libAnnotation/annotationlist.h>
#include "parameters.h"

bool fileExists (const char * fileName) {
   struct stat buf;
   int i = stat( fileName, &buf );
     if ( i == 0 )
       return true;
     return false;
}

int annoCount(const AnnotationList& annotations) {
	int sum = 0;
	for(unsigned int i = 0; i < annotations.size(); i++)
		sum += annotations[i].size();
	
	return sum;
}


void parseLearningParams(const inifile::IniFile &confFile, LearningParams& learnParams) {
	learnParams.svm_kernelType = confFile.GetInt("svm_kernel", "Learning", 0);
	learnParams.svm_maxNormalize = confFile.GetBool("svm_maxNormalize", "Learning", false);
	learnParams.svm_kernel_sigma = confFile.GetFloat("svm_kernel_sigma", "Learning", 1.0);
	learnParams.svm_costFactor = confFile.GetFloat("svm_costFactor", "Learning", 1.0);
	learnParams.svm_slack = confFile.GetFloat("svm_slack", "Learning", 0.01);	
	learnParams.svm_slackCV = confFile.GetBool("svm_slackCV", "Learning", false);
	learnParams.svm_min_slack_C = confFile.GetFloat("svm_min_slack_C", "Learning", 1e-6);
	learnParams.svm_max_slack_C = confFile.GetFloat("svm_max_slack_C", "Learning", 1e6);
	learnParams.svm_step_C = confFile.GetFloat("svm_step_C", "Learning", 10);
	learnParams.svm_min_kernel_sigma = confFile.GetFloat("svm_min_kernel_sigma", "Learning", 1e-6);
	learnParams.svm_max_kernel_sigma = confFile.GetFloat("svm_max_kernel_sigma", "Learning", 1e6);
	learnParams.svm_step_kernel_sigma = confFile.GetFloat("svm_step_kernel_sigma", "Learning", 10);
	learnParams.svm_trainProbSigmoid = confFile.GetBool("svm_fitProbSigmoid", "Learning", false);
	learnParams.svm_autoRange = confFile.GetBool("svm_autoRange", "Learning", true);
	learnParams.svm_autoRangeSubset = confFile.GetFloat("svm_autoRangeSubset", "Learning", 0.1);
		
	learnParams.folds = confFile.GetInt("folds", "Learning", 5);
	learnParams.used_folds = confFile.GetInt("used_folds", "Learning", 5);	
	
	learnParams.boosting_rounds = confFile.GetInt("boosting_rounds", "Learning", 100);
		
	learnParams.noRandomPerNegative = confFile.GetInt("noNegCrops", "Learning", 10);
	learnParams.noNegativeScales = confFile.GetInt("numberNegativeSamplesScaleSteps", "Learning" ,3);
    learnParams.negativeScaleStep = confFile.GetFloat("negativeSamplesScaleStep", "Learning" ,1.2);
	learnParams.minDetectionMargin = confFile.GetFloat("minDetectionMargin", "Learning", 0.0);
	learnParams.maxRetrainingSamples = confFile.GetInt("maximumRetrainingSamples", "Learning", 30000);
	
	
}


inifile::IniFile parseConfFile(const char* fileName, SW_Params& swParams, preAndPostProcessParams &ppParams, LearningParams &svmParams) {
	inifile::IniFile confFile(true);
	
	if (!confFile.Load(fileName)) {
		std::cerr << "Configuration file " << fileName << " could not be opened!" << std::endl;
		return confFile;
	};
	
	swParams.windowWidth = confFile.GetInt("slidingWindowWidth", "SlidingWindow", 20);
	swParams.windowHeight = confFile.GetInt("slidingWindowHeight", "SlidingWindow", 20);
	swParams.strideX = confFile.GetInt("strideX", "SlidingWindow", 4);
	swParams.strideY = confFile.GetInt("strideY", "SlidingWindow", 4);
	swParams.scaleStep = confFile.GetFloat("scaleStep", "SlidingWindow", 1.05);
	swParams.features =  confFile.GetString("features", "SlidingWindow", "");
	swParams.normalizeCues = confFile.GetInt("normalizeCues", "SlidingWindow", 0);
	
	std::string classifier_method = confFile.GetString("classifier", "SlidingWindow");
	swParams.classifier = SVM;
	if (classifier_method == "AdaBoost") 
		swParams.classifier = AdaBoost;
	
			
	ppParams.scaledWidth = confFile.GetInt("scaledWidth", "PreProcess", 130);
	ppParams.scaledHeight = confFile.GetInt("scaledHeight", "PreProcess", 130);
	ppParams.borderfactor = confFile.GetFloat("borderFactor", "PreProcess", 1.2);
	ppParams.minWidth = confFile.GetInt("minWidth", "PreProcess", 0);
	ppParams.maxWidth = confFile.GetInt("maxWidth", "PreProcess", 0);
	ppParams.minHeight = confFile.GetInt("minHeight", "PreProcess", 0);
	ppParams.maxHeight = confFile.GetInt("maxHeight", "PreProcess", 0);
	ppParams.blurKernelWidth = confFile.GetInt("blurKernelWidth", "PreProcess", 2);
	ppParams.extraBoundary = confFile.GetInt("extraBoundary", "PreProcess", 0);
	ppParams.jitterX = confFile.GetInt("jitterX", "PreProcess", 1);
	ppParams.jitterY = confFile.GetInt("jitterY", "PreProcess", 1);
	ppParams.mirror = confFile.GetInt("mirror", "PreProcess", 0);	
	
	ppParams.ahd_interpolate = confFile.GetInt("bayerInterpolationMethod", "PreProcess", 0);
	ppParams.sharpenPercentTrain = confFile.GetInt("sharpenPercentageTrain", "PreProcess", 0);
	ppParams.sharpenPercentTest = confFile.GetInt("sharpenPercentageTest", "PreProcess", 0);
	
	ppParams.smoothingBandWidthX = confFile.GetFloat("bandWidthX", "PostProcess", 4);
	ppParams.smoothingBandWidthY = confFile.GetFloat("bandWidthY", "PostProcess", 4);
	ppParams.smoothingBandWidthScale = confFile.GetFloat("bandWidthScale", "PostProcess", 1.6);
	ppParams.minDetectionsPerMode = confFile.GetInt("minDetectionsPerMode", "PostProcess", 3);
	ppParams.minNMSScore = confFile.GetFloat("mininumDetectionScore", "PostProcess", 0);
	ppParams.scoreMode = confFile.GetInt("scoreMode", "PostProcess", 0);
	
	// Dimensions and extension of context in pixels!
	ppParams.localContextL = confFile.GetInt("localContextL", "LocalContext", 0);
	ppParams.localContextR = confFile.GetInt("localContextR", "LocalContext", 0);	
	ppParams.localContextT = confFile.GetInt("localContextT", "LocalContext", 0);
	ppParams.localContextB = confFile.GetInt("localContextB", "LocalContext", 0);	

	parseLearningParams(confFile, svmParams);
	
	return confFile;
}

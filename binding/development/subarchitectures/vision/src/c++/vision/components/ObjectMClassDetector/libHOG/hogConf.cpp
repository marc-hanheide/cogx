#include <libHOG/hogConf.h>

inifile::IniFile HOG::parseConfFile(const char* fileName, HOGParams &hogParams, preAndPostProcessParams &ppParams, HOGModel::SVMParams &svmParams) {
	inifile::IniFile confFile(true);


	using namespace std;
	
	if (!confFile.Load(fileName)) {
		std::cerr << "Configuration file " << fileName << " could not be opened!" << endl;
		return confFile;
	};
	
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

	// The object part of the detector and detector parameters
	hogParams.m_BlurSigma=confFile.GetFloat("m_BlurSigma", "HOG", 0.0);
	hogParams.m_nWindowWidth = confFile.GetInt("m_nWindowWidth", "HOG", 64);
	hogParams.m_nWindowHeight = confFile.GetInt("m_nWindowHeight", "HOG", 128);;
	
	hogParams.m_nBlockWidth=confFile.GetInt("m_nBlockWidth", "HOG", 2);
	hogParams.m_nBlockHeight=confFile.GetInt("m_nBlockHeight", "HOG", 2);
	hogParams.m_nCellWidth=confFile.GetInt("m_nCellWidth", "HOG", 8);
	hogParams.m_nCellHeight=confFile.GetInt("m_nCellHeight", "HOG", 8);
	hogParams.m_nBlockHorizontalOverlap=confFile.GetInt("m_nBlockHorizontalOverlap", "HOG", 8);
	hogParams.m_nBlockVerticalOverlap=confFile.GetInt("m_nBlockVerticalOverlap", "HOG", 8);

	//Ensure that overlap is a multiple of cellWidth/height
	hogParams.m_nBlockHorizontalOverlap = hogParams.m_nBlockHorizontalOverlap / hogParams.m_nCellWidth * hogParams.m_nCellWidth;
	hogParams.m_nBlockVerticalOverlap = hogParams.m_nBlockVerticalOverlap / hogParams.m_nCellHeight * hogParams.m_nCellHeight;

    hogParams.m_nHistogramBins=confFile.GetInt("m_nHistogramBins", "HOG", 9);
	hogParams.m_nHistogramRange=confFile.GetInt("m_nHistogramRange", "HOG", 180);;
	hogParams.m_nGaussianWindow=confFile.GetInt("m_nGaussianWindow", "HOG", 0.5);
	hogParams.m_nGammaCompression=confFile.GetInt("m_nGammaCompression", "HOG", 0);
	hogParams.m_nNormType=confFile.GetInt("m_nNormType", "HOG", 2);
	hogParams.m_dClippingThres=confFile.GetFloat("m_dClippingThres", "HOG", 0.2);
        hogParams.m_dNormalizeEpsilon=confFile.GetFloat("m_dNormalizeEpsilon", "HOG", 1.0) * 
	hogParams.m_nBlockWidth * hogParams.m_nBlockHeight * hogParams.m_nHistogramBins;
	//libhog1
        //hogParams.m_dNormalizeEpsilon=confFile.GetFloat("m_dNormalizeEpsilon", "HOG", 1.0) *
	//	hogParams.m_nBlockWidth * hogParams.m_nBlockHeight * hogParams.m_nCellWidth * hogParams.m_nCellHeight *
	//	(2 * hogParams.m_nCellWidth) * (2 * hogParams.m_nCellHeight) * 256 * hogParams.m_nHistogramRange/hogParams.m_nHistogramBins / 100;
	hogParams.m_dNormalizeEpsilonHys=confFile.GetFloat("m_dNormalizeEpsilonHys", "HOG", 0.1);
	

	cout << "Performing parameter calculation..." << endl;
	cout << "    Object part of the detector: " << hogParams.m_nWindowWidth << "x" << hogParams.m_nWindowHeight << endl;
	cout << "    Context of the detector: " << ppParams.localContextL << ", " << ppParams.localContextR << ", " << ppParams.localContextT << ", " << ppParams.localContextB << endl;

	//Determine width and height in blocks of full detector
	hogParams.m_nWindowWidth += ppParams.localContextL + ppParams.localContextR;
	hogParams.m_nWindowHeight += ppParams.localContextT + ppParams.localContextB;

	cout << "    Full size of detector: "  << hogParams.m_nWindowWidth << "x" << hogParams.m_nWindowHeight << endl;

	hogParams.m_nHOGWidth= 1 + (hogParams.m_nWindowWidth - hogParams.m_nBlockWidth * hogParams.m_nCellWidth) / (hogParams.m_nBlockWidth * hogParams.m_nCellWidth - hogParams.m_nBlockHorizontalOverlap); 
	hogParams.m_nHOGHeight= 1 + (hogParams.m_nWindowHeight - hogParams.m_nBlockHeight * hogParams.m_nCellHeight) / (hogParams.m_nBlockHeight * hogParams.m_nCellHeight - hogParams.m_nBlockVerticalOverlap);

	cout << "    Dimension in blocks: "  << hogParams.m_nHOGWidth << "x" << hogParams.m_nHOGHeight << endl;

	// Ensure that there is no rounding involved and thus that window is centered in the end
	hogParams.m_nWindowWidth = hogParams.m_nHOGWidth * ( hogParams.m_nBlockWidth * hogParams.m_nCellWidth - hogParams.m_nBlockHorizontalOverlap) + hogParams.m_nBlockHorizontalOverlap;
	hogParams.m_nWindowHeight = hogParams.m_nHOGHeight * ( hogParams.m_nBlockHeight * hogParams.m_nCellHeight - hogParams.m_nBlockVerticalOverlap) + hogParams.m_nBlockVerticalOverlap;

	cout << "    Final window dimensions of detector: "  << hogParams.m_nWindowWidth << "x" << hogParams.m_nWindowHeight << endl;
	
	svmParams.kernelType = confFile.GetInt("kernel", "SVM", 0);	
	svmParams.costFactor = confFile.GetFloat("costFactor", "SVM", 1.0);
	svmParams.slack = confFile.GetFloat("slack", "SVM", 0.01);	
	svmParams.slackCV = confFile.GetBool("slackCV", "SVM", false);
	svmParams.noRandomPerNegative = confFile.GetInt("noNegCrops", "SVM", 10);
	svmParams.noNegativeScales = confFile.GetInt("numberNegativeSamplesScaleSteps", "SVM" ,3);
        svmParams.negativeScaleStep = confFile.GetFloat("negativeSamplesScaleStep", "SVM" ,1.2);
	svmParams.minDetectionMargin = confFile.GetFloat("minDetectionMargin", "SVM", 0.0);
	svmParams.maxRetrainingSamples = confFile.GetInt("maximumRetrainingSamples", "SVM", 30000);
	svmParams.trainProbSigmoid = confFile.GetBool("fitProbSigmoid", "SVM", false);	
	return confFile;
}

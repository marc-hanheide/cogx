#include <vector>
#include <string>
#include <iostream>
#include <fstream>

#include <libHOG/HOG.h>
#include <libAnnotation/annotationlist.h>
#include <libCmd/Options.h>
#include <libPPProcess/nonmaxsuppression.h>
#include <libPPProcess/preprocess.h>
#include <opencv/highgui.h>
#include <opencv/cv.h>
#include <QApplication>

#include "parameters.h"
#include "sw_svm.h"
#include "hog_features.h"
//#include "haar_features.h"
//#include "shapelet_features.h"
//#include "denseSC_features.h"

using namespace std;

void adjustBBoxes(Annotation& annos, const preAndPostProcessParams& preprocessParams, const int windowWidth, const int windowHeight) {
	preAndPostProcessParams ppp(preprocessParams);
	
	std::cout << "Centering and adjusting annotation bounding boxes..." << std::endl;
	std::cout << "Processing: " << annos.imageName() << std::endl;
	std::cout << "Border factor: " << ppp.borderfactor << std::endl;
					
	//Enforce sliding window dimension for scaling
	ppp.scaledWidth = windowWidth;
	ppp.scaledHeight = windowHeight;
				
	std::cout << "Scaled crop dimension: " << ppp.scaledWidth << "x" << ppp.scaledHeight << std::endl;
	std::cout << "Min annotation dimension: " << ppp.minWidth << "x" << ppp.minHeight << std::endl;
	std::cout << "Max annotation dimension: " << ppp.maxWidth << "x" << ppp.maxHeight << std::endl;
				
	Annotation scaledRects(annos);
	scaledRects.clear();
	for(unsigned int j = 0; j < annos.size(); ++j) {
		AnnoRect newRect;
		if (adjustBoundingBox(annos[j], newRect, ppp))
				scaledRects.addAnnoRect(newRect);					
	};
			
	annos = scaledRects;	
}

void prepareBBoxes(AnnotationList& poslist, AnnotationList& neglist, const preAndPostProcessParams& ppParams, const SW_Params& slidingWindowParams, const LearningParams& svmParams, bool negCrops) {
	// Figure out by counting the annos whether we have crops or full scale annotations
	const int noAnnos = annoCount(poslist);
	const int noNegAnnos = annoCount(neglist);
	
	preAndPostProcessParams ppp(ppParams);
	
	/*
	 * Postive samples
	 */
	
	for(unsigned int i = 0; i < poslist.size(); ++i) {
		IplImage* img = cvLoadImage(poslist[i].imageName().c_str());
		
		if (img == 0) {
			fprintf(stderr, "ERROR: Failed loading image: %s\n", poslist[i].imageName().c_str());
			continue;
		}
			
		// Full scale annotations -> ensure the correct aspect ratio
		if (noAnnos > 0) {
			adjustBBoxes(poslist[i], ppp, slidingWindowParams.windowWidth, slidingWindowParams.windowHeight);
			
			if (poslist[i].size() > 0)
				prepareBoundingBoxes(poslist[i], ppp, img->width, img->height, slidingWindowParams.windowWidth, slidingWindowParams.windowHeight, true, false);

		} else { //crops
			// Adds the boundary pixel and jitter to annotation (make sure AnnoRect.scale is set correctly)!!
			prepareBoundingBoxes(poslist[i], ppp, img->width, img->height, slidingWindowParams.windowWidth, slidingWindowParams.windowHeight, true, false);
		}
		cvReleaseImage(&img);
	}
			
	/*
	 * Negative samples
	 */
	
	for (unsigned int i = 0; i < neglist.size(); ++i) {
		IplImage* img = cvLoadImage(neglist[i].imageName().c_str());
		
		if (img == 0) {
			fprintf(stderr, "ERROR: Failed loading image: %s\n", neglist[i].imageName().c_str());
			continue;
		}
					
		if (negCrops)
			prepareBoundingBoxes(neglist[i], ppp, img->width, img->height, slidingWindowParams.windowWidth, slidingWindowParams.windowHeight, true, true);
		else {
			
			// If we have full images and no annotations draw random samples
			if (noNegAnnos == 0) 
				drawRandomSamples(neglist[i], img->width, img->height, svmParams.noRandomPerNegative, svmParams.negativeScaleStep, svmParams.noNegativeScales,slidingWindowParams.windowWidth, slidingWindowParams.windowHeight);
			else {
				// otherwise we ensure the correct aspect ratio
				adjustBBoxes(neglist[i], ppp, slidingWindowParams.windowWidth, slidingWindowParams.windowHeight);
				
				if (neglist[i].size() > 0)
					prepareBoundingBoxes(neglist[i], ppp, img->width, img->height, slidingWindowParams.windowWidth, slidingWindowParams.windowHeight, true, false);

			}
						
			if (neglist[i].size() > 0)
				prepareBoundingBoxes(neglist[i], ppp, img->width, img->height, slidingWindowParams.windowWidth, slidingWindowParams.windowHeight, false, false);
		}	
		
		cvReleaseImage(&img);
	}	
}

void prepareFeatures(std::vector<FeatureVisitor*>& features, const inifile::IniFile& confFile, SW_Params& sw_params) {
	
	if (sw_params.features.find("HOG") != string::npos) {		
		HOGParams hogP;
		hogP.m_nWindowWidth = sw_params.windowWidth;
		hogP.m_nWindowHeight = sw_params.windowHeight;
		parseHOG_parameters(confFile, hogP);
		features.push_back(new HOG_Visitor(hogP));		
	}
	
	/*if (sw_params.features.find("HaarVJ") != string::npos) {		
		HaarParams haarP;
		haarP.windowWidth = sw_params.windowWidth;
		haarP.windowHeight = sw_params.windowHeight;
		parseHaar_parameters(confFile, haarP, CascadeFile);
		features.push_back(new Haar_Visitor(haarP));		
	}
	
	if (sw_params.features.find("HaarPoggio") != string::npos) {		
		HaarParams haarP;
		haarP.windowWidth = sw_params.windowWidth;
		haarP.windowHeight = sw_params.windowHeight;
		parseHaar_parameters(confFile, haarP, Poggio);
		features.push_back(new Haar_Visitor(haarP));		
	}
	
	if (sw_params.features.find("Shapelet") != string::npos) {		
		SW_ShapeletParams shapeletParam;		
		shapeletParam.m_WindowWidth = sw_params.windowWidth;
		shapeletParam.m_WindowHeight = sw_params.windowHeight;
		parseShapelet_parameters(confFile, shapeletParam);		
		features.push_back(new Shapelet_Visitor(shapeletParam));		
	}
	
	if (sw_params.features.find("DSC") != string::npos) {		
		DenseSCParams denseSCParam;		
		denseSCParam.windowWidth = sw_params.windowWidth;
		denseSCParam.windowHeight = sw_params.windowHeight;
		parseDenseSC_parameters(confFile, denseSCParam);		
		features.push_back(new DenseSC_Visitor(denseSCParam));		
	} */
	
}


void freeFeatures(std::vector<FeatureVisitor*>& features) {
	for (unsigned int i = 0; i < features.size(); ++i)
		delete features[i];
	
	features.clear();
}


int main(int argc, char** argv) {	
	QApplication q(argc,argv, false);
	srand(42);

	//---------------------------------------------------------------------//	
	//--- 		Command Line Option parsing							 	---//	 
	//---------------------------------------------------------------------//
	
	
	static char* helptext[] = { "Object detection with multiple features and SVM",
								"This tool supports training as well as testing", NULL, NULL};
								
	static Cmd_ns::BoolOption training("train", "Use this tool to train a classifier");
	static Cmd_ns::BoolOption bootstrap("bootstrap", "Use this tool to bootstrap train a classifier");
	static Cmd_ns::BoolOption fileTraining("fileTrain", "Use this tool to train a classifier from a given featureFile");
	static Cmd_ns::BoolOption testing("test", "Use this tool to test with a classifier");
	static Cmd_ns::BoolOption nonmax("nonmax", "Use this tool to run the non-maximum suppression");
	static Cmd_ns::BoolOption featureExtract("features", "Use this to run featureExtraction only");	
	static Cmd_ns::BoolOption preProcessOnly("preProcess", "Only preprocesses the annotations and stores crops in /tmp");
	static Cmd_ns::FileOption configFile("confFile", "Specifies a config file", true);
	static Cmd_ns::FileOption posFile("posFile", "Specifies positive annotations in IDL format (with annotations or taken as center crops)", false);
	static Cmd_ns::FileOption negFile("negFile", "Specifies negative training images (with annotations or otherwise randomly sampled)", false);
	static Cmd_ns::FileOption negCropFile("cropNegFile", "Specifies negative training images as center crop images", false);
	static Cmd_ns::FileOption outFile("outFile", "Specifies the output file name", true);
	static Cmd_ns::FileOption modelname("model", "The file holding the SVM model", false);
	static Cmd_ns::FileOption testImage("image", "The image file name to be tested", false);
	
	static Cmd_ns::FloatOption minScaleFactor("minScale", "Minimum scale while testing", 0.0, 1000.0, false);
	static Cmd_ns::FloatOption maxScaleFactor("maxScale", "Maximum scale while testing", 0.0, 1000.0, false);
			
	static Cmd_ns::FloatOption costFactor("svmCostFactor", "The cost factor of negative samples for SVM training", 0.0, 1000.0, false);
	static Cmd_ns::IntOption noGrids("noGrids", "The number of overlapping grids", 1, 100, false);
	static Cmd_ns::BoolOption noHardExamples("skipHardExamples", "Do not train on hard examples (i.e. false positives on negative samples)");
	
	static Cmd_ns::FloatOption smoothX("smoothX", "Bandwith for smoothing in X direction", 0.0, 1000.0, false);
	static Cmd_ns::FloatOption smoothY("smoothY", "Bandwith for smoothing in Y direction", 0.0, 1000.0, false);
	static Cmd_ns::FloatOption smoothS("smoothS", "Bandwith for smoothing in scale", 0.0, 1000.0, false);
	static Cmd_ns::FloatOption minNMSScore("minNMSScore", "Minimum detection score for non-maximum suppression", -1000.0, 1000.0, false);
	static Cmd_ns::IntOption minDetPerMode("minDetPerMode", "Minimum detections per valid mode", 0, 1000, false);
	static Cmd_ns::IntOption scoreMode("scoreMode", "Non-maximum-suppression score mode", 0, 2, false);
								
	Cmd_ns::CmdParser myCmdParser = Cmd_ns::CmdParser::ProcessArgs(helptext, argc, argv);
	
	SW_Params slidingWindowParams;
	preAndPostProcessParams ppParams;
	LearningParams learningParams;
	
	if (!fileExists(static_cast<const char*>(configFile))) {
		std::cout << "Configuration file " << static_cast<const char*>(configFile) << "does not exist!" << std::endl;
		return 1;
	}
	
	if (negFile.isspecified() && negCropFile.isspecified()) {
		std::cout << "Please specify either -negCropFile or -negFile!" << std::endl;
		return 1;
	}
	
	// Get the parameters from config file and ...
	const inifile::IniFile conf = parseConfFile(static_cast<const char*>(configFile), slidingWindowParams, ppParams, learningParams);
	
	// ... set up the features
	std::vector<FeatureVisitor*> features;
	prepareFeatures(features, conf, slidingWindowParams);
	
	
	//---------------------------------------------------------------------//	
	//--- 		Parameters from config File can be overwritten here 	---//	 
	//---------------------------------------------------------------------//
		 
	if (outFile.isspecified() && (training || fileTraining || bootstrap))
		learningParams.modelFile = std::string(static_cast<const char*>(outFile));
	else
		learningParams.modelFile = "tmp.model";
				
	if (costFactor.isspecified())
		learningParams.svm_costFactor =  static_cast<float>(costFactor);
	
	if (!minScaleFactor.isspecified()) {
		minScaleFactor.setvalue(1.0);
	}
	
	if (!maxScaleFactor.isspecified()) {
		maxScaleFactor.setvalue(100.0);
	}
	
	// ------------ Overwrite NMS parameters if set on command line
	
	if (smoothX.isspecified()) {
		ppParams.smoothingBandWidthX = smoothX;
	}
	
	if (smoothY.isspecified()) {
		ppParams.smoothingBandWidthY = smoothY;
	}
	
	if (smoothS.isspecified()) {
		ppParams.smoothingBandWidthScale = smoothS;
	}
	
	if (minNMSScore.isspecified()) {
		ppParams.minNMSScore = minNMSScore;		
	}
	
	if (minDetPerMode.isspecified()) {
		ppParams.minDetectionsPerMode = minDetPerMode;
	}
	
	if (scoreMode.isspecified()) {
		ppParams.scoreMode = scoreMode;		
	}
		
	if(!noGrids.isspecified())
		noGrids.setvalue(1);
	
	
	
	
	// Prepare the boundingBoxes which might not be specified in the IDL file
	// For negative we have the choice of random crops, specified crops (given bounding  box)
	// and center crops, for postive samples only center crops and specified crops make sense
	AnnotationList poslist;
	AnnotationList neglist;
	if (training || featureExtract || bootstrap) {
				
		// Check that we have positive samples
		if (!posFile.isspecified() || !fileExists(posFile)) {
			cout << "Please specify (valid) positive training samples !" << endl;
			freeFeatures(features);
			return 0;
		}
		poslist = AnnotationList(posFile);
		
		// Check that we have either negative crops or full images
		if (!negFile.isspecified() && !negCropFile.isspecified() || !fileExists(negFile) && !fileExists(negCropFile) ) {
			std::cout << "Please specify (valid) negative training samples!" << std::endl;
			freeFeatures(features);
			return 0;
		}
		
		if (negFile.isspecified())
			neglist = AnnotationList(negFile);
		else
			neglist = AnnotationList(negCropFile);
		
		prepareBBoxes(poslist, neglist, ppParams, slidingWindowParams, learningParams, negCropFile.isspecified());
		
		//Debugging
		//poslist.save("/tmp/poslist.idl");
		//neglist.save("/tmp/neglist.idl");		
	}
	

	//---------------------------------------------------------------------//
	//---                        prepocess only                            //
	//---------------------------------------------------------------------//
	if (preProcessOnly) {
		printf("Preprocessing... \n");
		
		if (!posFile.isspecified()) {
			cout << "Please specify IDL file to preprocess as positive training samples!" << endl;
			freeFeatures(features);
			return 0;
		}			
		
		AnnotationList poslist(posFile);
		preProcess(poslist, ppParams, outFile);
		freeFeatures(features);
		return 0;
	}
		
	//---------------------------------------------------------------------//
	//---                         train                                 ---//
	//---------------------------------------------------------------------//
	if (training) {
		printf("Classifier Training ...\n");
				
		SW_Train swt(slidingWindowParams, learningParams, ppParams);
		swt.addFeature(features);
		
		swt.extractSamples(poslist, neglist);
		
		//swt.saveSamplesAsText("/tmp/test.svm");
		
		std::string negs = learningParams.modelFile + "-withoutHardExamples-Negatives.idl";
		neglist.saveIDL(negs);
		
		if(!swt.train())
			return 1;		
		swt.saveSupportVectors(poslist, neglist, false);
						
		//Find the hard examples and retrain
		if(noHardExamples == false) {
			swt.extractHardSamples(neglist);
			if(!swt.train(true))
				return 1;
			swt.saveSupportVectors(poslist, neglist, true);
		}
		freeFeatures(features);
		return 0;
	}
	
	//---------------------------------------------------------------------//
	//---                         bootstrap only                        ---//
	//---------------------------------------------------------------------//
	if (bootstrap) {
		printf("BootStrap Training ...\n");
				
		SW_Train swt(slidingWindowParams, learningParams, ppParams);
		swt.addFeature(features);
		
		bool binarymodel = true;		
		if (!swt.load_ClassifierModel(binarymodel)) {
			freeFeatures(features);
			return 1;	
		}
		
		swt.extractSamples(poslist, neglist);
		//swt.setSVMCostRatio();
		
		//Find the hard examples and retrain
		swt.extractHardSamples(neglist);
		if(!swt.train(true))
			return 1;
		swt.saveSupportVectors(poslist, neglist, true);
		
		freeFeatures(features);
		return 0;
	}
	
	//---------------------------------------------------------------------//
	//---                         file train hog                        ---//
	//---------------------------------------------------------------------//
	if (fileTraining) {
		printf("Training ...\n");
					
		if (!posFile.isspecified()) {
			cout << "Please specify input feature file in pos parameter!" << endl;
			freeFeatures(features);
			return 0;
		}			
		
		SW_Train swt(slidingWindowParams, learningParams, ppParams);
		swt.addFeature(features);
				
		swt.loadFeatures(posFile);
		
		if(!swt.train(true))
			return 1;		
		//swt.saveSamplesAsText("/tmp/test.svm");
			
		freeFeatures(features);
		return 0;
	}

	//---------------------------------------------------------------------//
	//---                         featureExtract                        ---//
	//---------------------------------------------------------------------//
	if (featureExtract) {
		printf("Feature Extraction ...\n");
						
		//Set Classifier to SVM since saving methods are only supported by SVM data structures
		slidingWindowParams.classifier = SVM;
		
		SW_Train ht(slidingWindowParams, learningParams, ppParams);
		ht.addFeature(features);		
		
		ht.extractSamples(poslist, neglist);
		
		ht.saveSamples(outFile);
		freeFeatures(features);
		return 0;
	}
	
	
	//---------------------------------------------------------------------//
	//---                         test hog                              ---//
	//---------------------------------------------------------------------//
	if (testing) {
		
		printf("Testing ...\n");
				
		bool binarymodel = true;
		float scaleStep = slidingWindowParams.scaleStep - 1.0; 
		int scaleType = 1; //0 = linear, 1 = exponential
		
		learningParams.modelFile = std::string(static_cast<const char*>(modelname));
		
		SW_Test ht(slidingWindowParams, learningParams, ppParams);
		ht.addFeature(features);
		
		if (!ht.load_ClassifierModel(binarymodel)) {
			freeFeatures(features);
			return 1;	
		}
		
		ht.setMinDetectionMargin(learningParams.minDetectionMargin);
		
		string::size_type pos = std::string(static_cast<const char*>(testImage)).rfind(".");
		string::size_type posslash = std::string(static_cast<const char*>(testImage)).rfind("/");
		string extension = std::string(static_cast<const char*>(testImage)).substr(pos,string::npos);
		if (posslash==string::npos) posslash=0; else posslash++;
		string saveName;

		AnnotationList initialHypos, finalHypos;
		int noWindows = 0;
		if (strcmp(extension.c_str(),".idl")==0) // idl-file
		{
			AnnotationList inputList(testImage);
			initialHypos = inputList;			
			noWindows = ht.testAllMultiScale(initialHypos, static_cast<int>(noGrids), static_cast<float>(minScaleFactor), static_cast<float>(maxScaleFactor), scaleStep, scaleType, std::string(static_cast<const char*>(outFile))+"-result.idl");
		}
		else if (strcasecmp(extension.c_str(), ".png") ==0 || strcasecmp(extension.c_str(), ".jpg") ==0) // png-File
		{			
			initialHypos.addAnnotation(Annotation(static_cast<const char*>(testImage)));
			ht.testImageMultiScale(testImage, initialHypos[0], static_cast<int>(noGrids), static_cast<float>(minScaleFactor), static_cast<float>(maxScaleFactor), scaleStep, scaleType);
			initialHypos[0].sortByScore();
		}

		//saveName = string("result-")+std::string(static_cast<const char*>(testImage)).substr(posSlash,pos)+".idl";
		saveName = std::string(static_cast<const char*>(outFile))+"-result.idl";
		initialHypos.save(saveName);
		
		inifile::IniFile saveConf(conf);
		saveConf.SetFileName(std::string(static_cast<const char*>(outFile))+"-result.conf");
		
		saveConf.CreateSection("Runtime Information");
		char now[500];
		sprintf(now, "%d", noWindows);
		saveConf.CreateKey("noWindows", now, "", "Runtime Information");		
		saveConf.Save();
		
		
		//--- NonMax supression ---//		
		std::vector<float> bandwidth(3);
		bandwidth[0] = ppParams.smoothingBandWidthX;
		bandwidth[1] = ppParams.smoothingBandWidthY;
		bandwidth[2] = ppParams.smoothingBandWidthScale;
		NonMaxSuppresion nms(bandwidth);

		saveName = std::string(static_cast<const char*>(outFile))+"-result-nms.idl";
		for(unsigned int i=0; i<initialHypos.size(); ++i)
		{
			cout << "Processing: " << initialHypos[i].imageName() << endl;
			
			Annotation valid;
			valid.setImageName(initialHypos[i].imageName());
			if (initialHypos[i].isStream()) valid.setFrameNr(initialHypos[i].frameNr());
			
			for(unsigned int j = 0; j < initialHypos[i].size(); j++) {
				if(initialHypos[i][j].score() >= ppParams.minNMSScore) 
					valid.addAnnoRect(initialHypos[i][j]);
			}
			nms.addDataPoints(valid);
			
			Annotation finalAnno; 
			if (initialHypos[i].isStream()) finalAnno.setFrameNr(initialHypos[i].frameNr());
			finalAnno.setImageName(initialHypos[i].imageName());
			nms.getModes(finalAnno, ppParams.minDetectionsPerMode, 0.01, 100, ppParams
					.scoreMode);
			
			finalHypos.addAnnotation(finalAnno);
			finalHypos.save(saveName);
		}
		
		freeFeatures(features);
		return 0;
	}
	//---------------------------------------------------------------------//
	//---                     nonmax suppression                         ---//
	//---------------------------------------------------------------------//
	if (nonmax)
	{
		AnnotationList initialHypos(testImage);
		for(unsigned int i=0; i<initialHypos.size(); ++i)
		{
			for(unsigned int j=0; j<initialHypos[i].size(); ++j)
			{
				AnnoRect& r = initialHypos[i][j];
				r.setScale((0.5f * r.w() / slidingWindowParams.windowWidth) + ( 0.5f * r.h() / slidingWindowParams.windowHeight));
			}
		}
		AnnotationList finalHypos;

		//--- NonMax supression ---//		
		std::vector<float> bandwidth(3);
		bandwidth[0] = ppParams.smoothingBandWidthX;
		bandwidth[1] = ppParams.smoothingBandWidthY;
		bandwidth[2] = ppParams.smoothingBandWidthScale;
		NonMaxSuppresion nms(bandwidth);

		string saveName = std::string(static_cast<const char*>(outFile))+"-result-nms.idl";
		for(unsigned int i=0; i < initialHypos.size(); ++i)
		{
			cout << "Processing: " << initialHypos[i].imageName();
			
			Annotation valid;
			valid.setImageName(initialHypos[i].imageName());
			if (initialHypos[i].isStream()) valid.setFrameNr(initialHypos[i].frameNr());
			
			for(unsigned int j = 0; j < initialHypos[i].size(); j++) {
				if(initialHypos[i][j].score() >= ppParams.minNMSScore) 
					valid.addAnnoRect(initialHypos[i][j]);
			}
			cout << " with " << valid.size() << " bounding boxes..." << endl;
			nms.addDataPoints(valid);
			
			Annotation finalAnno; 
			if (initialHypos[i].isStream()) finalAnno.setFrameNr(initialHypos[i].frameNr());
			finalAnno.setImageName(initialHypos[i].imageName());
			nms.getModes(finalAnno, ppParams.minDetectionsPerMode, 0.01, 100, ppParams.scoreMode);
			
			finalHypos.addAnnotation(finalAnno);
			finalHypos.save(saveName);
		}
		
		freeFeatures(features);
		return 0;

	}
}

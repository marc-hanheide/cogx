#ifndef SW_SVM_H
#define SW_SVM_H

#include <libAnnotation/annotationlist.h>
#include <libPPProcess/preprocess.h>
#include <libSVMdense/svm.h>
#include <libFeatures/featurevector.hh>
//#include <libJointBoosting/AdaBoost.h>
#include <opencv/cv.h>
#include <string>

#include "features_sw.h"
#include "parameters.h"

class SW_Test {
	public:
		SW_Test(const SW_Params& sw_params, const LearningParams& svm_params, const preAndPostProcessParams& pp_params) : 
			m_pTestedImage(0),
			m_swParams(sw_params),
			m_LearningParams(svm_params),
			m_preProcessParams(pp_params),
			m_featureDim(0) {};
			
		~SW_Test();
		
		bool load_ClassifierModel(bool binarymodel);		
		
		int testImageMultiScale(std::string fileName, Annotation& anno, const int noSteps=2, float minScale = 1.0, float maxScale = 100.0, float scaleStep = 0.05, int scaleType = 1);
		//int testImageMultiScaleFromStream(Annotation& anno, const int noSteps=2, float minScale = 1.0, float maxScale = 100.0, float scaleStep = 0.05, int scaleType = 1);
		int testAllMultiScale(AnnotationList& list, const int noSteps=2, float minScale = 1.0, float maxScale = 100.0, float scaleStep = 0.05, int scaleType = 1, std::string incrementalStore = "");
				
  		void setMinDetectionMargin(const float minMargin) {m_dMinMargin = minMargin;}
		int testImageMultiScale(IplImage* img, Annotation& anno, const int noSteps=2, float minScale = 1.0, float maxScale = 10.0, float scaleStep = 0.05, int scaleType = 1);
		int testImage_slidingWindow(IplImage* img, Annotation& anno, const float scale=1.0, const float xoffset=0, const float yoffset=0, float xbound = 0, float ybound = 0);
		int testImage(IplImage* img, Annotation& anno, const int noSteps=2, const float scale=1.0, float xbound = 0, float ybound = 0);
		
		void load_TradeOffModel(const char* fileName);
				
		void addFeature(FeatureVisitor* newFeature);
		void addFeature(std::vector<FeatureVisitor*>& newFeature);
		void clearFeatures();
		
		void normalizeCues(float* fv);
				
	private:
		std::vector<std::vector<double> > m_w;
		std::vector<double> m_d;
		double m_b;
			
					
  		float m_dMinMargin;
  		unsigned char* m_pTestedImage;
  		
  	protected:
  		SVMlight m_svm;
  		//AdaBoostClassifier m_boost;
  		const SW_Params& m_swParams;
  		LearningParams m_LearningParams;
  		preAndPostProcessParams m_preProcessParams;
  		
  		std::vector<FeatureVisitor*> features;
  		int m_featureDim;
  		std::vector<FeatureVector> m_boostingData;
  		std::vector<float> maxNormalizer;
  		
};


class SW_Train : public SW_Test {
	public:
		SW_Train(const SW_Params& sw_params, const LearningParams& learning_params, const preAndPostProcessParams& pp_params);
		
		bool train(bool withHardExamples = false);
		void extractHardSamples(const AnnotationList& negFileList);
		void extractSamples(const AnnotationList& posFileList, const AnnotationList& negFileList);
		
		void saveSupportVectors(const AnnotationList& posFileList, const AnnotationList& negFileList, bool containsHardExamples);
		void saveSamples(std::string fileName) {m_svm.save_features(fileName.c_str());}
		void saveSamplesAsText(std::string fileName) {m_svm.save_features_text(fileName.c_str());}
		void loadFeatures(const char* fileName);
		//void setSVMCostRatio() {m_svm.learn_parm.svm_costratio = 1.0f * negSamples / posSamples;}
		
	protected:
		IplImage* getCrop(IplImage* img, const AnnoRect& anno); 
		bool trainSVM(bool withHardExamples = false);
		//void trainAdaBoost(bool withHardExamples = false);
				
		int posSamples;
		int negSamples;
		
		std::vector<AnnoRect> m_NegRects;
		std::vector<int> m_NegRectFileIndex;
		std::vector<AnnoRect> m_PosRects;
		std::vector<int> m_PosRectFileIndex;	  	
};

#endif

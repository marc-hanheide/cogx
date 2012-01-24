

//------------------------------------------------------
// Sequential AdaBoost Classifier for place recognition
//
// oscar martinez-mozos
//------------------------------------------------------


#ifndef __SEQUENTIAL_ADABOOST_CLASSIFIER__
#define __SEQUENTIAL_ADABOOST_CLASSIFIER__

#include "dynamictable.h"
#include "rangeExampleGeneral.h"
#include "strongClassifierGeneralized.h"

using namespace oscar;

typedef DynamicTable<Feature> dyntab_features;

class SequentialAdaBoostClassifier {

	public:
		// constructor
		SequentialAdaBoostClassifier();		
		
		// destructor
		~SequentialAdaBoostClassifier();
		
		// methods
		int setHypotheses(int num_classes, char **files);
		void setDebug(bool debug);

		int configureHypotheses(const char *file);
		
		//---------------------------------------------------------------------------------
		//
		// -------------------------------------------------------------------------------- 
		int extractFeatures(dyntab_features *features, RangeExample *e);		
		
		//---------------------------------------------------------------------------------
		// print parameters of feature *f
		// -------------------------------------------------------------------------------- 
		void printParameters(Feature *f, ostream &os);	
			
			
		//---------------------------------------------------------------------------------
		// Parameters:
		//		INPUT:
		//    		frontLaser, rearLaser: lasers in CARMEN format
		//			featureList: list of features to calculate
		// 		OUTPUT :  
		// 			features : list of calculated features.  
		//						     Memory is allocated inside the function. The user must free it.	
		//		RETURNS
		//			<  0   if an error occurs
		//			>= 0   if OK	
		// -------------------------------------------------------------------------------- 
		int createSelectedFeatures(dyntab_featuresInfo *featureList,
										dyntab_features *features);
		
		//---------------------------------------------------------------------------------
		// Parameters:
		//		INPUT:
		//    		re: example
		//			featureList: list of features to calculate
		// 		OUTPUT :  
		// 			featureList : the calculate values are store here
		//		RETURNS
		//			<  0   if an error occurs
		//			>= 0   if OK	
		// -------------------------------------------------------------------------------- 
		int calcSelectedFeatures(RangeExampleGeneral *re,
										dyntab_featuresInfo *featureList);
																		
		//---------------------------------------------------------------------------------
		// Parameters:
		//		INPUT:
		//    		frontLaser, rearLaser: lasers in CARMEN format
		//			featureList: list of features to calculate
		// 		OUTPUT :  
		// 			featureList : the calculate values are store here
		//		RETURNS
		//			<  0   if an error occurs
		//			>= 0   if OK	
		// -------------------------------------------------------------------------------- 
		int calcSelectedFeatures(carmen_robot_laser_message *frontLaser, 
										carmen_robot_laser_message *rearLaser,
										dyntab_featuresInfo *featureList);


		//---------------------------------------------------------------------------------
		// Parameters:
		//		INPUT:
		//    		frontLaser, rearLaser: lasers in CARMEN format
		//			featureList: list of features to calculate
		// 		OUTPUT :  
		// 			featureList : the calculate values are store here
		//		RETURNS
		//			<  0   if an error occurs
		//			>= 0   if OK	
		// -------------------------------------------------------------------------------- 
		int calcSelectedFeatures(int numFrontReadings, float *frontLaser, 
										int numRearReadings, float *rearLaser,
										dyntab_featuresInfo *featureList);				
		
		//---------------------------------------------------------------------------------
		// Parameters:
		//		INPUT:
		//    		numWeakHypotheses: number of weak hypotheses used in each binary classifier
		//    		frontLaser, rearLaser: lasers in CARMEN format
		// 		OUTPUT :  
		// 			classConfidence: array where the confidence of each class is store
		//			confidence     : confidence of the selected class
		//		RETURNS:
		//			>=  0 the classification	 
		//			<   0  error 
		// -------------------------------------------------------------------------------- 
		int classify(int numWeakHypotheses, carmen_robot_laser_message *frontLaser, 
					 carmen_robot_laser_message *rearLaser, double *classConfidence,
					 double *confidences); 

		
		//---------------------------------------------------------------------------------
		// Parameters:
		//		INPUT:
		//    		numWeakHypotheses: number of weak hypotheses used in each binary classifier
		//    		frontLaser, rearLaser: arrays of num[Front|Rear]Readings
		// 		OUTPUT :  
		// 			classConfidence: array where the confidence of each class is store
		//			confidence     : confidence of the selected class
		//		RETURNS:
		//			>=  0 the classification	 
		//			<   0  error 
		// -------------------------------------------------------------------------------- 
		int classify2(int numWeakHypotheses, int numFrontReadings, double *frontLaser, 
					  int numRearReadings, double *rearLaser, double *classConfidence, 
					  double *confidences);

/*
		int classifyFeatures(int num_weak_hypotheses,
				     PlaceData::LaserFeatures& featureVector,
				     double *class_confidence,  
				     double *confidences);	
*/
		int getNumClassifiers();				  					 					 		

	private:

		// methods
		void processExample(oscar::StrongClassifierGeneralized *sc,  oscar::RangeExampleGeneral *e);
		void extractFeatures(oscar::StrongClassifierGeneralized *sc, oscar::RangeExampleGeneral *e);
	
		// data	
		oscar::StrongClassifierGeneralized *sc;
		int num_classes;
		int binary_classifiers;
		bool debug;
};

#endif // __SEQUENTIAL_ADABOOST_CLASSIFIER__



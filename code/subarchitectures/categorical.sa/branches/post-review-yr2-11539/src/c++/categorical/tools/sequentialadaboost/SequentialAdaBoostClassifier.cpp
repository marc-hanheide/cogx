//------------------------------------------------------
// Sequential AdaBoost Classifier for place recognition
//
//  oscar martinez mozos
//------------------------------------------------------


// leave "..." because using <...> reads the .h form ../../include and I will get errors compiling new versions
#include "SequentialAdaBoostClassifier.h"
#include "features/all_features.h"

#include <vector>
#include <string.h>

using namespace oscar;

const int FC=100;

Feature13 f13;
Feature20 f20;

//------------------------------------------------------
// constructor
//------------------------------------------------------
SequentialAdaBoostClassifier::SequentialAdaBoostClassifier() {
	debug = false;
	sc = NULL;
}


//------------------------------------------------------
// destructor
//------------------------------------------------------
SequentialAdaBoostClassifier::~SequentialAdaBoostClassifier() {
	delete [] sc; // delete strong classifier
}


//------------------------------------------------------
// debug
//------------------------------------------------------
void SequentialAdaBoostClassifier::setDebug(bool d) {
	debug = d;
}


int SequentialAdaBoostClassifier::getNumClassifiers() {
  return binary_classifiers + 1;
}


//------------------------------------------------------
// load de hypotheses files. One file for each class
//
// return:
//      -1 error
//      1 ok
//------------------------------------------------------
int SequentialAdaBoostClassifier::setHypotheses(int num_classes, char **files) {

	if ( sc != NULL ) {
		delete [] sc; // delete strong classifier
	}

	this->num_classes = num_classes;
	this->binary_classifiers = num_classes-1;
	sc = new StrongClassifierGeneralized[binary_classifiers];

	// read the binary classifiers
	// we don't need the last one
	for(int level=0; level < binary_classifiers; level++) {
		//-------------------
		// level i
		//-------------------
		char *hypotheses_file = files[level];

		// read hypotheses
		ifstream ifs;
		ifs.open(hypotheses_file);

		if ( debug ) {
			cerr << "hypothesis file level " << level << " : " << hypotheses_file << endl;
		}
		if ( !ifs.is_open() ) {
			cerr << "ERROR: opening hypotheses file: "  << hypotheses_file << endl;
			return -1;
		}
		sc[level].debug = this->debug;
		sc[level].readHypotheses(ifs);
		ifs.close();
		ifs.clear();

		// change index
		for(int j=0; j< (sc[level].hypotheses)->num(); j++ ) {
			ThresholdHypothesisGeneral *h;
			h = (sc[level].hypotheses)->getElement(j);
			h->index = j;
		}
		if ( debug ) {
			cerr << "total hypotheses for level " << level << ": "
				<< (sc[level].hypotheses)->num() << endl;
		}
	}// end for(int level=0; level<num_classes; level++)

	return 1; //all ok
}

int SequentialAdaBoostClassifier::configureHypotheses(const char *file) {
  // read hypotheses
  ifstream ifs;
  ifs.open(file);

  if ( !ifs.is_open() ) {
    cerr << "ERROR:SequentialAdaBoostClassifier: opening hypotheses file: "  << file << endl;
    return -1;
  }

  string line("");

  vector< string > hypothesisFiles;

  while(!ifs.eof()) {
    getline(ifs, line);

    if (line.empty())
      continue;

    if(line.compare("") == 0)
      continue;

    if(line[0] == '#')
      continue;

    hypothesisFiles.push_back(line);
  }

  if ( sc != NULL ) {
    delete [] sc; // delete strong classifier
  }

  num_classes = hypothesisFiles.size();
  binary_classifiers = num_classes - 1;
  sc = new StrongClassifierGeneralized[binary_classifiers];

  // read the binary classifiers
  // we don't need the last one

  for (int level = 0; level < binary_classifiers; level++) {
      // read individual hypothesis files
      ifstream ifs;
      ifs.open(hypothesisFiles[level].c_str());

      if ( !ifs.is_open() ) {
	cerr << "ERROR:SequentialAdaBoostClassifier opening hypothesis file: "  << hypothesisFiles[level].c_str() << endl;
	return -1;
      }
      sc[level].debug = this->debug;
      sc[level].readHypotheses(ifs);
      ifs.close();
      ifs.clear();

      // change index
      for(int j=0; j< (sc[level].hypotheses)->num(); j++ ) {
	ThresholdHypothesisGeneral *h;
	h = (sc[level].hypotheses)->getElement(j);
	h->index = j;
      }
    } // end for(int level=0; level<num_classes; level++)

   return 1; //all ok

}

//--------------------------------------------------
// calculate features for one example
//--------------------------------------------------
void SequentialAdaBoostClassifier::extractFeatures(StrongClassifierGeneralized *sc, RangeExampleGeneral *e) {

	Feature *f;
	ThresholdHypothesisGeneral *h;

	if (debug)
		cerr << "SequentialAdaBoostClassifier::extractFeatures: 1" << endl;

	// f13 needed by 16,17,18,19
	(sc->f13).getFeature(e, -1);

	if (debug)
		cerr << "SequentialAdaBoostClassifier::extractFeatures: 2" << endl;

	// f20 needed by 21, 22, 23
	(sc->f20).getFeature(e, -1);

	if (debug)
		cerr << "SequentialAdaBoostClassifier::extractFeatures: 3" << endl;




	for (int i=0; i<(sc->hypotheses)->num(); i++) {
		h = (sc->hypotheses)->getElement(i);
		f = h->feature;
		f->getFeature(e, i);

		if (debug)
			cerr << "SequentialAdaBoostClassifier::extractFeatures: getFeatures: " << i << endl;

	}

	if (debug)
		cerr << "SequentialAdaBoostClassifier::extractFeatures: 4" << endl;


	return;
}


//--------------------------------------------------
// processExample
//--------------------------------------------------
void SequentialAdaBoostClassifier::processExample(StrongClassifierGeneralized *sc,  RangeExampleGeneral *e) {

	if ( debug )
		cerr << "SequentialAdaBoostClassifier::processExample: 1" << endl;

	e->setNumFeatures( (sc->hypotheses)->num() );

	if ( debug )
		cerr << "SequentialAdaBoostClassifier::processExample: 2" << endl;


	extractFeatures( sc, e );

	return;
}



//------------------------------------------------------
// classify the laser
//
// return:
//      >0 classification [0...num_classes-1]
//      -1 error
//------------------------------------------------------
int SequentialAdaBoostClassifier::classify2(int numWeakHypotheses, int numFrontReadings, double *frontLaser,
											int numRearReadings, double *rearLaser, double *classConfidence,
											double *confidences) {

	carmen_robot_laser_message front, rear;

	//std::cerr << "classify2 1 " << std::endl;


	front.num_readings = numFrontReadings;
	front.range = new float[front.num_readings];
	for(int i=0; i<front.num_readings; i++) {
		front.range[i] = (float)(frontLaser[i]);
	}

	//std::cerr << "classify2 2 " << std::endl;


	rear.num_readings = numRearReadings;
	rear.range = new float[rear.num_readings];
	for(int i=0; i<rear.num_readings; i++) {
		rear.range[i] = (float)(rearLaser[i]);
	}

	//std::cerr << "classify2 3 " << std::endl;

	int ret;
	ret = classify(numWeakHypotheses, &front, &rear, classConfidence, confidences);

	//std::cerr << "classify2 4 " << std::endl;


	delete [] front.range;

	//std::cerr << "classify2 5 " << std::endl;

	delete [] rear.range;

	//std::cerr << "classify2 6 " << std::endl;

	return ret;
}



//------------------------------------------------------
// classify the laser
//
// return:
//      >0 classification [0...num_classes-1]
//      -1 error
//------------------------------------------------------
int SequentialAdaBoostClassifier::classify(int num_weak_hypotheses,
										   carmen_robot_laser_message *frontlaser,
										   carmen_robot_laser_message *rearlaser,
										   double *class_confidence,
										   double *confidences) {

	if ( confidences == NULL ) {
		cerr << "ERROR: SequentialAdaBoostClassifier::classify: parameter confidences==NULL" << endl;
		return -1;
	}

	if ( debug)
		std::cerr << "	classify 1 " << std::endl;

	//------------------------------------------------
	// classify
	//------------------------------------------------

	// create an example with the two lasers
	RangeExampleGeneral *re;
	re = new RangeExampleGeneral();
	re->initRange(frontlaser, rearlaser);

	if ( debug)
		std::cerr << "	classify 2 " << std::endl;


	// mark example as not classified
	re->classification = -1;
	re->pos_probability = new double[num_classes];

	double confidence;
	double confidence_sum;
	double pos_p, neg_p;
	float *pos_prob = new float[num_classes];
	float *neg_prob = new float[num_classes];


	for( int level=0; level < binary_classifiers; level++) {

		if ( debug)
			std::cerr << "	classify 3 " << std::endl;

		processExample(&(sc[level]), re);

		if ( debug)
			std::cerr << "	classify 4 " << std::endl;

		// if classification_type==0 then the weak hyapotheses return the value -1 or +1
		// if classification_type==1 then the weak hyapotheses return a value in the interval [-1, +1]
		int classification_type = 0;
		sc[level].classify(re, num_weak_hypotheses, &confidence, &confidence_sum, classification_type);

		if ( debug)
			std::cerr << "	classify 5 " << std::endl;


		//  see  "semantic labeling of places", stachniss, martinez-mozos, rottmann, burgard, 2005
		if ( confidence < 0.0 ) {
			// negative
			pos_p = 1.0 + confidence ;   //C_k
		}
		else {
			// positive
			pos_p = confidence ;
		}

		neg_p = 1.0 - pos_p;    // 1-C_k


		if ( level == 0) {
			// first binary classifier
			pos_prob[level] = pos_p;
			neg_prob[level] = neg_p;
		}
		else {
			pos_prob[level] = pos_p * neg_prob[level-1];   // z^k
			neg_prob[level] = neg_p * neg_prob[level-1];   // z^k_{-}
		}
	}


	// last class
	pos_prob[binary_classifiers] = 1.0 * neg_prob[binary_classifiers-1];


	double max_pos_prob = 0.0;
	int classification = -1;
	for( int level=0; level<num_classes; level++) {
		re->pos_probability[level] = pos_prob[level];
		if ( pos_prob[level] > max_pos_prob ) {
			max_pos_prob = pos_prob[level];
			classification = level;
		}
	}

	/*
	re->classification = classification;
	re->confidence = max_pos_prob;
	re->confidence_sum = 0.0;
	*/

	// returned values
	for( int level=0; level<num_classes; level++) {
		confidences[level] = re->pos_probability[level];
	}
	*class_confidence = max_pos_prob;

	delete re;
	delete [] pos_prob;
	delete [] neg_prob;

	// classification
	return classification;
}


//------------------------------------------------------
// classify the vector of features
//
// return:
//      >0 classification [0...num_classes-1]
//      -1 error
//------------------------------------------------------
/** Commented out as is not required and depends on old IDL file.
int SequentialAdaBoostClassifier::classifyFeatures(int num_weak_hypotheses,
						   PlaceData::LaserFeatures& featureVector,
						   double *class_confidence,
						   double *confidences) {

	if ( confidences == NULL ) {
		cerr << "ERROR: SequentialAdaBoostClassifier::classify: parameter confidences==NULL" << endl;
		return -1;
	}

	//------------------------------------------------
	// classify
	//------------------------------------------------


	// mark example as not classified
	double* pos_probability = new double[num_classes];
	double confidence;
	double confidence_sum;
	double pos_p, neg_p;
	float *pos_prob = new float[num_classes];
	float *neg_prob = new float[num_classes];


	for( int level=0; level < binary_classifiers; level++) {

	  // we don't need to do this anymore
	  //processExample(&(sc[level]), re);

	  // if classification_type==0 then the weak hyapotheses return the value -1 or +1
	  // if classification_type==1 then the weak hyapotheses return a value in the interval [-1, +1]
	  int classification_type = 0;
	  //sc[level].classify(re, num_weak_hypotheses, &confidence, &confidence_sum, classification_type)
	  sc[level].classifyFeatures(featureVector, num_weak_hypotheses, &confidence, &confidence_sum, classification_type);


	  //  see  "semantic labeling of places", stachniss, martinez-mozos, rottmann, burgard, 2005
	  if ( confidence < 0.0 ) {
	    // negative
	    pos_p = 1.0 + confidence ;   //C_k
	  }
	  else {
	    // positive
	    pos_p = confidence ;
	  }

	  neg_p = 1.0 - pos_p;    // 1-C_k


	  if ( level == 0) {
	    // first binary classifier
	    pos_prob[level] = pos_p;
	    neg_prob[level] = neg_p;
	  }
	  else {
	    pos_prob[level] = pos_p * neg_prob[level-1];   // z^k
	    neg_prob[level] = neg_p * neg_prob[level-1];   // z^k_{-}
	  }
	}


	// last class
	pos_prob[binary_classifiers] = 1.0 * neg_prob[binary_classifiers-1];


	double max_pos_prob = 0.0;
	int classification = -1;
	for( int level=0; level<num_classes; level++) {
	  pos_probability[level] = pos_prob[level];
	  if ( pos_prob[level] > max_pos_prob ) {
	    max_pos_prob = pos_prob[level];
	    classification = level;
	  }
	}


	//  re->classification = classification;
	//  re->confidence = max_pos_prob;
	//  re->confidence_sum = 0.0;


	// returned values
	for( int level=0; level<num_classes; level++) {
	  confidences[level] = pos_probability[level];
	}
	*class_confidence = max_pos_prob;

	delete [] pos_prob;
	delete [] neg_prob;
	delete [] pos_probability;

	// classification
	return classification;
}
*/




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
int SequentialAdaBoostClassifier::calcSelectedFeatures(RangeExampleGeneral *re,
						       dyntab_featuresInfo *featureList)
{
	//--------------------------------------------------
	// create list of features
	//--------------------------------------------------
	dyntab_features	*features = new dyntab_features(500);

	createSelectedFeatures(featureList, features);

	//-------------------------------------------------------
	// extract features from example
	//-------------------------------------------------------

	// feature 13
	// needed by 16, 17, 18, 19
	f13.setup(FC);
	f13.getFeature(re, -1);


	// featue 20
	// used by 21, 22, 23
	f20.getFeature(re, -1);

	if ( debug )
		cerr << "SequentialAdaBoostClassifier::calcSelectedFeatures(...): 1" << endl;

	Feature *f;
	FeatureValues* fv;

	re->setNumFeatures( features->num() );

	for (int i=0; i<features->num(); i++) {

	  f = features->getElement(i);

	  //printf("feature #%d %s\n", i, f->name);
	  //fflush(stdout);

	  f->getFeature(re, i);

	  if ( debug ) {
	    fv = re->getFeature(i);
	    cerr << "SequentialAdaBoostClassifier::calcSelectedFeatures(...): feature " << i << ": " << fv->value << " :" << f->name << endl;
	  }
	}

	//-------------------------------------------------------
	// prepare output
	//-------------------------------------------------------
	for (int i=0; i<features->num(); i++) {

		//copy the description
		FeatureInfo *fi;
		f = features->getElement(i);
		fi = featureList->getElement(i);
		strcpy(fi->description, f->name);

		// copy the value
		fv = re->getFeature(i);
		fi->result = fv->value;

		if ( debug )
			cerr << "SequentialAdaBoostClassifier::calcFeatures(...): fi->result: " << fi->result << endl;

	}

	if ( debug )
		cerr << "SequentialAdaBoostClassifier::calcFeatures(...): 2" << endl;

	features->setAutoDelete(true);
	delete features;

	return 1;
}
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
int SequentialAdaBoostClassifier::calcSelectedFeatures(carmen_robot_laser_message *frontLaser,
													carmen_robot_laser_message *rearLaser,
													dyntab_featuresInfo *featureList)
{

	// create an example with the two lasers
	RangeExampleGeneral *re;
	re = new RangeExampleGeneral();
	re->initRange(frontLaser, rearLaser);

	int ret;
	ret = calcSelectedFeatures(re, featureList);

	delete re;

	return ret;
}


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
int SequentialAdaBoostClassifier::calcSelectedFeatures(int numFrontReadings, float *frontLaser,
										int numRearReadings, float *rearLaser,
										dyntab_featuresInfo *featureList)
{
	carmen_robot_laser_message front, rear;


	front.num_readings = numFrontReadings;
	front.range = new float[front.num_readings];
	for(int i=0; i<front.num_readings; i++) {
		front.range[i] = (float)(frontLaser[i]);
	}


	rear.num_readings = numRearReadings;
	rear.range = new float[rear.num_readings];
	for(int i=0; i<rear.num_readings; i++) {
		rear.range[i] = (float)(rearLaser[i]);
	}


	int ret;
	ret = calcSelectedFeatures( &front, &rear, featureList );


	delete [] front.range;


	delete [] rear.range;


	return ret;
}


//---------------------------------------------------------------------------------
// Parameters:
//		INPUT:
//    		frontLaser, rearLaser: lasers in CARMEN format
//			list_features: list of features to calculate
// 		OUTPUT :
// 			features : list of calculated features.
//						     Memory is allocated inside the function. The user must free it.
//		RETURNS
//			<  0   if an error occurs
//			>= 0   if OK
// --------------------------------------------------------------------------------
int SequentialAdaBoostClassifier::createSelectedFeatures(dyntab_featuresInfo *featureList,
													dyntab_features *features)
{

	//--------------------------------------------------
	// rotation invariants
	//--------------------------------------------------

	for ( int i=0; i<featureList->num(); i++ ) {
		FeatureInfo *fi;

		fi = (FeatureInfo *)featureList->getElement(i);


		if ( strcmp(fi->name, "f2")==0 ) {
			// feature 2: average of the difference between consecutive beams
			Feature2 *f2;
			f2 = new Feature2;
			features->add(f2);
		}

		if ( strcmp(fi->name, "f2_sd")==0 ) {
			// feature 2_sd
			Feature2_sd *f2_sd;
			f2_sd = new Feature2_sd;
			features->add(f2_sd);
		}

		if ( strcmp(fi->name, "f3")==0 ) {
			int threshold = (int)fi->param;
			// feature 3:
			// different beam lengths
			Feature3 *f3;
			f3 = new Feature3;
			f3->setup(threshold);
			features->add(f3);
		}

		if ( strcmp(fi->name, "f5")==0 ) {
			// feature 5
			Feature5 *f5;
			f5 = new Feature5;
			features->add(f5);
		}

		if ( strcmp(fi->name, "f6")==0 ) {
			// feature 6
			Feature6 *f6;
			f6 = new Feature6;
			features->add(f6);
		}

		if ( strcmp(fi->name, "f7")==0 ) {
			// feature 7
			Feature7 *f7;
			f7 = new Feature7;
			features->add(f7);
		}

		if ( strcmp(fi->name, "f8")==0 ) {
			// feature 8
			Feature8 *f8;
			f8 = new Feature8;
			features->add(f8);
		}

		if ( strcmp(fi->name, "f8_sd")==0 ) {
			// feature 8_sd
			Feature8_sd *f8_sd;
			f8_sd = new Feature8_sd;
			features->add(f8_sd);
		}

		if ( strcmp(fi->name, "f9")==0 ) {
			// feature 9
			Feature9 *f9;
			f9 = new Feature9;
			features->add(f9);
		}


		if ( strcmp(fi->name, "f9_sd")==0 ) {
			// feature 9_sd
			Feature9_sd *f9_sd;
			f9_sd = new Feature9_sd;
			features->add(f9_sd);
		}

		if ( strcmp(fi->name, "f14")==0 ) {
			double threshold = fi->param;
			// feature 14
			// different thresholds
			Feature14 *f14;
			f14 = new Feature14;
			f14->setup(threshold);
			features->add(f14);
		}

		if ( strcmp(fi->name, "f16")==0 ) {
			int fd = (int)fi->param;
			// feature 16
			Feature16 *f16;
			f16 = new Feature16;
			f16->setup(fd, &f13);
			features->add(f16);
		}

		if ( strcmp(fi->name, "f17")==0 ) {
			// feature 17
			Feature17 *f17;
			f17 = new Feature17;
			f17->setup(&f13);
			features->add(f17);
		}

		if ( strcmp(fi->name, "f18")==0 ) {
			// feature 18
			Feature18 *f18;
			f18 = new Feature18;
			f18->setup(&f13);
			features->add(f18);
		}

		if ( strcmp(fi->name, "f19")==0 ) {
			// feature 19
			Feature19 *f19;
			f19 = new Feature19;
			f19->setup(&f13);
			features->add(f19);
		}

		if ( strcmp(fi->name, "f21")==0 ) {
			int moment = (int)fi->param;
			// feature 21
			Feature21 *f21;
			f21 = new Feature21;
			f21->setup(moment, &f20);
			features->add(f21);
		}

		if ( strcmp(fi->name, "f22")==0 ) {
			// feature 22
			Feature22 *f22;
			f22 = new Feature22;
			f22->setup(&f20);
			features->add(f22);
		}

		if ( strcmp(fi->name, "f23")==0 ) {
			// feature 23
			Feature23 *f23;
			f23 = new Feature23;
			f23->setup(&f20);
			features->add(f23);
		}

		if ( strcmp(fi->name, "f25")==0 ) {
			double deviation = fi->param;
			// feature 25
			Feature25 *f25;
			f25 = new Feature25;
			f25->setup(deviation);
			features->add(f25);
		}

		if ( strcmp(fi->name, "f26")==0 ) {
			// feature 26
			double deviation = fi->param;
			Feature26 *f26;
			f26 = new Feature26;
			f26->setup(deviation);
			features->add(f26);
		}

		// rotation-scale invariant

		if ( strcmp(fi->name, "f17_2")==0 ) {
			// feature 17_2
			Feature17_2 *f17_2;
			f17_2 = new Feature17_2;
			f17_2->setup(&f13);
			features->add(f17_2);
		}


		if ( strcmp(fi->name, "f18_2")==0 ) {
			// feature 18_2
			Feature18_2 *f18_2;
			f18_2 = new Feature18_2;
			f18_2->setup(&f13);
			features->add(f18_2);
		}


		if ( strcmp(fi->name, "f19_2")==0 ) {
			// feature 19_2
			Feature19_2 *f19_2;
			f19_2 = new Feature19_2;
			f19_2->setup(&f13);
			features->add(f19_2);
		}

		if ( strcmp(fi->name, "f27")==0 ) {
			// feature 27
			Feature27 *f27;
			f27 = new Feature27;
			features->add(f27);
		}


		if ( strcmp(fi->name, "f27_sd")==0 ) {
			// feature 27_sd
			Feature27_sd *f27_sd;
			f27_sd = new Feature27_sd;
			features->add(f27_sd);
		}


		if ( strcmp(fi->name, "f28")==0 ) {
			// feature 28
			Feature28 *f28;
			f28 = new Feature28;
			features->add(f28);
		}

		if ( strcmp(fi->name, "f28_sd")==0 ) {
			// feature 28_sd
			Feature28_sd *f28_sd;
			f28_sd = new Feature28_sd;
			features->add(f28_sd);
		}

		if ( strcmp(fi->name, "f29")==0 ) {
			// feature 29
			Feature29 *f29;
			f29 = new Feature29;
			features->add(f29);
		}


		if ( strcmp(fi->name, "f29_sd")==0 ) {
			// feature 29_sd
			Feature29_sd *f29_sd;
			f29_sd = new Feature29_sd;
			features->add(f29_sd);
		}


		if ( strcmp(fi->name, "f30")==0 ) {
			double threshold = fi->param;
			// feature 30
			Feature30 *f30;
			f30 = new Feature30;
			f30->setup(threshold);
			features->add(f30);
		}

		if ( strcmp(fi->name, "f31")==0 ) {
			// feature 31
			Feature31 *f31;
			f31 = new Feature31;
			features->add(f31);
		}

		if ( strcmp(fi->name, "f32")==0 ) {
			// feature 32
			Feature32 *f32;
			f32 = new Feature32;
			features->add(f32);
		}

		if ( strcmp(fi->name, "f33")==0 ) {
			double threshold = fi->param;
			// feature 33
			// different thresholds
			Feature33 *f33;
			f33 = new Feature33;
			f33->setup(threshold);
			features->add(f33);
		}


		if ( strcmp(fi->name, "f34")==0 ) {
			double threshold = fi->param;
			// feature 34
			// different thresholds
			Feature34 *f34;
			f34 = new Feature34;
			f34->setup(threshold);
			features->add(f34);
		}
	}//end 	for ( int i=0; i<numSelectedFeatures; i++ ) {



	return 1;
}



//--------------------------------------------
//
//--------------------------------------------
void SequentialAdaBoostClassifier::printParameters(oscar::Feature *f, ostream &os) {

	if ( strcmp( f->type, "f3" ) == 0 ){
		oscar::Feature3 *f3;
		f3 = (oscar::Feature3 *)f;
		os << f3->beam_length << endl;
	}

	if ( strcmp( f->type, "f14" ) == 0 ){
		oscar::Feature14 *f14;
		f14 = (oscar::Feature14 *)f;
		os << f14->threshold << endl;
	}

	if ( strcmp( f->type, "f16" ) == 0 ){
		oscar::Feature16 *f16;
		f16 = (oscar::Feature16 *)f;
		os << f16->index << endl;
	}

	if ( strcmp( f->type, "f21" ) == 0 ){
		oscar::Feature21 *f21;
		f21 = (oscar::Feature21 *)f;
		os << f21->invariant << endl;
	}

	if ( strcmp( f->type, "f25" ) == 0 ){
		oscar::Feature25 *f25;
		f25 = (oscar::Feature25 *)f;
		os << f25->deviation << endl;
	}

	if ( strcmp( f->type, "f26" ) == 0 ){
		oscar::Feature26 *f26;
		f26 = (oscar::Feature26 *)f;
		os << f26->deviation << endl;
	}

	if ( strcmp( f->type, "f30" ) == 0 ){
		oscar::Feature30 *f30;
		f30 = (oscar::Feature30 *)f;
		os << f30->threshold << endl;
	}

	if ( strcmp( f->type, "f33" ) == 0 ){
		oscar::Feature33 *f33;
		f33 = (oscar::Feature33 *)f;
		os << f33->threshold << endl;
	}

	if ( strcmp( f->type, "f34" ) == 0 ){
		oscar::Feature34 *f34;
		f34 = (oscar::Feature34 *)f;
		os << f34->threshold << endl;
	}
}


//--------------------------------------------------
// calculate features for one example
//--------------------------------------------------
int SequentialAdaBoostClassifier::extractFeatures(dyntab_features *features, RangeExample *e) {

	Feature *f;

	// feature 13
	// needed by 16, 17, 18, 19
	f13.setup(FC);
	f13.getFeature(e, -1);

	// featue 20
	// used by 21, 22, 23
	f20.getFeature(e, -1);


	for (int i=0; i<features->num(); i++) {
		f = features->getElement(i);
		f->getFeature(e, i);
	}

	return 1; // ok
}







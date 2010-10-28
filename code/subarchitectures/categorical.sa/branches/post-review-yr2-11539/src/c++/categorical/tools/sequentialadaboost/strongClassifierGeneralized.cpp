//-----------------------------------------
// strong classifier
//
// strong classifier obtained by adaboost
//
// "The boosting Approach to machine learning: and overview" robert e. schapire. 2002
//
// oscar martinez
//-----------------------------------------

#include <math.h>
#include <string.h>

#include "strongClassifierGeneralized.h"
#include "example.h"
#include "features/all_features.h"

using namespace oscar;



//-------------------------------------------
// constructor
//-------------------------------------------
StrongClassifierGeneralized::StrongClassifierGeneralized() {
	hypotheses = NULL;
	alpha = NULL;
	f13.setup(100);
}

//-------------------------------------------
// destructor
//-------------------------------------------
StrongClassifierGeneralized::~StrongClassifierGeneralized() {

	if (hypotheses != NULL) {
		delete hypotheses;
		hypotheses = NULL;
	}

	if ( alpha != NULL ) {
		delete [] alpha;
		alpha = NULL;
	}
}



//-------------------------------------------
//
//-------------------------------------------
void StrongClassifierGeneralized::readHypotheses(ifstream &ifs) {
	char line[500];
	char c;
	double r;
	int has_param;

	hypotheses = new dyntab_hypotheses(2);

	int num;
	ThresholdHypothesisGeneral *h;

	ifs >> num;
	ifs.getline(&c, 1);  // read "\n"

	for ( int i=0; i<num; i++ ) {
		h = new ThresholdHypothesisGeneral;
		ifs.getline(line, 500);  // name

		ifs.getline(line, 500);  // type

		ifs >> has_param;

		if ( strcmp(line, "f2")== 0 ) {
			oscar::Feature2 *f2 = new oscar::Feature2;
			h->feature = f2;
		}
		else
		if ( strcmp(line, "f2_sd")== 0 ) {
			oscar::Feature2_sd *f2_sd = new oscar::Feature2_sd;
			h->feature = f2_sd;
		}
		else
		if ( strcmp(line, "f3")== 0 ) {
			oscar::Feature3 *f3 = new oscar::Feature3;
			double d;
			ifs >> d;
			f3->setup(d);
			h->feature = f3;
		}
		else
		if ( strcmp(line, "f5")== 0 ) {
			oscar::Feature5 *f5 = new oscar::Feature5;
			h->feature = f5;
		}
		else
		if ( strcmp(line, "f6")== 0 ) {
			oscar::Feature6 *f6 = new oscar::Feature6;
			h->feature = f6;
		}
		else
		if ( strcmp(line, "f7")== 0 ) {
			oscar::Feature7 *f7 = new oscar::Feature7;
			h->feature = f7;
		}
		else
		if ( strcmp(line, "f8")== 0 ) {
			oscar::Feature8 *f8 = new oscar::Feature8;
			h->feature = f8;
		}
		else
		if ( strcmp(line, "f8_sd")== 0 ) {
			oscar::Feature8_sd *f8_sd = new oscar::Feature8_sd;
			h->feature = f8_sd;
		}
		else
		if ( strcmp(line, "f9")== 0 ) {
			oscar::Feature9 *f9 = new oscar::Feature9;
			h->feature = f9;
		}
		else
		if ( strcmp(line, "f9_sd")== 0 ) {
			oscar::Feature9_sd *f9_sd = new oscar::Feature9_sd;
			h->feature = f9_sd;
		}
		else
		if ( strcmp(line, "f14")== 0 ) {
			oscar::Feature14 *f14 = new oscar::Feature14;
			h->feature = f14;
			double d;
			ifs >> d;
			f14->setup(d);
			h->feature = f14;
		}
		else
		if ( strcmp(line, "f16")== 0 ) {
			oscar::Feature16 *f16 = new oscar::Feature16;
			int d;
			ifs >> d;
			f16->setup(d, &f13);
			h->feature = f16;
		}
		else
		if ( strcmp(line, "f17")== 0 ) {
			oscar::Feature17 *f17 = new oscar::Feature17;
			f17->setup(&f13);
			h->feature = f17;
		}
		else
		if ( strcmp(line, "f18")== 0 ) {
			oscar::Feature18 *f18 = new oscar::Feature18;
			f18->setup(&f13);
			h->feature = f18;
		}
		else
		if ( strcmp(line, "f19")== 0 ) {
			oscar::Feature19 *f19 = new oscar::Feature19;
			f19->setup(&f13);
			h->feature = f19;
		}
		if ( strcmp(line, "f17_2")== 0 ) {
			oscar::Feature17_2 *f17_2 = new oscar::Feature17_2;
			f17_2->setup(&f13);
			h->feature = f17_2;
		}
		else
		if ( strcmp(line, "f18_2")== 0 ) {
			oscar::Feature18_2 *f18_2 = new oscar::Feature18_2;
			f18_2->setup(&f13);
			h->feature = f18_2;
		}
		else
		if ( strcmp(line, "f19_2")== 0 ) {
			oscar::Feature19_2 *f19_2 = new oscar::Feature19_2;
			f19_2->setup(&f13);
			h->feature = f19_2;
		}
		else
		if ( strcmp(line, "f21")== 0 ) {
			oscar::Feature21 *f21 = new oscar::Feature21;
			int d;
			ifs >> d;
			f21->setup(d, &f20);
			h->feature = f21;
		}
		else
		if ( strcmp(line, "f22")== 0 ) {
			oscar::Feature22 *f22 = new oscar::Feature22;
			f22->setup(&f20);
			h->feature = f22;
		}
		else
		if ( strcmp(line, "f23")== 0 ) {
			oscar::Feature23 *f23 = new oscar::Feature23;
			f23->setup(&f20);
			h->feature = f23;
		}
		else
		if ( strcmp(line, "f25")== 0 ) {
			oscar::Feature25 *f25 = new oscar::Feature25;
			double d;
			ifs >> d;
			f25->setup(d);
			h->feature = f25;
		}
		else
		if ( strcmp(line, "f26")== 0 ) {
			oscar::Feature26 *f26 = new oscar::Feature26;
			double d;
			ifs >> d;
			f26->setup(d);
			h->feature = f26;
		}
		else
		if ( strcmp(line, "f27")== 0 ) {
			oscar::Feature27 *f27 = new oscar::Feature27;
			h->feature = f27;
		}
		else
		if ( strcmp(line, "f27_sd")== 0 ) {
			oscar::Feature27_sd *f27_sd = new oscar::Feature27_sd;
			h->feature = f27_sd;
		}
		else
		if ( strcmp(line, "f28")== 0 ) {
			oscar::Feature28 *f28 = new oscar::Feature28;
			h->feature = f28;
		}
		else
		if ( strcmp(line, "f28_sd")== 0 ) {
			oscar::Feature28_sd *f28_sd = new oscar::Feature28_sd;
			h->feature = f28_sd;
		}
		else
		if ( strcmp(line, "f29")== 0 ) {
			oscar::Feature29 *f29 = new oscar::Feature29;
			h->feature = f29;
		}
		else
		if ( strcmp(line, "f29_sd")== 0 ) {
			oscar::Feature29_sd *f29_sd = new oscar::Feature29_sd;
			h->feature = f29_sd;
		}
		else
		if ( strcmp(line, "f30")== 0 ) {
			oscar::Feature30 *f30 = new oscar::Feature30;
			double d;
			ifs >> d;
			f30->setup(d);
			h->feature = f30;
		}
		else
		if ( strcmp(line, "f31")== 0 ) {
			oscar::Feature31 *f31 = new oscar::Feature31;
			h->feature = f31;
		}
		else
		if ( strcmp(line, "f32")== 0 ) {
			oscar::Feature32 *f32 = new oscar::Feature32;
			h->feature = f32;
		}
		else
		if ( strcmp(line, "f33")== 0 ) {
			oscar::Feature33 *f33 = new oscar::Feature33;
			double d;
			ifs >> d;
			f33->setup(d);
			h->feature = f33;
		}
		else
		if ( strcmp(line, "f34")== 0 ) {
			oscar::Feature34 *f34 = new oscar::Feature34;
			double d;
			ifs >> d;
			f34->setup(d);
			h->feature = f34;
		}


		ifs >> h->threshold;
		ifs >> h->direction;
		ifs >> r;
		ifs >> h->misclassified;
		ifs >> h->sum_weights;
		ifs >> h->index;

		h->alpha = 0.5 * log ( (1.0+r) / (1.0-r) );
		ifs.getline(&c, 1);  // read "\n"

		hypotheses->add(h);

		if ( debug ) {
			cerr << "Hypothesis---------------------" << endl;
			cerr << line << endl;
			cerr << h->threshold << endl;
			cerr << h->direction << endl;
			cerr << r << endl;
			cerr << h->misclassified << endl;
			cerr << h->sum_weights << endl;
			cerr << h->index << endl;
			cerr << h->alpha << endl;
		}
	}
}


//-------------------------------------------
//
//-------------------------------------------
void StrongClassifierGeneralized::classify(RangeExample *re, int num_hypotheses, double *confidence, double *confidence_sum, int classification_type) {

	ThresholdHypothesisGeneral *h;
	double sum, conf;
	double c;
	int num;

	if ( num_hypotheses <= 0 ) {
		num = hypotheses->num();
	}
	else {
		num = num_hypotheses;
	}

	sum = 0.0;
	for ( int i=0; i<num; i++) {
		h = hypotheses->getElement(i);
		h->classify(re, h->index, &conf);

		if ( classification_type ==0 ) {
			// {-1, +1}
			if ( conf < 0.0 ) {
				c = -1.0;
			}
			else {
				c = 1.0;
			}
		}
		else {
			// [+1,-1]
			c = conf;
		}
		sum += c * h->alpha;
	}

	*confidence_sum = sum;

	// confidence value
	if (  *confidence_sum >= 0.0 ) {
		// positive
		*confidence = exp( sum ) / ( exp(-sum) + exp (sum) );
	}
	else {
		// negative
		*confidence = exp( -sum ) / ( exp(-sum) + exp (sum) );
		*confidence = -(*confidence);
	}
}


/* Commented out as is not required and depends on old IDL file.
void StrongClassifierGeneralized::classifyFeatures(PlaceData::LaserFeatures& featureVector,
						   int num_hypotheses,
						   double *confidence,
						   double *confidence_sum,
						   int classification_type) {

	ThresholdHypothesisGeneral *h;
	double sum, conf;
	double c;
	int num;

	if ( num_hypotheses <= 0 ) {
		num = hypotheses->num();
	}
	else {
		num = num_hypotheses;
	}

	sum = 0.0;
	for ( int i=0; i<num; i++) {
		h = hypotheses->getElement(i);
		double featureValue = featureVector.features[i].value;
		h->classifyFeature(featureValue, &conf);

		if ( classification_type ==0 ) {
			// {-1, +1}
			if ( conf < 0.0 ) {
				c = -1.0;
			}
			else {
				c = 1.0;
			}
		}
		else {
			// [+1,-1]
			c = conf;
		}
		sum += c * h->alpha;
	}

	*confidence_sum = sum;

	// confidence value
	if (  *confidence_sum >= 0.0 ) {
		// positive
		*confidence = exp( sum ) / ( exp(-sum) + exp (sum) );
	}
	else {
		// negative
		*confidence = exp( -sum ) / ( exp(-sum) + exp (sum) );
		*confidence = -(*confidence);
	}
}
*/



















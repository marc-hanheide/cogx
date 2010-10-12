//-----------------------------------------------------------
// feature 28_sd:
//		standard deviation  of the beams length / maximum length
//
// type 28_sd
//
// oscar martinez
//-----------------------------------------------------------

#include "feature28_sd.h"
#include "feature28.h"

#include <math.h>

#include <iostream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature28_sd::Feature28_sd() {
	setName("f28_sd:std deviation of beams lentgth / maximum length");
	setType("f28_sd");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature28_sd::~Feature28_sd() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature28_sd::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature28 f28;
	f28.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double mean = fv->value;


	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	double sum;
	double v, val;
	double std;
	double max =0.0;

	// look for maximum	
	for (int i=0; i<n; i++) {
		val = (double)re->totalRange[i] + 0.0001;		
		if ( val > max ) {
			max = val;
		}
	}
		
	sum = 0.0;
	for (int i=0; i<n; i++) {
		v = ((double)re->totalRange[i] + 0.0001) / max;
		v = v - mean;
		v = v*v;
		sum += v;
	}

	std = sum / ( (double)n );
	std = sqrt(std);

	fv->value = std;

	return 1;
}













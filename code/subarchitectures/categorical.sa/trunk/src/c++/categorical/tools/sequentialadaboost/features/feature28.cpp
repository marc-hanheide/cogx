//-----------------------------------------------------------
// feature 28:
//		average of length of beams / maximum length
//
// type 28
//
// oscar martinez
//-----------------------------------------------------------

#include "feature28.h"

#include <iostream>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature28::Feature28() {
	setName("f28: average of beams length /maximum length");
	setType("f28");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature28::~Feature28() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature28::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	double sum, max = 0.0;
	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	double val;
	
	sum = 0.0;	
	for (int i=0; i<n; i++) {
		val = (double)re->totalRange[i] + 0.0001;
		sum += val;
		if ( val > max ) {
			max = val;
		}
	}
	double mean = sum / (double)n;
	
	// to avoid divide by cero problems 
	mean = mean / max;

	fv = &(re->list_features[index_feature]);
	fv->value = mean;

	return 1;
}













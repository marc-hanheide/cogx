//-----------------------------------------------------------
// feature 8:
//		average of length of beams
//
// type 8
//
// oscar martinez
//-----------------------------------------------------------

#include "feature8.h"

#include <iostream>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature8::Feature8() {
	setName("f8: average of beams length");
	setType("f8");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature8::~Feature8() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature8::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	double sum;
	int n = (re->flaser).num_readings + (re->rlaser).num_readings;

	sum = 0.0;
	for (int i=0; i<n; i++) {
		sum += re->totalRange[i];
	}
	double mean = sum / (double)n;

	fv = &(re->list_features[index_feature]);
	fv->value = mean;

	return 1;
}













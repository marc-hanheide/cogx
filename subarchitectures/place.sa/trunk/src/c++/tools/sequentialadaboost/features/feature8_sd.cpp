//-----------------------------------------------------------
// feature 8_sd:
//		standard deviation  of the beams length
//
// type 8_sd
//
// oscar martinez
//-----------------------------------------------------------

#include "feature8_sd.h"
#include "feature8.h"

#include <iostream>
#include <math.h>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature8_sd::Feature8_sd() {
	setName("f8_sd: std deviation of beams lentgth");
	setType("f8_sd");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature8_sd::~Feature8_sd() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature8_sd::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature8 f8;
	f8.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double mean = fv->value;


	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	double sum;
	double v;
	double std;
	sum = 0.0;
	for (int i=0; i<n; i++) {
		v = re->totalRange[i] - mean;
		v = v*v;
		sum += v;
	}

	std = sum / ( (double)n );
	std = sqrt(std);

	fv->value = std;

	return 1;
}













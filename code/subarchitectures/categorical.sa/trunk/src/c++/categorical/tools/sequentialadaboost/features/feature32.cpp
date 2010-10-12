//-----------------------------------------
// feature 32:  kurtosis
//
// type = 32
//
// oscar martinez
//-----------------------------------------


#include "feature32.h"
#include "feature8.h"
#include "feature8_sd.h"


#include <math.h>
#include <iostream>
#include <fstream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature32::Feature32() {
	setName("f32: kurtosis");
	setType("f32");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature32::~Feature32() {
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature32::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature32::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature8 f8;
	f8.getFeature(re, index_feature);
	FeatureValues *f;
	f = re->getFeature(index_feature);
	double mean = f->value;
		
	// std
	Feature8_sd f8_sd;
	f8_sd.getFeature(re, index_feature);
	f = re->getFeature(index_feature);
	double std = f->value;

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	
	double sum = 0.0;
	for (int i=0; i<n; i++) {
		double a = (double)re->totalRange[i] - mean; 
		sum += a*a*a*a;
	}
	sum = sum / ( (double)n * std*std*std*std );
	
	// write feature
	fv = &(re->list_features[index_feature]);
	fv->value = sum;

	return 1;
}




































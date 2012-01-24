//-----------------------------------------------------------
// feature 33:
//		  gaps / (number_of_beams - 1)
// type 33
//
// oscar martinez
//-----------------------------------------------------------

#include "feature14.h"
#include "feature33.h"

#include <math.h>

#include <iostream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature33::Feature33() {
	setName("f33: gaps / (number_of_beams - 1)");
	setType("f33");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature33::~Feature33() {
}

//-----------------------------------------
//
//-----------------------------------------
void Feature33::setup(double t) {
	threshold = t;  // 0.5 meters
	char name[500];
	sprintf(name, "feature33:number of gaps. Threshold %f / (number_of_beams - 1)", threshold);
	setName(name);
}

//-----------------------------------------
// get feature
//-----------------------------------------
int Feature33::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature14 f14;
	f14.setup(threshold);
	f14.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double gaps = fv->value;

	fv->value = gaps / (re->num_beams -1);

	return 1;
}













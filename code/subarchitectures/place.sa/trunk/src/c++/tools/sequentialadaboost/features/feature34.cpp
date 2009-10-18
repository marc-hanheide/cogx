//-----------------------------------------------------------
// feature 34:
//		  gaps (f_30)/ (number_of_beams - 1)
// type 34
//
// oscar martinez
//-----------------------------------------------------------

#include "feature30.h"
#include "feature34.h"

#include <math.h>

#include <iostream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature34::Feature34() {
	setName("f34: gaps (f_30) / (number_of_beams - 1)");
	setType("f34");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature34::~Feature34() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature34::setup(double t) {
	threshold = t;  // 0.5 meters
	char name[500];
	sprintf(name, "feature34:number of rel. gaps. Threshold %f / (number_of_beams - 1)", threshold);
	setName(name);
}

//-----------------------------------------
// get feature
//-----------------------------------------
int Feature34::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature30 f30;
	f30.setup(threshold);
	f30.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double gaps = fv->value;

	fv->value = gaps / (re->num_beams -1);

	return 1;
}













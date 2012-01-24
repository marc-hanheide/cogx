//-----------------------------------------------------------
// feature 10:
// number N of sides to a regular digital polygon (Computer and robot vision VI, p61)
//
// type 10
//
// oscar martinez
//-----------------------------------------------------------

#include "feature9_m_sd.h"
#include "feature10.h"

#include <math.h>
#include <iostream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature10::Feature10() {
	setName("f10: # sides to a regular digital polygon");
	setType("f10");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature10::~Feature10() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature10::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature10::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean/std
	Feature9_m_sd f9_m_sd;
	f9_m_sd.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double ratio = fv->value;

	double n = 1.4111 * pow(ratio, 0.4724);

	// write feature
	fv->value = n;

	return 1;
}













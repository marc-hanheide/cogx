//-----------------------------------------------------------
// feature 9_m_sd:
//		mean of of the distances from end of beams to centroid divided by
//		standard deviation
// type 9_m_sd
//
// oscar martinez
//-----------------------------------------------------------

#include "feature9_m_sd.h"
#include "feature9_sd.h"
#include "feature9.h"
#include "feature5.h"
#include "rangeExample.h"

#include <iostream>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature9_m_sd::Feature9_m_sd() {
	setName("f9_m_sd: mean / std deviation of distance to centroid");
	setType("f9_m_sd");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature9_m_sd::~Feature9_m_sd() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature9_m_sd::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature9 f9;
	f9.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double mean = fv->value;

	// standard deviation
	Feature9_sd f9_sd;
	f9_sd.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double std = fv->value;



	// write feature
	fv->value = mean / std;

	return 1;
}













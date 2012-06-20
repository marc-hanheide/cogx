//-----------------------------------------
// feature 19:
//
// major_axis_len / minor_axis_len  of the ellipse using 1 and -1 fourier descriptors
//
// type = 19
//
// oscar martinez
//-----------------------------------------



#include "feature19.h"
#include "feature13.h"

#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature19::Feature19() {
	setName("f19: fourier: minor axis / major axis");
	setType("f19");
	f13 = NULL;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature19::~Feature19() {
}

//-----------------------------------------
//
//-----------------------------------------
void Feature19::setup(oscar::Feature13 *f)  {
	f13 = f;
}

//-----------------------------------------
// get feature
//-----------------------------------------
int Feature19::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	if ( f13 == NULL ) {
		cerr << "ERROR: Feature17::getFeature: f13==NULL" << endl;
		return -1;
	}

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}
	fv = re->getFeature(index_feature);

	//fv->value = f13->major_axis_len / f13->minor_axis_len;

	fv->value = f13->minor_axis_len / f13->major_axis_len;
	
	return 1;
}





































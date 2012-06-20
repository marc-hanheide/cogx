//-----------------------------------------
// feature 26:
//
// Look for the two local minima in beams legth with minimum value.
// Beam distance between these two minima.
//
// type = 26
//
// oscar martinez
//-----------------------------------------

#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature26.h"


using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature26::Feature26() {
	setName("f26: angle between 2 minimum");
	setType("f26");
	deviation = -1.0;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature26::~Feature26() {
}


//-----------------------------------------
// setup
//-----------------------------------------
void Feature26::setup(double d) {	
	deviation = d;
	f25.setup(deviation);
	char name[500];
	sprintf(name, "f26: angle between 2 minimum deviation %le", deviation);
	setName(name);	
}


//-----------------------------------------
//
//-----------------------------------------
void Feature26::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature26::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature26::getFeature: list_features==NULL" << endl;
		return -1;
	}

	if ( deviation < 0.0 ) {
		cerr << "ERROR: Feature26::getFeature(...): deviation NOT set" << endl;
		return -1;
	}

	fv = re->getFeature(index_feature);

	f25.getFeature(e, index_feature);

	fv->value = f25.beam_distance;

	return 1;
}





































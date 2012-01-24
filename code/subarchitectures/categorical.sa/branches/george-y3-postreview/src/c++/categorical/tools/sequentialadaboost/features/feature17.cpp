//-----------------------------------------
// feature 17:
//
// major_axis length of the ellipse using 1 and -1 fourier descriptors
//
// type = 17
//
// oscar martinez
//-----------------------------------------


#include "feature17.h"
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
Feature17::Feature17() {
	setName("f17: fourier: major axis length using");
	setType("f17");
	f13 = NULL;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature17::~Feature17() {
}

//-----------------------------------------
//
//-----------------------------------------
void Feature17::setup(oscar::Feature13 *f)  {
	f13 = f;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature17::getFeature(oscar::Example *e, int index_feature) {

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


	fv->value = f13->major_axis_len;

	return 1;
}





































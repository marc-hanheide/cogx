//-----------------------------------------
// feature 17_2:
//
// major_axis length of the ellipse using 1 and -1 fourier descriptors
//
// type = 17_2
//
// oscar martinez
//-----------------------------------------


#include "feature17_2.h"
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
Feature17_2::Feature17_2() {
	setName("f17_2:fourier: major axis length using descp.");
	setType("f17_2");
	f13 = NULL;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature17_2::~Feature17_2() {
}

//-----------------------------------------
//
//-----------------------------------------
void Feature17_2::setup(oscar::Feature13 *f)  {
	f13 = f;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature17_2::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	if ( f13 == NULL ) {
		cerr << "ERROR: Feature17_2::getFeature: f13==NULL" << endl;
		return -1;
	}

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}
	fv = re->getFeature(index_feature);


	fv->value = f13->major_axis_len_d;

	return 1;
}





































//-----------------------------------------
// feature 16:
//
// one fourier descriptor
//
// type = 16
//
// oscar martinez
//-----------------------------------------


#include "feature16.h"
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
Feature16::Feature16() {
	index = 1;
	char name[500];
	sprintf(name, "f16: fourier descriptor %d", index);
	setName(name);
	setType("f16");
	f13 = NULL;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature16::~Feature16() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature16::setup(int i, oscar::Feature13 *f)  {
	index = i;
	f13 = f;
	char name[500];
	sprintf(name, "fourier descriptor %d", index);
	setName(name);
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature16::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	if ( f13 == NULL ) {
		cerr << "ERROR: Feature16::getFeature: f13==NULL" << endl;
		return -1;
	}

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature16::getFeature: list_features==NULL" << endl;
		return -1;
	}


	fv = re->getFeature(index_feature);

	int n = f13->n_coefficients;
	fv->value = gsl_complex_abs(f13->descriptors[index + n]);

	return 1;
}





































//-----------------------------------------
// feature 21:
//
// invariants. Digital Image processing, gonzalez, p516
//
// type = 21
//
// oscar martinez
//-----------------------------------------

#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature21.h"

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature21::Feature21() {
	invariant = 0;
	char name[500];
	sprintf(name, "f21:moment invariant %d", invariant);
	setName(name);
	setType("f21");
	f20 = NULL;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature21::~Feature21() {
}

//-----------------------------------------
// setup
//-----------------------------------------
void Feature21::setup(int i, oscar::Feature20 *f) {

	if (i > 6) {
		invariant = 0;
	}
	else {
		invariant = i;
	}
	char name[500];
	sprintf(name, "moment invariant %d", invariant);
	setName(name);
	f20 = f;
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature21::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	if ( f20 == NULL ) {
		cerr << "Feature21::getFeature f20==NULL" << endl;
		return -1;
	}

	oscar::FeatureValues *fv;
	fv = re->getFeature(index_feature);
	fv->value = f20->inv[invariant];

	return 1;
}



































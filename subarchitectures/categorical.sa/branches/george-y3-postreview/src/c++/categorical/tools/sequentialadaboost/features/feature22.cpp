//-----------------------------------------
// feature 22:
//
// M_cmp (slides from Dmitrij)
//
// type = 22
//
// oscar martinez
//-----------------------------------------

#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature22.h"

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature22::Feature22() {
	setName("f22:M_cmp");
	setType("f22");
	f20 = NULL;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature22::~Feature22() {
}

//-----------------------------------------
// setup
//-----------------------------------------
void Feature22::setup(oscar::Feature20 *f) {
	f20 = f;
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature22::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	if ( f20 == NULL ) {
		cerr << "Feature22::getFeature f20==NULL" << endl;
		return -1;
	}

	oscar::FeatureValues *fv;
	fv = re->getFeature(index_feature);
	fv->value = f20->u00 / (f20->u20 + f20->u02);

	return 1;
}



































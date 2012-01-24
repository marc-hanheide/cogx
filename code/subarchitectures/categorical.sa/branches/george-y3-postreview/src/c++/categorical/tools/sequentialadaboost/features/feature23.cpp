//-----------------------------------------
// feature 23:
//
// M_ect (slides from Dmitrij)
//
// type = 23
//
// oscar martinez
//-----------------------------------------

#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature23.h"

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature23::Feature23() {
	setName("f23:M_ect");
	setType("f23");
	f20 = NULL;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature23::~Feature23() {
}

//-----------------------------------------
// setup
//-----------------------------------------
void Feature23::setup(oscar::Feature20 *f) {
	f20 = f;
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature23::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	if ( f20 == NULL ) {
		cerr << "Feature23::getFeature f20==NULL" << endl;
		return -1;
	}

	oscar::FeatureValues *fv;
	fv = re->getFeature(index_feature);
	fv->value = sqrt(pow(f20->u20 - f20->u02, 2) + 4.0*f20->u11*f20->u11) / (f20->u20+ f20->u02);

	return 1;
}



































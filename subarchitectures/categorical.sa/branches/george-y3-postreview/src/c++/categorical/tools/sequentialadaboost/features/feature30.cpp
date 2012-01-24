//-----------------------------------------
// feature 30:
//
// number of relative gaps.
// One gap:
//		b_i / b_{i+1} > threshold
//
// type = 30
//
// oscar martinez
//-----------------------------------------

#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature30.h"


using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature30::Feature30() {
	threshold = 0.5;  // 0.5 meters
	char name[500];
	sprintf(name, "f30:number of rel. gaps. Threshold %f", threshold);
	setName(name);
	setType("f30");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature30::~Feature30() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature30::setup(double t) {
	threshold = t;  // 0.5 meters
	char name[500];
	sprintf(name, "f30:number of rel. gaps. Threshold %f", threshold);
	setName(name);
}


//-----------------------------------------
//
//-----------------------------------------
void Feature30::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature30::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}
	fv = re->getFeature(index_feature);

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	int next;
	double rel;
	int gaps = 0;
	double min = 0.0001;
	double a, b;
	
	for(int i=0; i<n; i++) {
		next = (i+1)%n;
		a = (double)re->totalRange[i] + min;
		b = (double)re->totalRange[next] + min;
		rel = a / b;
		if ( rel > 1.0 ) {
			rel = 1.0 / rel;
		}	
		if (rel < threshold) {
			gaps++;
		}
	}

	fv->value = (double)gaps;

	return 1;
}





































//-----------------------------------------
// feature 14:
//
// number of gaps
//
// type = 14
//
// oscar martinez
//-----------------------------------------


#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature14.h"


using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature14::Feature14() {
	threshold = 0.5;  // 0.5 meters
	char name[500];
	sprintf(name, "f14: number of gaps. Threshold >%f", threshold);
	setName(name);
	setType("f14");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature14::~Feature14() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature14::setup(double t) {
	threshold = t;  // 0.5 meters
	char name[500];
	sprintf(name, "f14: number of gaps. Threshold >%f", threshold);
	setName(name);
}


//-----------------------------------------
//
//-----------------------------------------
void Feature14::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature14::getFeature(oscar::Example *e, int index_feature) {

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
	double diff;
	int gaps = 0;
	for(int i=0; i<n; i++) {
		next = (i+1)%n;
		diff = fabs( re->totalRange[i] - re->totalRange[next]);
		if (diff > threshold) {
			gaps++;
		}
	}

	fv->value = (double)gaps;

	return 1;
}





































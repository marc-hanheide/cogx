//-----------------------------------------
// feature3
//
// oscar martinez
//-----------------------------------------


#include "feature3.h"

#include <iostream>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructors
//-----------------------------------------
Feature3::Feature3() {
	beam_length = 10.0;
	setType("f3");
}

Feature3::Feature3(double len) {
	setup(len);
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature3::~Feature3() {
}

//-----------------------------------------
// get feature
//-----------------------------------------
void Feature3::setup(double len) {
	beam_length = len;
	char n[100];
	sprintf(n, "f3: average of diff. between CUT(%f) consecutive beams",
			beam_length);

	setName(n);
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature3::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	float newRange1, newRange2;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature3::getFeature: list_features==NULL" << endl;
		return -1;
	}

	if ( beam_length < 0.0 ) {
		cerr << "ERROR: Feature3::getFeature: beam_length < 0" << endl;
		return -1;
	}

	double sum;
	int i1, i2;

	sum = 0.0;
	for (int i=0; i<re->num_beams; i++) {
		i1 = i;
		i2 = (i+1) % re->num_beams;
		// cut beams
		newRange1 = re->totalRange[i1];
		newRange2 = re->totalRange[i2];
		if ( newRange1 > beam_length ) {
			newRange1 = beam_length;
		}
		if ( newRange2 > beam_length ) {
			newRange2 = beam_length;
		}
		sum += (double) fabs(newRange1 - newRange2);
	}

	fv = &(re->list_features[index_feature]);
	fv->value = sum / ( (double)re->num_beams );

	/*
	char n[500];
	sprintf(n, "index %d-%s", index_feature, name);
	fv->setName(n);
	*/

	return 1;
}












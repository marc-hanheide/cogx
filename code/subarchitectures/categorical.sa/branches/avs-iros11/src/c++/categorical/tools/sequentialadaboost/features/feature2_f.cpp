//-----------------------------------------
// feature2_f
//
// oscar martinez
//-----------------------------------------


#include "feature2_f.h"
#include "rangeExample.h"

#include <iostream>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature2_f::Feature2_f() {
	setName("f2_f: average of diff. between consecutive beams.Only front laser.");
	setType("f2_f");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature2_f::~Feature2_f() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature2_f::getFeature(oscar::Example *e, int index_feature) {

	// casting
	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2_f::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature2_f::getFeature: list_features==NULL" << endl;
		return -1;
	}

	double sum;
	int i1, i2;

	carmen_robot_laser_message *f;
	f = &(re->flaser);
	sum = 0.0;
	for (int i=0; i < f->num_readings; i++) {
		i1 = i;
		i2 = (i+1) % f->num_readings;
		sum += (double) fabs(f->range[i1] - f->range[i2]);
	}

	fv = &(re->list_features[index_feature]);
	fv->value = sum / ( (double)f->num_readings );

	/*
	char n[500];
	sprintf(n, "index %d-%s", index_feature, name);
	fv->setName(n);
	*/

	return 1;
}










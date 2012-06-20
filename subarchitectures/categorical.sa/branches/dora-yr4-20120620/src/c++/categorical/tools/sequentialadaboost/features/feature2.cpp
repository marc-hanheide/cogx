//-----------------------------------------
// feature2
//
// oscar martinez
//-----------------------------------------


#include "feature2.h"

#include <iostream>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature2::Feature2() {
	setName("f2: average of diff. between consecutive beams");
	setType("f2");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature2::~Feature2() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature2::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	double sum;
	int i1, i2;

	sum = 0.0;
	for (int i=0; i<re->num_beams; i++) {
		i1 = i;
		i2 = (i+1) % re->num_beams;
		sum += (double) fabs(re->totalRange[i1] - re->totalRange[i2]);
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













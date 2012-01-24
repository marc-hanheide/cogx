//-----------------------------------------
// feature1
//
// oscar martinez
//-----------------------------------------


#include "feature1.h"
#include "rangeExample.h"
#include <math.h>

#include <iostream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature1::Feature1() {
	setType("1");
}


//-----------------------------------------
// constructor
//-----------------------------------------
Feature1::Feature1(int b1, int b2) {
	setup(b1,b2);
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature1::~Feature1() {
}

//-----------------------------------------
// setUp
//-----------------------------------------
int Feature1::setup(int b1, int b2) {
	beam1=b1;
	beam2=b2;

	char n[100];
	sprintf(n, "feature1: beam%d -beam%d", beam1, beam2);
	setName(n);

	return 1;
}

//-----------------------------------------
// get feature
//-----------------------------------------
int Feature1::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	if ( (beam1 < 0 && beam1 > re->num_beams -1) ||
		 (beam2 < 0 && beam2 > re->num_beams -1)    )
	{
		cerr << "ERROR: Feature1::getFeature: beam1 or beam2 out of range" << endl;
		return -1;
	}

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	fv = &(re->list_features[index_feature]);

	fv->value = (double) (re->totalRange[beam1] - re->totalRange[beam2]);

	/*
	char n[500];
	sprintf(n, "index %d-%s", index_feature, name);
	fv->setName(n);
	*/

	return 1;
}











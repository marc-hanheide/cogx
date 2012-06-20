//-----------------------------------------
// feature 24:  4pi*area/sqr(perimeter)
//
// type = 24
//
// oscar martinez
//-----------------------------------------


#include "feature24.h"
#include "feature5.h"
#include "feature6.h"


#include <math.h>
#include <values.h>
#include <iostream>
#include <fstream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature24::Feature24() {
	setName("f24:Form factor:4pi*area/sqr(perimeter)");
	setType("f24");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature24::~Feature24() {
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature24::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// area
	Feature5 f5;
	f5.getFeature(re, index_feature);
	FeatureValues *f;
	f = re->getFeature(index_feature);
	double area = f->value;

	// perimeter
	Feature6 f6;
	f6.getFeature(re, index_feature);
	f = re->getFeature(index_feature);
	double perimeter = f->value;


	// write feature
	fv = &(re->list_features[index_feature]);

	// 4pi*area/sqr(perimeter)
	fv->value = 4.0*M_PI*area / sqrt(perimeter);

	return 1;
}




































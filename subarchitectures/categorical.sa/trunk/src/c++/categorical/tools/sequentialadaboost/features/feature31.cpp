//-----------------------------------------
// feature 31:  area*4*PI / perimeter**2 
//
// type = 31
//
// oscar martinez
//-----------------------------------------


#include "feature31.h"
#include "feature5.h"
#include "feature6.h"


#include <math.h>
#include <iostream>
#include <fstream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature31::Feature31() {
	setName("f31: (4*PI*area) / (perimeter**2)");
	setType("f31");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature31::~Feature31() {
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature31::getFeature(oscar::Example *e, int index_feature) {

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
	fv->value = (area * 4.0 * M_PI) / (perimeter * perimeter);

	return 1;
}




































//-----------------------------------------
// feature 7:  perimeter/area
//
// type = 7
//
// oscar martinez
//-----------------------------------------


#include "feature7.h"
#include "feature5.h"
#include "feature6.h"

// system includes
#include <math.h>
#include <iostream>
#include <fstream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature7::Feature7() {
	setName("f7: perimeter/area");
	setType("f7");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature7::~Feature7() {
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature7::getFeature(oscar::Example *e, int index_feature) {

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
	fv->value = perimeter/area;;

	return 1;
}




































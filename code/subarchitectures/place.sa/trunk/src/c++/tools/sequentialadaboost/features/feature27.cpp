//-----------------------------------------
// feature27
//
// oscar martinez
//-----------------------------------------


#include "feature27.h"

#include <math.h>
#include <iostream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature27::Feature27() {
	setName("f27: average of relation between consecutive beams: b_i / b_{i+1}");
	setType("f27");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature27::~Feature27() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature27::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature27::getFeature: list_features==NULL" << endl;
		return -1;
	}

	double sum;
	int i1, i2;
	double rel;
	double min = 0.0001;
	
	sum = 0.0;
	for (int i=0; i<re->num_beams; i++) {
		i1 = i;
		i2 = (i+1) % re->num_beams;
		rel = ((double)re->totalRange[i1]+min) / ((double)re->totalRange[i2]+min) ; 
		if ( rel > 1.0 ) {
			rel = 1.0 / rel;
		}
		sum += (double)rel;
	}

	fv = &(re->list_features[index_feature]);
	fv->value = sum / ( (double)re->num_beams );

	return 1;
}













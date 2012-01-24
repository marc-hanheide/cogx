//-----------------------------------------------------------
// feature 2_sd:
//		standard deviation  of the difference between consecutive beams
//
// type 2_sd
//
// oscar martinez
//-----------------------------------------------------------

#include "feature2_sd.h"
#include "feature2.h"

#include <iostream>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature2_sd::Feature2_sd() {
	setName("f2_sd: std deviation of diff. between consecutive beams");
	setType("f2_sd");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature2_sd::~Feature2_sd() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature2_sd::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature2 f2;
	f2.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double mean = fv->value;


	double sum;
	int i1, i2;
	double v;
	double std;
	sum = 0.0;
	for (int i=0; i<re->num_beams; i++) {
		i1 = i;
		i2 = (i+1) % re->num_beams;
		v = fabs(re->totalRange[i1] - re->totalRange[i2]) - mean;
		v = v*v;
		sum += v;
	}

	std = sum / ( (double)re->num_beams );
	std = sqrt(std);

	fv->value = std;

	return 1;
}













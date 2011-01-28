//-----------------------------------------------------------
// feature 27_sd:
//		standard deviation  of the difference between consecutive beams
//
// type 27_sd
//
// oscar martinez
//-----------------------------------------------------------

#include "feature27_sd.h"
#include "feature27.h"

#include <math.h>

#include <iostream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature27_sd::Feature27_sd() {
	setName("f27_sd:std deviation of rel between consecutive beams: b_i/b_{i+1} ");
	setType("f27_sd");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature27_sd::~Feature27_sd() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature27_sd::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature27 f27;
	f27.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double mean = fv->value;


	double sum;
	int i1, i2;
	double v;
	double std;
	double rel;
	double min = 0.0001;
	
	sum = 0.0;
	for (int i=0; i<re->num_beams; i++) {
		i1 = i;
		i2 = (i+1) % re->num_beams;
		rel = ((double)re->totalRange[i1] + min ) / ((double)re->totalRange[i2] + min ); 
		if ( rel > 1.0 ) {
			rel = 1.0 / rel;
		}
		v = rel - mean;
		v = v*v;
		sum += v;
	}

	std = sum / ( (double)re->num_beams );
	std = sqrt(std);

	fv->value = std;

	return 1;
}













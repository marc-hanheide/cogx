//-----------------------------------------
// feature 5: area of the region covered using cross produt
// (http://geometryalgorithms.com/Archive/algorithm_0101/algorithm_0101.htm#Polygons)
//
// oscar martinez
//-----------------------------------------


#include "feature5.h"

#include <math.h>
#include <iostream>
#include <fstream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature5::Feature5() {
	setName("f5: cross product area");
	setType("f5");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature5::~Feature5() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature5::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature5::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}

	//---------------------------------------------
	// calculating area of the lasers
	//---------------------------------------------
	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	carmen_point_t *V;
	V = new carmen_point_t[n +2];

	double flaser_resolution = M_PI / (double)( (re->flaser).num_readings -1 );
	double rlaser_resolution = M_PI / (double)( (re->rlaser).num_readings -1 );
	double theta;
	carmen_point_t point;

	// store the pints in an array
	int index = 0;
	for (int j=0; j<(re->flaser).num_readings; j++) {
		theta = (re->flaser).theta -M_PI_2 + j*flaser_resolution;
		polar_to_rect( (re->flaser).range[j], theta, &point.x, &point.y );
		V[index].x = point.x;
		V[index].y = point.y;
		if( index == 0 ) {
			V[n].x = point.x;
			V[n].y = point.y;
		}
		if( index == 1 ) {
			V[n +1].x = point.x;
			V[n +1].y = point.y;
		}
		index++;
	}


	for (int j=0; j<(re->rlaser).num_readings; j++) {
		theta = (re->rlaser).theta -M_PI_2 + j*rlaser_resolution;
		polar_to_rect( (re->rlaser).range[j], theta, &point.x, &point.y);
		V[index].x = point.x;
		V[index].y = point.y;
		index++;
	}


	//calculate area
    float area = 0;
    int   i, j, k;     // indices

    for (i=1, j=2, k=0; i<=n; i++, j++, k++) {
        area += V[i].x * (V[j].y - V[k].y);
    }
    area = area / 2.0;


	delete [] V;

	// write feature
	fv = &(re->list_features[index_feature]);


	fv->value = area;

	return 1;
}




































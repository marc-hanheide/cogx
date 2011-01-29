//-----------------------------------------
// feature 6: perimeter
//
// type = 6
//
// oscar martinez
//-----------------------------------------


#include "feature6.h"

#include <math.h>
#include <iostream>
#include <fstream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature6::Feature6() {
	setName("f6: perimeter");
	setType("f6");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature6::~Feature6() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature6::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
//
//-----------------------------------------
double Feature6::distance(carmen_point_t *p1, carmen_point_t *p2) {
	double x, y, d;
	x = p2->x - p1->x;
	y = p2->y - p1->y;
	d = hypot(x, y);
	return d;
}

//-----------------------------------------
// get feature
//-----------------------------------------
int Feature6::getFeature(oscar::Example *e, int index_feature) {

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
	V = new carmen_point_t[n+1];

	double flaser_resolution = M_PI / (double)( (re->flaser).num_readings -1 );
	double rlaser_resolution = M_PI / (double)( (re->rlaser).num_readings -1 );
	double theta;
	carmen_point_t point;

	// store the points in an array
	int index = 0;
	for (int j=0; j<(re->flaser).num_readings; j++) {
		theta = (re->flaser).theta -M_PI_2 + j*flaser_resolution;
		polar_to_rect( (re->flaser).range[j], theta, &point.x, &point.y );
		V[index].x = point.x;
		V[index].y = point.y;
		if ( index==0 ) {
			V[n].x = point.x;
			V[n].y = point.y;
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

	//calculate perimeter
	double perimeter =0 ;
	double dist;
    for (int i=0; i<n; i++) {
		dist = distance(&V[i], &V[i+1]);
        perimeter = perimeter + dist;
    }

	delete [] V;

	// write feature
	fv = &(re->list_features[index_feature]);
	fv->value = perimeter;

	return 1;
}




































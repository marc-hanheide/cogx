//-----------------------------------------------------------
// feature 29_sd:
//		standard deviation  of the distances from end of beams to centroid / max distance
// type 29_sd
//
// oscar martinez
//-----------------------------------------------------------

#include "feature29_sd.h"
#include "feature29.h"
#include "feature5.h"

#include <iostream>
#include <math.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature29_sd::Feature29_sd() {
	setName("f29_sd: std deviation of distance to centroid / max distance");
	setType("f29_sd");
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature29_sd::~Feature29_sd() {
}

//-----------------------------------------
//
//-----------------------------------------
void Feature29_sd::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
//
//-----------------------------------------
double Feature29_sd::distance(carmen_point_t *p1, carmen_point_t *p2) {
	double x, y, d;
	x = p2->x - p1->x;
	y = p2->y - p1->y;
	d = hypot(x, y);
	return d;
}

//-----------------------------------------
// get feature
//-----------------------------------------
int Feature29_sd::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature2::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;

	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature1::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// mean
	Feature29 f29;
	f29.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	double mean = fv->value;

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	carmen_point_t *V;
	V = new carmen_point_t[n];

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
		//cerr << V[index].x << " " << V[index].y << endl;
		index++;
	}

	for (int j=0; j<(re->rlaser).num_readings; j++) {
		theta = (re->rlaser).theta -M_PI_2 + j*rlaser_resolution;
		polar_to_rect( (re->rlaser).range[j], theta, &point.x, &point.y);
		V[index].x = point.x;
		V[index].y = point.y;
		//cerr << V[index].x << " " << V[index].y << endl;
		index++;
	}

	double max = 0.0;
	double d;	
	
	for (int i=0; i<n; i++) {
		d = distance(&(f29.centroid), &V[i]);
		if ( d > max ) {
			max = d;
		}
	}

	// standard deviation of distances to centroid
	double v;
	double sum=0;
	for (int i=0; i<n; i++) {
		v = (distance(&(f29.centroid), &V[i]) / max) - mean;
		v = v*v;
		sum += v;
	}
	double std = sum / ( (double)n );
	std = sqrt(std);

	delete [] V;

	// write feature
	fv->value = std;

	return 1;
}













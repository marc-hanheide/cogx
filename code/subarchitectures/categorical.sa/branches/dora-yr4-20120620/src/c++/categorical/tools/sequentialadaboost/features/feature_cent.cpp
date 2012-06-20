//-----------------------------------------
// feature cent: centroid of a polygon and area
//
// type = cent
//
// oscar martinez
//-----------------------------------------


#include "feature_cent.h"
#include "feature5.h"


#include <math.h>
#include <iostream>
#include <fstream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature_cent::Feature_cent() {
	setName("centroid");
	setType("cent");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature_cent::~Feature_cent() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature_cent::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}



//-----------------------------------------
// get feature
//-----------------------------------------
int Feature_cent::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}

	//---------------------------------------------
	// calculating centroid
	//---------------------------------------------

	// area
	Feature5 f5;
	f5.getFeature(re, index_feature);
	fv = re->getFeature(index_feature);
	area = fv->value;

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	carmen_point_t *V;
	V = new carmen_point_t[n +1];

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
		if( index == 0 ) {
			V[n].x = point.x;
			V[n].y = point.y;
		}
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

	//calculate centroid
	double v;

	// cx
	double sum = 0;
    for (int i=0; i<n; i++) {
		v = (V[i].x + V[i+1].x) * ( V[i].x * V[i+1].y - V[i+1].x * V[i].y );
		sum = sum + v;

		cerr << V[i].x << " " << V[i].y << endl;
    }
    centroid.x = sum / (6.0 * area);

	// cy
	sum = 0;
    for (int i=0; i<n; i++) {
		v = (V[i].y + V[i+1].y) * ( V[i].x * V[i+1].y - V[i+1].x * V[i].y );
		sum = sum + v;
    }
    centroid.y = sum / (6.0 * area);

	//cerr << centroid.x << " " << centroid.y << endl;

	delete V;

	return 1;
}




































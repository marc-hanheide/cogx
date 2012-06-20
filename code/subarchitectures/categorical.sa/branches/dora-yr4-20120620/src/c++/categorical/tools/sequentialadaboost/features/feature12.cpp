//-----------------------------------------
// feature 12:
// Second order spatial moments
// (Computer and robot vision VI, p73)
//
// type = 12
//
// oscar martinez
//-----------------------------------------


#include <math.h>
#include <iostream>
#include <fstream>

#include "feature12.h"
#include "feature5.h"
#include "feature_cent.h"

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature12::Feature12() {
	setName("f12: major and minor axis of ellipse");
	setType("f12");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature12::~Feature12() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature12::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature12::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}

	// centroid
	/*
	Feature_cent f_cent;
	f_cent.getFeature(re, index_feature);
	double area = f_cent.area;
	carmen_point_t centroid = f_cent.centroid;
	*/

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	carmen_point_t *V;
	V = new carmen_point_t[n];

	double flaser_resolution = M_PI / (double)( (re->flaser).num_readings -1 );
	double rlaser_resolution = M_PI / (double)( (re->rlaser).num_readings -1 );
	double theta;
	carmen_point_t point;

	// store the points in an array
	int index = 0;
	centroid.x = 0;
	centroid.y = 0;
	for (int j=0; j<(re->flaser).num_readings; j++) {
		theta = (re->flaser).theta -M_PI_2 + j*flaser_resolution;
		polar_to_rect( (re->flaser).range[j], theta, &point.x, &point.y );
		V[index].x = point.x;
		V[index].y = point.y;
		centroid.x += point.x;
		centroid.y += point.y;
		//cerr << V[index].x << " " << V[index].y << endl;
		index++;
	}

	for (int j=0; j<(re->rlaser).num_readings; j++) {
		theta = (re->rlaser).theta -M_PI_2 + j*rlaser_resolution;
		polar_to_rect( (re->rlaser).range[j], theta, &point.x, &point.y);
		V[index].x = point.x;
		V[index].y = point.y;
		centroid.x += point.x;
		centroid.y += point.y;
		//cerr << V[index].x << " " << V[index].y << endl;
		index++;
	}
	centroid.x = centroid.x / n;
	centroid.y = centroid.y / n;

	double v;
	// urr
	urr = 0;
	for(int i=0; i<n; i++) {
		v = ( V[i].y - centroid.y )*( V[i].y - centroid.y );
		urr = urr + v;
	}
	//urr = urr / area;
	urr = urr / n;

	// urc
	urc = 0;
	for(int i=0; i<n; i++) {
		v = ( V[i].y - centroid.y )*( V[i].x - centroid.x );
		urc = urc + v;
	}
	//urc = urc / area;
	urc = urc / n;

	// ucc
	ucc = 0;
	for(int i=0; i<n; i++) {
		v = ( V[i].x - centroid.x )*( V[i].x - centroid.x );
		ucc = ucc + v;
	}
	//ucc = ucc / area;
	ucc = ucc / n;


	// axis
	if ( urc ==0 && urr > ucc ) {
		major_axis = 4.0 * pow(urr, 0.5);
		minor_axis = 4.0 * pow(ucc, 0.5);

		major_angle = - M_PI_2;
		minor_angle = 0;
	}
	else
	if ( urc == 0 && urr <= ucc ) {
		major_axis = 4.0 * pow(ucc, 0.5);
		minor_axis = 4.0 * pow(urr, 0.5);

		major_angle = 0;
		minor_angle = -M_PI_2;
	}
	else
	if ( urc != 0 && urr <= ucc ) {
		double a, b, c ,d;
		a = (urr -ucc)*(urr -ucc);
		b = pow ( a + 4.0*urc*urc, 0.5);
		c = 8.0*(urr + ucc + b);
		d = pow(c, 0.5);
		major_axis = d;

		a = (urr -ucc)*(urr -ucc);
		b = pow ( a + 4.0*urc*urc, 0.5);
		c = 8.0*(urr + ucc - b);
		d = pow(c, 0.5);
		minor_axis = d;

		// angles
		a = (urr -ucc)*(urr -ucc);
		b = pow ( a + 4.0*urc*urc, 0.5);
		c = urr - ucc + b;
		d = (-2.0*urc) / c;
		major_angle = atan(d);

		minor_angle = major_angle + M_PI_2;
	}

	if ( urc != 0 && urr > ucc ) {
		double a, b, c ,d;
		a = (urr -ucc)*(urr -ucc);
		b = pow ( a + 4.0*urc*urc, 0.5);
		c = 8.0*(urr + ucc + b);
		d = pow(c, 0.5);
		major_axis = d;

		a = (urr -ucc)*(urr -ucc);
		b = pow ( a + 4.0*urc*urc, 0.5);
		c = 8.0*(urr + ucc - b);
		d = pow(c, 0.5);
		minor_axis = d;

		// angles
		a = (ucc -urr)*(ucc -urr);
		b = pow ( a + 4.0*urc*urc, 0.5);
		c = pow(ucc + urr +b, 0.5);
		d = c / (-2.0*urc);
		major_angle = atan(d);

		minor_angle = major_angle + M_PI_2;
	}

	polar_to_rect(major_axis, major_angle, &major.x, &major.y);
	major.x += centroid.x;
	major.y += centroid.y;

	polar_to_rect(minor_axis, minor_angle, &minor.x, &minor.y);
	minor.x += centroid.x;
	minor.y += centroid.y;



	// write feature
	fv = &(re->list_features[index_feature]);

	delete V;

	return 1;
}




































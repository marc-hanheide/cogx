//-----------------------------------------
// feature 20:
//
// moments and invariants. Digital Image processing, gonzalez, p514-515
//
// type = 20
//
// oscar martinez
//-----------------------------------------


#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature20.h"

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature20::Feature20() {
	setName("f20:moments");
	setType("f20");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature20::~Feature20() {
}

//-----------------------------------------
//
//-----------------------------------------
void Feature20::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature20::getFeature(oscar::Example *e, int index_feature) {

	//cerr << "Feature20::getFeature()" << endl;

	oscar::RangeExample *re = (oscar::RangeExample *)e;

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

	// center
	ux=0;
	for (int i=0; i<n; i++) {
		ux += V[i].x;
	}
	ux = ux / n;

	uy=0;
	for (int i=0; i<n; i++) {
		uy += V[i].y;
	}
	uy = uy / n;

	// central moments
	u11=0;
	for (int i=0; i<n; i++) {
		u11 += (V[i].x - ux)*(V[i].y - uy);
	}


	u20=0;
	for (int i=0; i<n; i++) {
		u20 += (V[i].x - ux)*(V[i].x - ux);
	}

	u02=0;
	for (int i=0; i<n; i++) {
		u02 += (V[i].y - uy)*(V[i].y - uy);
	}

	u30=0;
	for (int i=0; i<n; i++) {
		u30 += (V[i].x - ux)*(V[i].x - ux)*(V[i].x - ux);
	}

	u12=0;
	for (int i=0; i<n; i++) {
		u12 += (V[i].x - ux)*(V[i].y - uy)*(V[i].y - uy);
	}

	u21=0;
	for (int i=0; i<n; i++) {
		u21 += (V[i].x - ux)*(V[i].x - ux)*(V[i].y - uy);
	}


	u03=0;
	for (int i=0; i<n; i++) {
		u03 += (V[i].y - uy)*(V[i].y - uy)*(V[i].y - uy);
	}

	double mu;
	
	u00=n*n;  // I am not quite sure but according to the formula the moment 00 must be n*n

	// normalized central moments
	mu = ((1.0 + 1.0)/2.0) + 1.0;
	n_u11= u11 / pow(u00, mu);

	mu = ((2.0 + 0.0)/2.0) +1.0;
	n_u20= u20 / pow(u00, mu);

	mu = ((0.0 + 2.0) / 2.0) +1.0;
	n_u02= u02 / pow(u00, mu);

	mu = ((1.0 + 2.0 ) / 2.0) +1.0;
	n_u12= u12 / pow(u00, mu);

	mu = ((2.0 + 1.0)/2.0) +1.0;
	n_u21= u21 / pow(u00, mu);

	mu = ((3.0 + 0.0)/2.0) +1.0;
	n_u30= u30 / pow(u00, mu);

	mu = ((0.0 + 3.0)/2.0) +1.0;
	n_u03= u03 / pow(u00, mu);

	// invariants
	inv[0] = n_u20 + n_u02;

	inv[1] = pow(n_u20 - n_u02, 2) + 4.0*n_u11*n_u11;

	inv[2] = pow(n_u30 - 3.0*n_u12, 2) + pow(3.0*n_u21 - n_u03, 2);

	inv[3] = pow(n_u30 + n_u12, 2) + pow(n_u21 + n_u03, 2);

	inv[4] = (n_u30 - 3.0*n_u12)*(n_u30 + n_u12)*( pow(n_u30 + n_u12,2) - 3.0*pow(n_u21+n_u03,2) )
		   + (3.0*n_u21 - n_u03)*(n_u21 + n_u03)*( 3.0*pow(n_u30 + n_u12,2) - pow(n_u21+n_u03,2));

	inv[5] = (n_u20-n_u02)*( pow(n_u30+n_u12,2) - pow(n_u21+n_u03,2)) +
		   4.0*n_u11*(n_u30 +n_u12)*(n_u21+n_u03);

	inv[6] = (3.0*n_u21-n_u03)*(n_u30+n_u12)*( pow(n_u30+n_u12,2) - 3.0*pow(n_u21+n_u03,2))
		   + (3.0*n_u12-n_u30)*(n_u21+n_u03)*( 3.0*pow(n_u30+n_u12,2) - pow(n_u21+n_u03,2));

	delete [] V;

	return 1;
}



































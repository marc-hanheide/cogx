//-----------------------------------------
// feature 11:
//    extremal points. Calculate the extremal points. Don't use for classification. Use only
//    for calculation new features. (Computer and robot vision VI, p62)
//
// type = 11
//
// oscar martinez
//-----------------------------------------


#include <math.h>
#include <iostream>
#include <fstream>
#include "feature11.h"
#include <math.h>
#include <values.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature11::Feature11() {
	setName("f11:extremal points");
	setType("f11");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature11::~Feature11() {
}


//-----------------------------------------
//
//-----------------------------------------
void Feature11::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature11::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature11::getFeature: list_features==NULL" << endl;
		return -1;
	}

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	carmen_point_t *V;
	V = new carmen_point_t[n];

	double flaser_resolution = M_PI / (double)( (re->flaser).num_readings -1 );
	double rlaser_resolution = M_PI / (double)( (re->rlaser).num_readings -1 );
	double theta;
	carmen_point_t point;

	// store the points in an array
	double cmin = MAXDOUBLE;
	double cmax = MINDOUBLE;
	double rmin = MAXDOUBLE;
	double rmax = MINDOUBLE;
	int index = 0;
	for (int j=0; j<(re->flaser).num_readings; j++) {
		theta = (re->flaser).theta -M_PI_2 + j*flaser_resolution;
		polar_to_rect( (re->flaser).range[j], theta, &point.x, &point.y );
		V[index].x = point.x;
		V[index].y = point.y;

		index++;
		if ( point.x < cmin ) {
			cmin = point.x;
		}
		if ( point.x > cmax ) {
			cmax = point.x;
		}

		if ( point.y < rmin ) {
			rmin = point.y;
		}
		if ( point.y > rmax ) {
			rmax = point.y;
		}
	}

	for (int j=0; j<(re->rlaser).num_readings; j++) {
		theta = (re->rlaser).theta -M_PI_2 + j*rlaser_resolution;
		polar_to_rect( (re->rlaser).range[j], theta, &point.x, &point.y);
		V[index].x = point.x;
		V[index].y = point.y;
		index++;

		if ( point.x < cmin ) {
			cmin = point.x;
		}
		if ( point.x > cmax ) {
			cmax = point.x;
		}

		if ( point.y < rmin ) {
			rmin = point.y;
		}
		if ( point.y > rmax ) {
			rmax = point.y;
		}
	}

	double offset_y, offset_x;
	offset_y = rmax;
	offset_x = cmin;

	// (0,0) in book is top left
	rmax = rmax - rmin;
	cmax = cmax - cmin;

	rmin = 0;
	cmin = 0;

	double c1, c2, c3, c4, c5, c6, c7, c8;
	double r1, r2, r3, r4, r5, r6, r7, r8;
	r1 = rmin;
	r2 = rmin;

	c3 = cmax;
	c4 = cmax;

	r5 = rmax;
	r6 = rmax;

	c7 = cmin;
	c8 = cmin;

	c1 = MAXDOUBLE;
	c2 = MINDOUBLE;
	r3 = MAXDOUBLE;
	r4 = MINDOUBLE;
	c5 = MINDOUBLE;
	c6 = MAXDOUBLE;
	r7 = MINDOUBLE;
	r8 = MAXDOUBLE;

	double c, r;


	for(int i=0; i<n; i++) {
		c = V[i].x - offset_x;
		r = offset_y - V[i].y;
		//r = V[i].y;

		//cerr << c << " " << 10-r << endl;
		cerr << c << " " << r << endl;

		if ( r == rmin ) {
			if ( c < c1 ) {
				c1 = c;
			}
			if ( c > c2 ) {
				c2 = c;
			}
		}

		if ( c == cmax ) {
			if ( r < r3 ) {
				r3 = r;
			}
			if ( r > r4 ) {
				r4 = r;
			}
		}

		if ( r == rmax ) {
			if ( c < c6 ) {
				c6 = c;
			}
			if ( c > c5 ) {
				c5 = c;
			}
		}

		if ( c == cmin ) {
			if ( r < r8 ) {
				r8 = r;
			}
			if ( r > r7 ) {
				r7 = r;
			}
		}

	}


	p1.x = c1;
	p1.y = r1;

	p2.x = c2;
	p2.y = r2;

	p3.x = c3;
	p3.y = r3;

	p4.x = c4;
	p4.y = r4;

	p5.x = c5;
	p5.y = r5;

	p6.x = c6;
	p6.y = r6;

	p7.x = c7;
	p7.y = r7;

	p8.x = c8;
	p8.y = r8;

	// write feature
	fv = &(re->list_features[index_feature]);


	return 1;
}




































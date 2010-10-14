//-----------------------------------------
// feature 25:
//
// Look for the two local minima in beams legth with minimum value.
// Distance between these two minima.
//
// type = 25
//
// oscar martinez
//-----------------------------------------

#include <math.h>
#include <iostream>
#include <fstream>

#include <float.h>
#include <limits.h>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature25.h"

#define MAXDOUBLE  1.79769313486231470e+308

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature25::Feature25() {
	setName("f25: euclidean distance between 2 minimum");
	setType("f25");
	debug = false;
	deviation = -1.0;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature25::~Feature25() {
}


//-----------------------------------------
// setup
//-----------------------------------------
void Feature25::setup(double d) {
	deviation = d;
	char name[500];
	sprintf(name, "f25: euclidean distance between 2 minimum deviation %le", deviation);
	setName(name);
}


//-----------------------------------------
//
//-----------------------------------------
void Feature25::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}

//-----------------------------------------
//
//-----------------------------------------
double Feature25::calcDistance(carmen_point_t *p1, carmen_point_t *p2) {
	double diffx = p2->x - p1->x;
	double diffy = p2->y - p1->y;

	return hypot(diffx, diffy);
}

//-----------------------------------------
// get feature
//-----------------------------------------
int Feature25::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;


	if ( deviation < 0.0 ) {
		cerr << "ERROR: Feature25::getFeature(...): deviation NOT set" << endl;
		return -1;
	}

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature25::getFeature: list_features==NULL" << endl;
		return -1;
	}
	

	fv = re->getFeature(index_feature);

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;

	double flaser_resolution = 0;
	if ((re->flaser).num_readings > 0)
	  flaser_resolution = M_PI / (double)( (re->flaser).num_readings -1 );

	double rlaser_resolution = 0;
	if ((re->rlaser).num_readings)
	  rlaser_resolution =  M_PI / (double)( (re->rlaser).num_readings -1 );

	//double flaser_resolution = M_PI / (double)( (re->flaser).num_readings -1 );
	//double rlaser_resolution = M_PI / (double)( (re->rlaser).num_readings -1 );

	double theta;
	carmen_point_t *V;
	V = new carmen_point_t[n];
	carmen_point_t point;


	// store the points in an array
	int index = 0;
	for (int j=0; j<(re->flaser).num_readings; j++) {
		theta = (re->flaser).theta -M_PI_2 + j*flaser_resolution;
		polar_to_rect( (re->flaser).range[j], theta, &point.x, &point.y );
		V[index].x = point.x;
		V[index].y = point.y;
		if ( debug) {
			//cerr << "0 0" << endl;
			cerr << V[index].x << " " << V[index].y << endl;
		}
		index++;
	}

	for (int j=0; j<(re->rlaser).num_readings; j++) {
		theta = (re->rlaser).theta -M_PI_2 + j*rlaser_resolution;
		polar_to_rect( (re->rlaser).range[j], theta, &point.x, &point.y);
		V[index].x = point.x;
		V[index].y = point.y;
		if ( debug ) {
			//cerr << "0 0" << endl;
			cerr << V[index].x << " " << V[index].y << endl;
		}
		index++;
	}


	// look for the first minimum
	min1 = MAXDOUBLE;
	index1=-1;


	first_min= MAXDOUBLE;
	first_index = -1;
	last_index =-1;
	npoints=0;
	bool start=false;
	for(int i=0; i<n; i++) {
		if (debug) {
			cerr << i << " " << re->totalRange[i] << endl;
		}
		// look for the first min
		if ( re->totalRange[i] < (first_min - deviation) ) {
			start = true;
			first_min = re->totalRange[i];
			first_index = i;
			npoints = 1;
			last_index = first_index;
			min1 = re->totalRange[i];
		}
		else
		if ( start &&
			 (re->totalRange[i] > (first_min - deviation)) &&
			 (re->totalRange[i] < (first_min + deviation))   ) {
			// still in minimum
			first_min = re->totalRange[i];
			last_index = i;
			npoints++;
			min1 += re->totalRange[i];
		}
		else {
			start= false;
			// larger value
			// do nothing
		}
	}



	index1 = (last_index + first_index) / 2;
	min1 = min1 / (double)npoints;

	if ( debug )  {
		cerr << index1 << " " << min1 << endl;
	}

	// look for the second minimum
	bool again;
	do {
		again = false;
		if ( debug ) {
			cout << "1------------: " << endl;
			cout << "first_index:" << first_index << endl;
			cout << "last_index:" << last_index << endl;
		}

		// look for min2
		min2 = MAXDOUBLE;
		index2=-1;
		first_min2= MAXDOUBLE;
		first_index2 = -1;
		last_index2 =-1;
		npoints2=0;
		start = false;
		bool next = false;

		for(int i=0; i<n; i++) {
			// index not in range
			next = false;
			if ( first_index < last_index ){
				if ( (i < first_index) || (i > last_index) ) {
					next = true;
				}
			}
			else {
				if ( (i > last_index) && (i < first_index) ) {
					next = true;
				}
			}
			if ( next ) {
				// look for the first min
				if ( re->totalRange[i] < (first_min2 - deviation) ) {
					start = true;
					first_min2 = re->totalRange[i];
					first_index2 = i;
					npoints2 = 1;
					last_index2 = first_index2;
					min2 = re->totalRange[i];

					if ( debug) {
						cout << "first_min2: " << first_min2 << endl;
						cout << "first_index2: " <<  i << endl;
					}
				}
				else
				if ( start &&
					(re->totalRange[i] > (first_min2 - deviation)) &&
					(re->totalRange[i] < (first_min2 + deviation))   ) {
					// still in minimum
					first_min2 = re->totalRange[i];
					last_index2 = i;
					npoints2++;
					min2 += re->totalRange[i];
				}
				else {
					// larger value
					// do nothing
					start = false;
				}
			}
		}



		if ( debug ) {
			cout << "2------------: " << endl;
			cout << "first_index2:" << first_index2 << endl;
			cout << "last_index2:" << last_index2 << endl;
		}
		// check for the same interval
		if ( ((last_index2+1)%n == first_index) &&
			 (re->totalRange[last_index2] < re->totalRange[first_index] + deviation) &&
			 (re->totalRange[last_index2] > re->totalRange[first_index] - deviation) )
		{
			// it is the same interval
			first_index = first_index2;
			// look again
			again = true;
		}
		else
		if ( ((last_index+1)%n == first_index2) &&
			 (re->totalRange[last_index] < re->totalRange[first_index2] + deviation) &&
			 (re->totalRange[last_index] > re->totalRange[first_index2] - deviation) )
		{
			// it is the same interval
			last_index = last_index2;
			// look again
			again = true;
		}

	} while (again);


	index2 = (last_index2 + first_index2) / 2;
	min2 = min2 / (double)npoints2;

	if ( debug )  {
		cerr << index2 << " " << min2 << endl;
	}

	// distance between points
	point1 = V[index1];
	point2 = V[index2];
	distance = calcDistance(&point1, &point2);
	beam_distance = abs(index1 -index2);

	fv->value = distance;

	delete [] V;

	return 1;
}





































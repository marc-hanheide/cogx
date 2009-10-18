//-----------------------------------------
// feature 4: area of the region covered by lasers with morfology
// type = 4
//
// oscar martinez
//-----------------------------------------


#include "feature4.h"

#include <math.h>
#include <iostream>
#include <fstream>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature4::Feature4() {
	setName("f4:area");
	setType("f4");
	resolution = 0.1;
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature4::~Feature4() {
}


//-----------------------------------------
// setup
//-----------------------------------------
void Feature4::setup(double re) {
	resolution = re;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature4::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}

	fv = &(re->list_features[index_feature]);

	fv->value = value;

	return 1;
}


//-----------------------------------------
// calculate
//-----------------------------------------
int Feature4::calculate(oscar::Example *e) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	double x,y;
	double angle_incr;
	carmen_robot_laser_message *fl, *rl;

	double max_x = -MAXDOUBLE;
	double max_y = -MAXDOUBLE;
	double min_x = MAXDOUBLE;
	double min_y = MAXDOUBLE;


	// for each range get the maximum and minimum coordinate
	fl = &(re->flaser);
	rl = &(re->rlaser);
	angle_incr =  M_PI / (double)fl->num_readings;

	// front laser
	for(int j=0; j < fl->num_readings; j++) {
		end_of_beam(fl->x, fl->y, fl->theta - M_PI_2 + ((double)j)*angle_incr,
					fl->range[j], &x, &y);
		max_x = carmen_fmax(max_x, x);
		max_y = carmen_fmax(max_y, y);
		min_x = carmen_fmin(min_x, x);
		min_y = carmen_fmin(min_y, y);
	}

	// rear laser
	angle_incr =  M_PI / (double)rl->num_readings;
	for(int j=0; j < rl->num_readings; j++) {
		end_of_beam(rl->x, rl->y, rl->theta - M_PI_2 + ((double)j)*angle_incr,
					rl->range[j], &x, &y);
		max_x = carmen_fmax(max_x, x);
		max_y = carmen_fmax(max_y, y);
		min_x = carmen_fmin(min_x, x);
		min_y = carmen_fmin(min_y, y);
	}

	// paint map
	min_x -= 1.0;
	min_y -= 1.0;
	max_x += 1.0;
	max_y += 1.0;

	MAP map=init_map(max_x-min_x, max_y-min_y, // size
					 resolution,       // resolution in meters
					 min_x, // offset x
					 min_y, // offset y
					 0);    // prior. Not probability prior. Prior for hints and sum.
					 		     // see mapping.cpp

	MAP per_map=init_map(max_x-min_x, max_y-min_y, // size
					 resolution,       // resolution in meters
					 min_x, // offset x
					 min_y, // offset y
					 0);    // prior. Not probability prior. Prior for hints and sum.
					 		     // see mapping.cpp

	//origin of rlaser same as flaser for avoiding the line in the middle
	rl->x = fl->x;
	rl->y = fl->y;

	laser_to_map( &map, fl);
	laser_to_map( &map, rl);

	double** prob_map = probability_grid(&map);
	int **binary_map;

	binary_map = new (int*)[map.size_x];
	for(int i=0; i<map.size_x; i++) {
		binary_map[i] = new int[map.size_y];
	}
	prob_to_binary(map.size_x, map.size_y, prob_map, binary_map);

	// calculate area of the map in pixels
	double area=0;
	for ( int i=0; i<map.size_x; i++) {
		for ( int j=0; j<map.size_y; j++) {
			if ( binary_map[i][j] == 1 ) {
				area += resolution * resolution; // meters**2;
			}
		}
	}

	//write_map_to_ppm(&map, "map.ppm");
	//write_binary_map_to_ppm(map.size_x, map.size_y, binary_map, "binary_map.ppm");
	value = area;

	return 1;
}


































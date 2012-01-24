//-----------------------------------------
// feature 15_2:
//
// percentage of number of points that lie in lines extracted from scans.
//
// type = 15_2
//
// oscar martinez
//-----------------------------------------


#include "feature15_2.h"

#include "rangeExample.h"

#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>




using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature15_2::Feature15_2() {
	setName("f15_2:percentage of number of points in lines.");
	setType("f15_2");
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature15_2::~Feature15_2() {
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature15_2::getFeature(oscar::Example *e, int index_feature) {

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	//cerr << "Feature1::getFeature(oscar::RangeExample *re, int index_feature)" << endl;

	FeatureValues *fv;
	if (re->list_features == NULL ) {
		cerr << "ERROR: Feature4::getFeature: list_features==NULL" << endl;
		return -1;
	}
	fv = re->getFeature(index_feature);

	carmen_linemapping_segment_set front_segments_local;
	carmen_linemapping_segment_set rear_segments_local;
	int num_points;

	// front laser
	carmen_linemapping_init(0, NULL);

	// some parameters
	/*
	carmen_linemapping_set_laser_max_length(80);
	carmen_linemapping_set_sam_tolerance(0.3);  // orig 0.03
	carmen_linemapping_set_sam_max_gap(0.5);  // orig  0.2
	carmen_linemapping_set_sam_min_length(0.3); // orig 0.2
	carmen_linemapping_set_sam_min_num(2);
	carmen_linemapping_set_sam_fit_split(false);

	carmen_linemapping_set_merge_max_dist(2); // orig 0.1
	carmen_linemapping_set_merge_overlap_min_percent(0.0); // orig 0.5
	carmen_linemapping_set_merge_overlap_min_length(0.0);  // orig 0.5
	carmen_linemapping_set_merge_uniformly_distribute_dist(0.5); // orig 0.05
	*/

	front_segments_local = carmen_linemapping_get_segments_from_scan(&(re->flaser), CARMEN_LINEMAPPING_ROBOT_FRAME);
	num_points=0;
	for(int i=0; i < front_segments_local.num_segs; i++) {
		num_points += front_segments_local.segs[i].weight;
	}

	// rear laser
	rear_segments_local = carmen_linemapping_get_segments_from_scan(&(re->rlaser), CARMEN_LINEMAPPING_ROBOT_FRAME);
	for(int i=0; i < rear_segments_local.num_segs; i++) {
		num_points += rear_segments_local.segs[i].weight;
	}

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;

	fv->value = (double)num_points / (double)n;

	return 1;
}





































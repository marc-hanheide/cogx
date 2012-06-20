//-----------------------------------------
// Each example is a range beam.
// It Includes front laser beams and rear laser beams.
//
// oscar martinez
//-----------------------------------------


#include "rangeExample.h"

#include <string.h>
#include <iostream>


using namespace std;
using namespace oscar;

//-----------------------------------------
// constructors
//-----------------------------------------
RangeExample::RangeExample() {
	flaser.range = NULL;
	rlaser.range = NULL;
	totalRange = NULL;
	list_features = NULL;
	num_features = 0;
	filename = NULL;
	num_beams = 0;
}

//-----------------------------------------
// constructors
//-----------------------------------------
RangeExample::RangeExample(int n) {
	flaser.range = NULL;
	rlaser.range = NULL;
	totalRange = NULL;
	list_features = NULL;
	setNumFeatures(n);
	filename = NULL;
}



//-----------------------------------------
// destructor
//-----------------------------------------
RangeExample::~RangeExample(){
	clean();
}

//-----------------------------------------
// clean range information
//-----------------------------------------
int RangeExample::clean() {

	if (totalRange != NULL) {
		delete [] totalRange;
		totalRange = NULL;
	}

	flaser.range = NULL;
	rlaser.range = NULL;

	if ( filename != NULL ) {
		delete filename;
		filename = NULL;
	}

	return 1; //ok
}


//-----------------------------------------
// set the name of the file to read
//-----------------------------------------
int RangeExample::setFilename(char *f) {
	int l = strlen(f);
	filename = new char[l+2];
	strcpy(filename, f);

	return 1;
}

//-----------------------------------------
// prepare space for n features
//-----------------------------------------
int RangeExample::setNumFeatures(int n) {

	//cerr << "RangeExample::setNumFeatures: " << n << endl;

	if ( list_features != NULL ) {
		delete [] list_features;
		list_features = NULL;
	}

	num_features = n;
	list_features = new FeatureValues[num_features];

	return 1; //ok
}

//-----------------------------------------
// set one feature
//-----------------------------------------
int RangeExample::setFeature(int n, FeatureValues *f) {
	list_features[n] = *f;

	return 1;
}

//-----------------------------------------
// get one feature
//-----------------------------------------
FeatureValues * RangeExample::getFeature(int n) {
	return &(list_features[n]);
}



//-----------------------------------------
// 
//-----------------------------------------
void RangeExample::scale(float factor) {
	
	// beams
	for (int i=0; i<flaser.num_readings; i++) {
		flaser.range[i] = flaser.range[i] * factor;
	}

	for (int i=0; i<rlaser.num_readings; i++) {
		rlaser.range[i] = rlaser.range[i] * factor;	
	}	
}


//-----------------------------------------
// Reads laser from a ifstream.
// The source file must be created with the utility "readlogfile"
// return  1  ok
//		  -1 error
//-----------------------------------------
int RangeExample::readRange(ifstream &ifs) {

	int i;
	double kk;

	ifs >> pose.x;
	ifs >> pose.y;
	ifs >> pose.theta;

	// type is set externally
 	ifs >> type;
	//ifs >> i;

	//front laser
	ifs >> flaser.num_readings;
	ifs >> flaser.x;
	ifs >> flaser.y;
	ifs >> flaser.theta;
	ifs >> flaser.odom_x;
	ifs >> flaser.odom_y;
	ifs >> flaser.odom_theta;
	ifs >> flaser.timestamp;

	//rear laser
	ifs >> rlaser.num_readings;
	ifs >> rlaser.x;
	ifs >> rlaser.y;
	ifs >> rlaser.theta;
	ifs >> rlaser.odom_x;
	ifs >> rlaser.odom_y;
	ifs >> rlaser.odom_theta;
	ifs >> rlaser.timestamp;

	num_beams = flaser.num_readings + rlaser.num_readings;

	// beams
	totalRange = new float[num_beams];

	// point to first frontal beam
	flaser.range = &totalRange[0];

	for (i=0; i<flaser.num_readings; i++) {
		ifs >> totalRange[i];
		ifs >> kk; // x
		ifs >> kk; // y
	}

	// point to first rear beam
	rlaser.range = &totalRange[flaser.num_readings];

	for (i=0; i<rlaser.num_readings; i++) {
		ifs >> totalRange[i + flaser.num_readings];
		ifs >> kk; // x
		ifs >> kk; // y
	}

	return 1; //ok
}


//-----------------------------------------
// init example from lasers
//-----------------------------------------
int RangeExample::initRange(carmen_robot_laser_message *frontlaser,
					  carmen_robot_laser_message *rearlaser)
{
	flaser.num_readings = frontlaser->num_readings;
	flaser.x = frontlaser->x;
	flaser.y = frontlaser->y;
	flaser.theta = frontlaser->theta;
	flaser.odom_x = frontlaser->odom_x;
	flaser.odom_y = frontlaser->odom_y;
	flaser.odom_theta = frontlaser->odom_theta;
	flaser.timestamp = frontlaser->timestamp;

	rlaser.num_readings = rearlaser->num_readings;
	rlaser.x = rearlaser->x;
	rlaser.y = rearlaser->y;
	rlaser.theta = rearlaser->theta;
	rlaser.odom_x = rearlaser->odom_x;
	rlaser.odom_y = rearlaser->odom_y;
	rlaser.odom_theta = rearlaser->odom_theta;
	rlaser.timestamp = rearlaser->timestamp;

	num_beams = flaser.num_readings + rlaser.num_readings;

	// beams
	totalRange = new float[num_beams];

	// point to first frontal beam
	flaser.range = &totalRange[0];
	float *p;
	for (int i=0; i<flaser.num_readings; i++) {
		p = frontlaser->range;
		totalRange[i] = p[i];
	}

	// point to first rear beam
	rlaser.range = &totalRange[flaser.num_readings];

	for (int i=0; i<rlaser.num_readings; i++) {
		p = rearlaser->range;
		totalRange[i + flaser.num_readings] = p[i];
	}

	return 1; // ok
}




//-----------------------------------------
// debugging purpose
//-----------------------------------------
int RangeExample::debug() {
	int i;

	//front laser
	cerr << "front laser-----------------------" << endl;
	cerr << flaser.num_readings << " ";
	cerr << flaser.x << " ";
	cerr << flaser.y << " ";
	cerr << flaser.theta << " ";
	cerr << flaser.odom_x << " ";
	cerr << flaser.odom_y << " ";
	cerr << flaser.odom_theta << " ";
	cerr << flaser.timestamp << endl;

	//rear laser
	cerr << "rear laser-----------------------" << endl;
	cerr << rlaser.num_readings << " ";
	cerr << rlaser.x << " ";
	cerr << rlaser.y << " ";
	cerr << rlaser.theta << " ";
	cerr << rlaser.odom_x << " ";
	cerr << rlaser.odom_y << " ";
	cerr << rlaser.odom_theta << " ";
	cerr << rlaser.timestamp << endl;

	// beams
	cerr << "beams-----------------------" << endl;
	if ( flaser.range != NULL ) {
		for (i=0; i<flaser.num_readings; i++) {
			cerr << flaser.range[i] << endl;
		}
	}

	// beams
	if ( rlaser.range != NULL ) {
		for (i=0; i<rlaser.num_readings; i++) {
			cerr << rlaser.range[i] << endl;;
		}
	}

	// features
	cerr << "features-----------------------" << endl;
	if (list_features != NULL) {
		for (i=0; i<num_features; i++) {
			cerr << list_features[i].name << " " << list_features[i].value << endl;
		}
	}

	return 1; //ok
}


//-----------------------------------------
// Reads laser from a ifstream.
// return  1  ok
//		  -1 error
//-----------------------------------------
int RangeExample::readOneRange(ifstream &ifs) {

	ios_base::iostate old_state = ifs.exceptions(); //save exception state
	
	ifs.exceptions(ios_base::eofbit);

	int error =1;
	
	try {
		int i;
		
		// type is set externally
		ifs >> type;
		
		ifs >> pose.x;
		ifs >> pose.y;
		ifs >> pose.theta;

		flaser.x = pose.x;
		flaser.y = pose.y;
		flaser.theta = pose.theta;
		flaser.odom_x = pose.x;
		flaser.odom_y = pose.y;
		flaser.odom_theta = pose.theta;
		flaser.timestamp = 0.0;
		
		rlaser.x = pose.x;
		rlaser.y = pose.y;
		rlaser.theta = pose.theta;
		rlaser.odom_x = pose.x;
		rlaser.odom_y = pose.y;
		rlaser.odom_theta = pose.theta;
		rlaser.timestamp = 0.0;
			
		//front laser
		ifs >> flaser.num_readings;
	
		//rear laser
		ifs >> rlaser.num_readings;
	
		num_beams = flaser.num_readings + rlaser.num_readings;
	
		// beams
		totalRange = new float[num_beams];
	
		// point to first frontal beam
		flaser.range = &totalRange[0];
	
		for (i=0; i<flaser.num_readings; i++) {
			ifs >> totalRange[i];
		}
	
		// point to first rear beam
		rlaser.range = &totalRange[flaser.num_readings];
	
		for (i=0; i<rlaser.num_readings; i++) {
			ifs >> totalRange[i + flaser.num_readings];
		}
	}
	catch( exception e ) {
		error = -1;
	}
	
	ifs.exceptions(old_state); // reset exception state
	
	return error; //ok
}

//-----------------------------------------
// Sets the ranges of the example
// return  1  ok
//		  -1 error
//-----------------------------------------
int RangeExample::setRanges(int numRanges, double *ranges) {

	int error =1;
	
	try {
	  // Since we cannot obtain the position
	  // of the laser (i.e. robot) from the ranges
	  // set it to <0,0,0>
		
	  pose.x = 0.0;
	  pose.y = 0.0;
	  pose.theta = 0.0;

	  flaser.x = pose.x;
	  flaser.y = pose.y;
	  flaser.theta = pose.theta;
	  flaser.odom_x = pose.x;
	  flaser.odom_y = pose.y;
	  flaser.odom_theta = pose.theta;
	  flaser.timestamp = 0.0;
		
	  rlaser.x = pose.x;
	  rlaser.y = pose.y;
	  rlaser.theta = pose.theta;
	  rlaser.odom_x = pose.x;
	  rlaser.odom_y = pose.y;
	  rlaser.odom_theta = pose.theta;
	  rlaser.timestamp = 0.0;
			
	  //front laser
	  flaser.num_readings = numRanges;
	  
	  //rear laser
	  rlaser.num_readings = 0;
	
	  num_beams = flaser.num_readings + rlaser.num_readings;
	
	  // beams
	  totalRange = new float[num_beams];
	  
	  // point to first frontal beam
	  flaser.range = &totalRange[0];
	  
	  for (int i=0; i < numRanges; i++) {
	    totalRange[i] = ranges[i];
	  }
	}
	catch( exception e ) {
	  error = -1;
	}
	
	return error; //ok
}










//-----------------------------------------
// Each example is a range beam.
// It Includes front laser beams and rear laser beams.
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_RANGE_EXAMPLE__
#define __OSCAR_RANGE_EXAMPLE__


#include "example.h"
#include <fstream>

namespace oscar{

//-----------------------------------------
// class RangeExample
//-----------------------------------------
class RangeExample: public Example {

	// methods
	public:
		RangeExample();
		RangeExample(int n);
		~RangeExample();

		// Reads laser from a ifstream.
		// The source file must be created with the utility "readlogfile"
		// return  1  ok
		//		  -1 error
		int readRange(std::ifstream &ifs);
		
		//second version of readRange
		// the data is read from one file with one example per line with format:
		// label x y theta num_front_readings num_rear_readings front_reading_1 ... front_reading_{num_front_readings} rear_reading_1 ... rear_reading_{num_rear_readings} 
		int readOneRange(std::ifstream &ifs);

		// set the ranges of this RangeExample object using the given ranges
		int setRanges(int numRanges, double *ranges);
		
		// prepare space forn n features
		int setNumFeatures(int n);

		// set filename
		int setFilename(char *f);

		// set one feature
		int setFeature(int n, FeatureValues *f);

		FeatureValues* getFeature(int index);

		int initRange(carmen_robot_laser_message *frontlaser, carmen_robot_laser_message *rearlaser);

		// scale the beams
		void scale(float factor);


		// clean range information
		int clean();

		// debugging purpose
		int debug();

	// data members
	public:
		enum {MAX_BEAMS=362};

		int num_beams; // number of total beams

		carmen_robot_laser_message flaser;
		carmen_robot_laser_message rlaser;

		float *totalRange;

		char *filename;
};

}

#endif // __OSCAR_RANGE_EXAMPLE__








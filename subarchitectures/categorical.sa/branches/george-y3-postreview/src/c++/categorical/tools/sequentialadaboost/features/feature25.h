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

#ifndef __OSCAR_FEATURE25__
#define __OSCAR_FEATURE25__

#include "../rangeExample.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature25: public Feature {

    // functions
    public:
		Feature25();
		~Feature25();

		void polar_to_rect(double r, double theta, double *x, double *y );
		int getFeature(oscar::Example *example, int index_feature);

		void setup(double d);

		double calcDistance(carmen_point_t *p1, carmen_point_t *p2);

		void test() { cerr << "Feature25::test()" << endl; }

		int index1, index2;
		double min1, min2;
		double deviation;
		double first_min;
		double first_min2;

		int first_index;
		int last_index;
		int npoints;

		int first_index2;
		int last_index2;
		int npoints2;

		carmen_point_t point1, point2;

		double distance;
		int beam_distance;

		bool debug;
};


} // end namespace

#endif // __OSCAR_FEATURE25__

//-----------------------------------------
// feature 12:
// Second order spatial moments. Used for other features
// (Computer and robot vision VI, p73)
//
// type = 12
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE12__
#define __OSCAR_FEATURE12__

#include "rangeExample.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature12: public Feature {

    // functions
    public:
		Feature12();
		~Feature12();

		void polar_to_rect(double r, double theta, double *x, double *y );
		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature12::test()" << endl; }

	public:
		double urr, urc, ucc;
		double major_axis, minor_axis;
		double major_angle, minor_angle;

		carmen_point_t major, minor;

		carmen_point_t centroid;
};


} // end namespace

#endif // __OSCAR_FEATURE12__






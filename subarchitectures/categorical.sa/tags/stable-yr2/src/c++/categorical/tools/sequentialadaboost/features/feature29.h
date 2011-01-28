//-----------------------------------------
// feature 29: mean of the distance to the centroid / max distance
// http://astronomy.swin.edu.au/~pbourke/geometry/polyarea/
// type = 29
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE29__
#define __OSCAR_FEATURE29__

#include "../rangeExample.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature29: public Feature {

    // functions
    public:
		Feature29();
		~Feature29();

		void polar_to_rect(double r, double theta, double *x, double *y );

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

		double distance(carmen_point_t *p1, carmen_point_t *p2);

		carmen_point_t centroid;

		bool debug;

};


} // end namespace

#endif // __OSCAR_FEATURE29__

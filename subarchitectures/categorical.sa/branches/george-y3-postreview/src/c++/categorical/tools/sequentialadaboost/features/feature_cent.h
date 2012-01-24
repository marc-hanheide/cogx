//-----------------------------------------
// feature cent: centroid of a polygon and area
//
// type = cent
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE_cent__
#define __OSCAR_FEATURE_cent__

#include "../rangeExample.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature_cent: public Feature {

    // functions
    public:
		Feature_cent();
		~Feature_cent();

		void polar_to_rect(double r, double theta, double *x, double *y );

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

		double distance(carmen_point_t *p1, carmen_point_t *p2);

	public:
		carmen_point_t centroid;
		double area;

};


} // end namespace

#endif // __OSCAR_FEATURE_cent__

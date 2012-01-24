//-----------------------------------------
// feature 6: perimeter
//
// type = 6
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE6__
#define __OSCAR_FEATURE6__

#include "../rangeExample.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature6: public Feature {

    // functions
    public:
		Feature6();
		~Feature6();

		void polar_to_rect(double r, double theta, double *x, double *y );

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

		double distance(carmen_point_t *p1, carmen_point_t *p2);

};


} // end namespace

#endif // __OSCAR_FEATURE6__

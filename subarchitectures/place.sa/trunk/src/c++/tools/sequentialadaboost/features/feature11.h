//-----------------------------------------
// feature 11:
//    extremal points. Calculate the extremal points. Don't use for classification. Use only
//    for calculation new features. (Computer and robot vision VI, p62)
//
// type = 11
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE11__
#define __OSCAR_FEATURE11__

#include "rangeExample.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature11: public Feature {

    // functions
    public:
		Feature11();
		~Feature11();

		void polar_to_rect(double r, double theta, double *x, double *y );		
		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

	public:
		carmen_point_t p1, p2, p3, p4, p5, p6, p7, p8;

};


} // end namespace

#endif // __OSCAR_FEATURE11__






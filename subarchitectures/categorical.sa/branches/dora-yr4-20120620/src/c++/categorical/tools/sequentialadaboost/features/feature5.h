//-----------------------------------------
// feature 5: area of the region covered using cross produt
// (http://geometryalgorithms.com/Archive/algorithm_0101/algorithm_0101.htm#Polygons)
//
// type = 5
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE5__
#define __OSCAR_FEATURE5__

#include "../rangeExample.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature5: public Feature {

    // functions
    public:
		Feature5();
		~Feature5();

		void polar_to_rect(double r, double theta, double *x, double *y );

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

};


} // end namespace

#endif // __OSCAR_FEATURE5__

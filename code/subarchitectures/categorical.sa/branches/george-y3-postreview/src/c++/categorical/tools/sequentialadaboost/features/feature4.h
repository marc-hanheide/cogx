//-----------------------------------------
// feature 4: area of the region covered by lasers with morfology
// type = 4
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE4__
#define __OSCAR_FEATURE4__

#include "rangeExample.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature4: public Feature {

    // functions
    public:
		Feature4();
		~Feature4();

		int getFeature(oscar::Example *example, int index_feature);
		void setup(double resolution);

		void test() { cerr << "Feature4::test()" << endl; }

		int calculate(oscar::Example *example);

	public:
		double resolution;


};


} // end namespace

#endif // __OSCAR_FEATURE4__

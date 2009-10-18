//-----------------------------------------
// feature 1: length relation between beams
// type = 1
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE1__
#define __OSCAR_FEATURE1__

#include "rangeExample.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature1: public Feature {

    // functions
    public:
		Feature1();
		Feature1(int beam1, int beam2);
		~Feature1();

		int setup(int beam1, int beam2);
		int getFeature(oscar::Example *example, int index_feature);

		// data members
		int beam1, beam2;

		void test() { cerr << "Feature1::test()" << endl; }

};


} // end namespace

#endif // __OSCAR_FEATURE1__

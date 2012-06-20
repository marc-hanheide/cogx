//-----------------------------------------
// feature 31:  perimeter**2 / area
//
// type = 31
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE31__
#define __OSCAR_FEATURE31__

#include "../rangeExample.h"
#include "feature5.h"
#include "feature6.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature31: public Feature {

    // functions
    public:
		Feature31();
		~Feature31();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

};


} // end namespace

#endif // __OSCAR_FEATURE31__

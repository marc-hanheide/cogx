//-----------------------------------------
// feature 7:  perimeter/area
//
// type = 7
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE7__
#define __OSCAR_FEATURE7__

#include "../rangeExample.h"
#include "feature5.h"
#include "feature6.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature7: public Feature {

    // functions
    public:
		Feature7();
		~Feature7();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

};


} // end namespace

#endif // __OSCAR_FEATURE7__

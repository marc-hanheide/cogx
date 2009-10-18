//-----------------------------------------
// feature 24:  4pi*area/sqr(perimeter)
//
// type = 24
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE24__
#define __OSCAR_FEATURE24__

#include "rangeExample.h"
#include "feature5.h"
#include "feature6.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature24: public Feature {

    // functions
    public:
		Feature24();
		~Feature24();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

};


} // end namespace

#endif // __OSCAR_FEATURE24__

//-----------------------------------------
// feature 32:  Kurtosis
//
// type = 32
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE32__
#define __OSCAR_FEATURE32__

#include "../rangeExample.h"
#include "feature5.h"
#include "feature6.h"
#include <iostream>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature32: public Feature {

    // functions
    public:
		Feature32();
		~Feature32();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature4::test()" << endl; }

};


} // end namespace

#endif // __OSCAR_FEATURE32__

//-----------------------------------------------------------
// feature 8:
//		average of length of beams
//
// type 8
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE8__
#define __OSCAR_FEATURE8__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature8: public Feature {

    // functions
    public:
		Feature8();
		~Feature8();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE8__











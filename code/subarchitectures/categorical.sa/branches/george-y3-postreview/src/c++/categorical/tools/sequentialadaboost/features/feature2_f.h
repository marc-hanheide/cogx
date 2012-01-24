//-----------------------------------------------------------
// feature 2_f:
//		average of the difference between consecutive beams. Only front laser.
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE2_F__
#define __OSCAR_FEATURE2_F__

#include "rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature2_f: public Feature {

    // functions
    public:
		Feature2_f();
		~Feature2_f();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2_f::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE2_F__

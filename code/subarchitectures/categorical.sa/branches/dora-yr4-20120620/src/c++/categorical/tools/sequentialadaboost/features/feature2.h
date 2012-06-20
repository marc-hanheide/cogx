//-----------------------------------------------------------
// feature 2:
//		average of the difference between consecutive beams
// type 2
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE2__
#define __OSCAR_FEATURE2__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature2: public Feature {

    // functions
    public:
		Feature2();
		~Feature2();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE2__

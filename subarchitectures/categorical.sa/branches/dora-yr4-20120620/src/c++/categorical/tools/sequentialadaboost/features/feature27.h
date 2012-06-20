//-----------------------------------------------------------
// feature 2:
//		average of the difference between consecutive beams
// type 2
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE27__
#define __OSCAR_FEATURE27__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature27: public Feature {

    // functions
    public:
		Feature27();
		~Feature27();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE27__

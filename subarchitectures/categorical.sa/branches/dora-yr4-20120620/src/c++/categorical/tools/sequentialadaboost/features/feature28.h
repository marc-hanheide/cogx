//-----------------------------------------------------------
// feature 28:
//		average of length of beams / max_length
//
// type 28
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE28__
#define __OSCAR_FEATURE28__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature28: public Feature {

    // functions
    public:
		Feature28();
		~Feature28();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE28__











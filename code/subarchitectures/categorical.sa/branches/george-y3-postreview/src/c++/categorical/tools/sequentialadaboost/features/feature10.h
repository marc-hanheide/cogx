//-----------------------------------------------------------
// feature 10:
// number N of sides to a regular digital polygon (Computer and robot vision VI, p61)
//
// type 10
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE10__
#define __OSCAR_FEATURE10__

#include "rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature10: public Feature {

    // functions
    public:
		Feature10();
		~Feature10();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE10__

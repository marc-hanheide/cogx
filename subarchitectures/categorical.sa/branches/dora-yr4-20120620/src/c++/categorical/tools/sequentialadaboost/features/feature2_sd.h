//-----------------------------------------------------------
// feature 2_sd:
//		standard deviation  of the difference between consecutive beams
// type 2_sd
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE2_SD__
#define __OSCAR_FEATURE2_SD__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature2_sd: public Feature {

    // functions
    public:
		Feature2_sd();
		~Feature2_sd();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE2_SD__

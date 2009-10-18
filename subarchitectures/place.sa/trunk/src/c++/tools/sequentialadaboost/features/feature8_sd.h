//-----------------------------------------------------------
// feature 8_sd:
//		standard deviation  of the beams length
// type 8_sd
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE8_SD__
#define __OSCAR_FEATURE8_SD__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature8_sd: public Feature {

    // functions
    public:
		Feature8_sd();
		~Feature8_sd();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE8_SD__

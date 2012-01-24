//-----------------------------------------------------------
// feature 28_sd:
//		standard deviation  of the beams length / maximum length
// type 28_sd
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE28_SD__
#define __OSCAR_FEATURE28_SD__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature28_sd: public Feature {

    // functions
    public:
		Feature28_sd();
		~Feature28_sd();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE28_SD__

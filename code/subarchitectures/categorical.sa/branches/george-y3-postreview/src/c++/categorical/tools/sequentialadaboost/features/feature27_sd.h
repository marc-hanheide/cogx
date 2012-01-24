//-----------------------------------------------------------
// feature 27_sd:
//		standard deviation  of the relation between consecutive beams: b_i/b_{i+1}
// type 27_sd
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE27_SD__
#define __OSCAR_FEATURE27_SD__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature27_sd: public Feature {

    // functions
    public:
		Feature27_sd();
		~Feature27_sd();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE27_SD__






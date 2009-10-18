//-----------------------------------------------------------
// feature 9_m_sd:
//		mean of of the distances from end of beams to centroid divided by
//		standard deviation
// type 9_m_sd
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE9_M_SD__
#define __OSCAR_FEATURE9_M_SD__

#include "rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature9_m_sd: public Feature {

    // functions
    public:
		Feature9_m_sd();
		~Feature9_m_sd();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE9_SD__

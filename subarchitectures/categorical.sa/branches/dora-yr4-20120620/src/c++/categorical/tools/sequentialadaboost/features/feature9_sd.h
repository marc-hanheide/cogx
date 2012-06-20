//-----------------------------------------------------------
// feature 9_sd:
//		standard deviation  of the distances from end of beams to centroid
// type 9_sd
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE9_SD__
#define __OSCAR_FEATURE9_SD__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature9_sd: public Feature {

    // functions
    public:
		Feature9_sd();
		~Feature9_sd();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }

		void polar_to_rect(double r, double theta, double *x, double *y );

		double distance(carmen_point_t *p1, carmen_point_t *p2);		
};


} // end namespace

#endif // __OSCAR_FEATURE9_SD__

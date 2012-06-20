//-----------------------------------------------------------
// feature 29_sd:
//		standard deviation  of the distances from end of beams to centroid / max distance
// type 29_sd
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE29_SD__
#define __OSCAR_FEATURE29_SD__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature29_sd: public Feature {

    // functions
    public:
		Feature29_sd();
		~Feature29_sd();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature2::test()" << endl; }

		void polar_to_rect(double r, double theta, double *x, double *y );

		double distance(carmen_point_t *p1, carmen_point_t *p2);		
};


} // end namespace

#endif // __OSCAR_FEATURE29_SD__

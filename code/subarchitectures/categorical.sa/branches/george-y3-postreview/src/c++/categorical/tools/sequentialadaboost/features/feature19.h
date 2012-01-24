//-----------------------------------------
// feature 19:
//
// major_axis_len / minor_axis_len  of the ellipse using 1 and -1 fourier descriptors
//
// type = 19
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE19__
#define __OSCAR_FEATURE19__

#include "../rangeExample.h"
#include "feature13.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature19: public Feature {

    // functions
    public:
		Feature19();
		~Feature19();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature19::test()" << endl; }

		void setup(oscar::Feature13 *f);

	public:
		oscar::Feature13 *f13;
};


} // end namespace

#endif // __OSCAR_FEATURE19__

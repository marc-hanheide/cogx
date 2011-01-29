//-----------------------------------------
// feature 18_2:
//
// minor axis length of the ellipse using 1 and -1 fourier descriptors
//
// type = 18_2
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE18_2__
#define __OSCAR_FEATURE18_2__

#include "../rangeExample.h"
#include "feature13.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature18_2: public Feature {

    // functions
    public:
		Feature18_2();
		~Feature18_2();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature18_2::test()" << endl; }

		void setup(oscar::Feature13 *f);

	public:
		oscar::Feature13 *f13;

};


} // end namespace

#endif // __OSCAR_FEATURE18_2__

//-----------------------------------------
// feature 18:
//
// minor axis length of the ellipse using 1 and -1 fourier descriptors
//
// type = 18
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE18__
#define __OSCAR_FEATURE18__

#include "../rangeExample.h"
#include "feature13.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature18: public Feature {

    // functions
    public:
		Feature18();
		~Feature18();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature18::test()" << endl; }

		void setup(oscar::Feature13 *f);

	public:
		oscar::Feature13 *f13;

};


} // end namespace

#endif // __OSCAR_FEATURE18__

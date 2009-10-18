//-----------------------------------------
// feature 16:
//
// one fourier descriptor
//
// type = 16
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE16__
#define __OSCAR_FEATURE16__

#include "../rangeExample.h"
#include "feature13.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature16: public Feature {

    // functions
    public:
		Feature16();
		~Feature16();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature16::test()" << endl; }

		void setup(int index, oscar::Feature13 *f);

	public:
		int index;
		oscar::Feature13 *f13;
};


} // end namespace

#endif // __OSCAR_FEATURE16__

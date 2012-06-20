//-----------------------------------------
// feature 17_2:
//
// major_axis length of the ellipse using 1 and -1 fourier descriptors
//
// type = 17_2
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE17_2__
#define __OSCAR_FEATURE17_2__

#include "../rangeExample.h"
#include "feature13.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature17_2: public Feature {

    // functions
    public:
		Feature17_2();
		~Feature17_2();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature17_2::test()" << endl; }

		void setup(oscar::Feature13 *f);

	public:
	
		oscar::Feature13 *f13;

};


} // end namespace

#endif // __OSCAR_FEATURE17_2__

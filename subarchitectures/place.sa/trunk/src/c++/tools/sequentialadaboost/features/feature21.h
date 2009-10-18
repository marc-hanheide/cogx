//-----------------------------------------
// feature 21:
//
// invariants. Digital Image processing, gonzalez, p516
//
// type = 21
//
// oscar martinez
//-----------------------------------------


#ifndef __OSCAR_FEATURE21__
#define __OSCAR_FEATURE21__

#include "../rangeExample.h"
#include "feature20.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature21: public Feature {

    // functions
    public:
		Feature21();
		~Feature21();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature21:test()" << endl; }

		void setup(int i, oscar::Feature20 *f);

	public:
		int invariant;
		oscar::Feature20 *f20;
};


} // end namespace

#endif // __OSCAR_FEATURE21__

















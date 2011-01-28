//-----------------------------------------
// feature 22:
//
// M_cmp (slides from Dmitrij)
//
// type = 22
//
// oscar martinez
//-----------------------------------------


#ifndef __OSCAR_FEATURE22__
#define __OSCAR_FEATURE22__

#include "../rangeExample.h"
#include "feature20.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature22: public Feature {

    // functions
    public:
		Feature22();
		~Feature22();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature22:test()" << endl; }

		void setup(oscar::Feature20 *f);

	public:
		oscar::Feature20 *f20;
};


} // end namespace

#endif // __OSCAR_FEATURE22__

















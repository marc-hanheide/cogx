//-----------------------------------------
// feature 23:
//
// M_ect (slides from Dmitrij)
//
// type = 23
//
// oscar martinez
//-----------------------------------------


#ifndef __OSCAR_FEATURE23__
#define __OSCAR_FEATURE23__

#include "../rangeExample.h"
#include "feature20.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature23: public Feature {

    // functions
    public:
		Feature23();
		~Feature23();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature23:test()" << endl; }

		void setup(oscar::Feature20 *f);

	public:
		oscar::Feature20 *f20;
};


} // end namespace

#endif // __OSCAR_FEATURE23__

















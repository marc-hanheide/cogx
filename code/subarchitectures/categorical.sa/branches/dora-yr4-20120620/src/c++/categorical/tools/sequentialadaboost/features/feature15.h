//-----------------------------------------
// feature 15:
//
// number of points that lie in lines extracted from scans.
// Lines have a maximum length of max_lengthe
//
// type = 15
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE15__
#define __OSCAR_FEATURE15__


extern "C" {
#include <carmen/linemapping.h>
}


#include "feature.h"
#include "rangeExample.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature15: public Feature {

    // functions
    public:
		Feature15();
		~Feature15();

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature12::test()" << endl; }

};


} // end namespace

#endif // __OSCAR_FEATURE15__






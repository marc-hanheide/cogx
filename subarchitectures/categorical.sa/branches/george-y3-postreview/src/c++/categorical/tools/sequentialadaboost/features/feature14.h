//-----------------------------------------
// feature 14:
//
// number of gaps. Difference between two consecutive beams
//
// type = 14
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE14__
#define __OSCAR_FEATURE14__

#include "../rangeExample.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature14: public Feature {

    // functions
    public:
		Feature14();
		~Feature14();

		void polar_to_rect(double r, double theta, double *x, double *y );
		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature12::test()" << endl; }

		void setup(double t);

		double threshold;
};


} // end namespace

#endif // __OSCAR_FEATURE14__

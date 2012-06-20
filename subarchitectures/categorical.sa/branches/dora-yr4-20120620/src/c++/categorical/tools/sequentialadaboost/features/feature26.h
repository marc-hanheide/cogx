//-----------------------------------------
// feature 26:
//
// Look for the two local minima in beams legth with minimum value.
// Beam distance between these two minima.
//
// type = 26
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE26__
#define __OSCAR_FEATURE26__

#include "../rangeExample.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature25.h"

namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature26: public Feature {

    // functions
    public:
		Feature26();
		~Feature26();

		void polar_to_rect(double r, double theta, double *x, double *y );
		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature26::test()" << endl; }

		void setup(double d);

		double deviation;

		Feature25 f25;
};


} // end namespace

#endif // __OSCAR_FEATURE26__

//-----------------------------------------
// feature 30:
//
// number of relative gaps.
// One gap:
//		b_i / b_{i+1} > threshold
//
// type = 30
//
// oscar martinez
//-----------------------------------------


#ifndef __OSCAR_FEATURE30__
#define __OSCAR_FEATURE30__

#include "../rangeExample.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar {

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature30: public Feature {

    // functions
    public:
		Feature30();
		~Feature30();

		void polar_to_rect(double r, double theta, double *x, double *y );
		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature12::test()" << endl; }

		void setup(double t);

		double threshold;
};


} // end namespace

#endif // __OSCAR_FEATURE30__

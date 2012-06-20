//-----------------------------------------
// feature 20:
//
// moments and invariants. Digital Image processing, gonzalez, p514-515
// invariants to translation, rotation and scale
//
// type = 20
//
// oscar martinez
//-----------------------------------------


#ifndef __OSCAR_FEATURE20__
#define __OSCAR_FEATURE20__

#include "../rangeExample.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature20: public Feature {

    // functions
    public:
		Feature20();
		~Feature20();

		void polar_to_rect(double r, double theta, double *x, double *y );

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature20:test()" << endl; }

		// reconstruction of figure with coefficients

	public:
		// moments
		double ux, uy;
		double u00, u11, u20, u02, u30, u12, u21, u03;
		double n_u00, n_u11, n_u20, n_u02, n_u30, n_u12, n_u21, n_u03;
		double inv[7];
};


} // end namespace

#endif // __OSCAR_FEATURE20__






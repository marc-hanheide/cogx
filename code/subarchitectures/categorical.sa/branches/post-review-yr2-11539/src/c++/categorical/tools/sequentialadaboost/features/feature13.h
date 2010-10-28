//-----------------------------------------
// feature 13:
//
// fourier coefficients
//
// type = 13
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE13__
#define __OSCAR_FEATURE13__

#include "../rangeExample.h"
#include <iostream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature13: public Feature {

    // functions
    public:
		Feature13();
		~Feature13();

		void polar_to_rect(double r, double theta, double *x, double *y );
		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature12::test()" << endl; }

		void setup(int n);

		// reconstruction of figure with coefficients
		void reconstruction(int n_points, int n_coefficients);

	public:

		gsl_complex *coefficients;
		gsl_complex *descriptors;
		int n_coefficients;
		double perimeter;
		carmen_point_t major_axis, minor_axis;
		double major_axis_len, minor_axis_len;
		double major_axis_angle, minor_axis_angle;

		carmen_point_t major_axis_d, minor_axis_d;
		double major_axis_len_d, minor_axis_len_d;
		double major_axis_angle_d, minor_axis_angle_d;
};


} // end namespace

#endif // __OSCAR_FEATURE13__






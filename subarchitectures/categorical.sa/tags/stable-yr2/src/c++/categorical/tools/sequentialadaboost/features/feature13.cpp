//-----------------------------------------
// feature 13:
//
// fourier descriptors. Mustererkennung
//
// type = 13
//
// oscar martinez
//-----------------------------------------


#include <math.h>
#include <iostream>
#include <fstream>

#include <gsl/gsl_complex.h>
#include <gsl/gsl_complex_math.h>

#include "feature13.h"

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature13::Feature13() {
	setName("fourier.");
	setType("f13");
	n_coefficients = 1;
	coefficients = new gsl_complex[n_coefficients*2 +1];
	descriptors = new gsl_complex[n_coefficients*2 +1];
}


//-----------------------------------------
// destructor
//-----------------------------------------
Feature13::~Feature13() {
	if ( coefficients != NULL ){
		delete [] coefficients;
		coefficients = NULL;
		n_coefficients = 0;
	}
	if ( descriptors != NULL ){
		delete [] descriptors;
		descriptors = NULL;
	}
}

//-----------------------------------------
// number of fourier coefficients
//-----------------------------------------
void Feature13::setup(int n) {
	if ( coefficients != NULL || n_coefficients != n){
		delete [] coefficients;
		coefficients = NULL;
	}
	if ( descriptors != NULL || n_coefficients != n ){
		delete [] descriptors;
		descriptors = NULL;
	}
	n_coefficients = n;
	coefficients = new gsl_complex[n_coefficients*2 +1];
	descriptors = new gsl_complex[n_coefficients*2 +1];
}

//-----------------------------------------
//
//-----------------------------------------
void Feature13::polar_to_rect(double r, double theta, double *x, double *y ) {
    *x = r*cos(theta);
    *y = r*sin(theta);
    return;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature13::getFeature(oscar::Example *e, int index_feature) {

	//cerr << "Feature13::getFeature()" << endl;

	oscar::RangeExample *re = (oscar::RangeExample *)e;

	int n = (re->flaser).num_readings + (re->rlaser).num_readings;
	gsl_complex *V = new gsl_complex[n];

	double flaser_resolution = M_PI / (double)( (re->flaser).num_readings -1 );
	double rlaser_resolution = M_PI / (double)( (re->rlaser).num_readings -1 );
	double theta;
	carmen_point_t point;

 	//cerr << "---------------------------" << endl;
	
	// store the points in an array
	int index = 0;
	for (int j=0; j<(re->flaser).num_readings; j++) {
		theta = (re->flaser).theta -M_PI_2 + j*flaser_resolution;
		polar_to_rect( (re->flaser).range[j], theta, &point.x, &point.y );
		V[index] = gsl_complex_rect (point.x, point.y);
		//cerr << point.x << " " << point.y << endl;
		index++;
	}

	for (int j=0; j<(re->rlaser).num_readings; j++) {
		theta = (re->rlaser).theta -M_PI_2 + j*rlaser_resolution;
		polar_to_rect( (re->rlaser).range[j], theta, &point.x, &point.y);
		V[index] = gsl_complex_rect (point.x, point.y);
		//cerr << point.x << " " << point.y << endl;
		index++;
	}

	
	// perimeter
	// problems with division by 0
	perimeter = 0.000000000000000000000001;
	gsl_complex diff;
	for(int i=0; i<n; i++) {
		int j = (i+1)%n;
		diff = gsl_complex_sub(V[j], V[i]);
		perimeter += gsl_complex_abs(diff);
	}
	//cerr << "perimeter: " << perimeter << endl;


	// differences
	gsl_complex *diff_x = new gsl_complex[n];
	double *diff_x_abs = new double[n];
	for(int i=0; i<n; i++) {
		int j = (i+1)%n;
		diff_x[i] = gsl_complex_sub(V[j], V[i]);
		diff_x_abs[i] = gsl_complex_abs(diff_x[i]);
		// problems with 0 values
		diff_x_abs[i] = diff_x_abs[i] + 0.000000000000000000000001;
	}

	// normalized differences
	//cerr << "z_d----------" << endl;
	gsl_complex *diff_z = new gsl_complex[n];
	for(int i=0; i<n; i++) {
		diff_z[i] = gsl_complex_div_real( diff_x[i], diff_x_abs[i] );
		//cerr << GSL_REAL(diff_z[i]) << " + " << GSL_IMAG(diff_z[i]) << "i" << endl;
	}
	//cerr << "z_d----end------" << endl;


	// values of tk
	//cout << "tk----------" << endl;
	double *tk = new double[n];
	tk[0] = 0;

	for(int k=1; k<n; k++) {
	  double acc = 0;
	  for(int i=0; i<k; i++) {
	    acc += gsl_complex_abs( diff_x[i] );
	  }
	  tk[k] = acc;
	  //cout << tk[k] << endl;
	}

	// linienschwerpunkte
	gsl_complex a, b, c, d;
	d = gsl_complex_rect(0, 0);
	for (int k=0; k<n; k++) {
		int l= (k+1)%n;
		a = gsl_complex_add( V[k], V[l]);
		b = gsl_complex_mul_real( a, diff_x_abs[k] );
		d = gsl_complex_add( d, b);
	}
	coefficients[n_coefficients] = gsl_complex_div_real(d, 2.0*perimeter);

	//cout << "coeff[0]:" << GSL_REAL(coefficients[n_coefficients]) << "+"
	//	 << GSL_IMAG(coefficients[n_coefficients]) << "i" << endl;

	// coefficients
	for (int num=-n_coefficients; num<=n_coefficients; num++) {
		if ( num != 0 ) {
			index = num + n_coefficients;
			d = gsl_complex_rect(0, 0);
			for (int k=0; k<n; k++) {
				int l;
				if ( k==0) {
					l = n-1;
				}
				else {
					l = k-1;
				}
				a = gsl_complex_sub( diff_z[l], diff_z[k] );
				double e = - num * 2.0 * M_PI * tk[k] / perimeter;
				b = gsl_complex_polar(1, e);
				c = gsl_complex_mul(a, b);
				d = gsl_complex_add(d, c);
			}
			double factor;
			factor = perimeter / ((2.0 * M_PI * num)*(2.0 * M_PI * num));
			coefficients[index] = gsl_complex_mul_real(d, factor);
			//cout << "coeff[" << num << "]: " << GSL_REAL(coefficients[index]) << " + "
			// 	 << GSL_IMAG(coefficients[index]) << "i" << endl;
		}
	}


	//descriptors
	double arg, arg1, arg2;
	double abs1;
	double abs;
	double factor;
	double ex;
	arg1 = gsl_complex_arg( coefficients[n_coefficients+1] );
	arg2 = gsl_complex_arg( coefficients[n_coefficients+2] );
	abs1 = gsl_complex_abs(coefficients[n_coefficients+1]);

	for(int i=-n_coefficients; i<=n_coefficients; i++) {
		index = i + n_coefficients;
		arg = gsl_complex_arg( coefficients[index] );
		ex = arg + (1.0 -(double)i )*arg2 - (2.0-(double)i)*arg1;
		a = gsl_complex_polar(1.0, ex);
		abs = gsl_complex_abs(coefficients[index]);
		factor = abs / abs1;
		descriptors[index] = gsl_complex_mul_real(a, factor);
	}

	//cerr << "---------------------------" << endl;
	
	// axis of the ellipse with coefficients 1 and -1
	double a_abs = gsl_complex_abs( coefficients[n_coefficients+1] );
	double b_abs = gsl_complex_abs( coefficients[n_coefficients-1] );
	major_axis_len = a_abs + b_abs;
	minor_axis_len = fabs(a_abs - b_abs);

	double a_arg, b_arg;
	a_arg = gsl_complex_arg( coefficients[n_coefficients+1] );
	b_arg = gsl_complex_arg( coefficients[n_coefficients-1] );

	major_axis_angle = (a_arg + b_arg) / 2.0;
	minor_axis_angle = major_axis_angle + M_PI_2;

	polar_to_rect(major_axis_len, major_axis_angle, &major_axis.x, &major_axis.y);
	polar_to_rect(minor_axis_len, minor_axis_angle, &minor_axis.x, &minor_axis.y);

	major_axis.x += GSL_REAL(coefficients[n_coefficients]);
	major_axis.y += GSL_IMAG(coefficients[n_coefficients]);

	minor_axis.x += GSL_REAL(coefficients[n_coefficients]);
	minor_axis.y += GSL_IMAG(coefficients[n_coefficients]);


	// axis of the ellipse with descriptors 1 and -1
	a_abs = gsl_complex_abs( descriptors[n_coefficients+1] );
	b_abs = gsl_complex_abs( descriptors[n_coefficients-1] );
	major_axis_len_d = a_abs + b_abs;
	minor_axis_len_d = fabs(a_abs - b_abs);

	a_arg = gsl_complex_arg( descriptors[n_coefficients+1] );
	b_arg = gsl_complex_arg( descriptors[n_coefficients-1] );

	major_axis_angle_d = (a_arg + b_arg) / 2.0;
	minor_axis_angle_d = major_axis_angle_d + M_PI_2;

	polar_to_rect(major_axis_len_d, major_axis_angle_d, &major_axis_d.x, &major_axis_d.y);
	polar_to_rect(minor_axis_len_d, minor_axis_angle_d, &minor_axis_d.x, &minor_axis_d.y);

	// don not need. Centred at the origin
	//major_axis_d.x += GSL_REAL(descriptors[n_coefficients]);
	//major_axis_d.y += GSL_IMAG(descriptors[n_coefficients]);

	//minor_axis_d.x += GSL_REAL(descriptors[n_coefficients]);
	//minor_axis_d.y += GSL_IMAG(descriptors[n_coefficients]);

	delete [] V;
	delete [] diff_x;
	delete [] diff_x_abs;
	delete [] diff_z;
	delete [] tk;

	return 1;
}


//-----------------------------------------
// reconstruction of figure with coefficients
//-----------------------------------------
void Feature13::reconstruction(int n_p, int n_c) {

	double step = perimeter / (double)n_p;
	ofstream ofs("re_coefficients.txt");
	ofstream ofs2("re_descriptors.txt");
	

	int index;
	gsl_complex a, b, x;


	// reconstruction with coefficients
	for (double t=0.0; t<= perimeter; t += step) {
		x = gsl_complex_rect(0.0, 0.0);
		for ( int n=-n_c; n<=n_c; n++) {
			index = n + n_c;
			double e = n * 2.0 * M_PI * t / perimeter;
			b = gsl_complex_polar(1.0, e);
			a = gsl_complex_mul(coefficients[index], b);
			x = gsl_complex_add(x, a);
		}
		ofs << x.dat[0] << " " << x.dat[1] << endl;
	}


	// reconstruction with descriptors
	for (double t=0.0; t<= perimeter; t += step) {
		x = gsl_complex_rect(0.0, 0.0);
		for ( int n=-n_c; n<=n_c; n++) {
			index = n + n_c;
			double e = n * 2.0 * M_PI * t / perimeter;
			b = gsl_complex_polar(1.0, e);
			a = gsl_complex_mul(descriptors[index], b);
			x = gsl_complex_add(x, a);
		}
		ofs2 << x.dat[0] << " " << x.dat[1] << endl;
	}
	

	ofs.close();
	ofs2.close();
}



































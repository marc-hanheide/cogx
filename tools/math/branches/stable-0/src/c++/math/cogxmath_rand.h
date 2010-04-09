/**
 * Some statistics functions.
 *
 * @author Michael Zillich
 */

#ifndef COGXMATH_RAND_H
#define COGXMATH_RAND_H

/**
 * Univariate Normal Probability Density Function.
 */
extern double normpdf(double x, double mu, double sig);

/**
 * Univariate Normal Cumulative Distribution Function.
 */
extern double normcdf(double x, double mu, double sig);

/**
 * Bivariate Normal Probability Density Function.
 */
extern double mvnpdf2(double x1, double x2, double mu1, double mu2,
  double sig11, double sig12, double sig22);

extern double normrand(double mu, double sig);

extern double unifrand(double a, double b);

#endif

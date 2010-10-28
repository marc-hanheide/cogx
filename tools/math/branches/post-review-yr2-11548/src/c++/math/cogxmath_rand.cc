/**
 * Some statistics functions.
 *
 * @author Michael Zillich
 */

#include <cmath>
#include "cogxmath_rand.h"

static long idum = 7;

/* (C) Copr. 1986-92 Numerical Recipes Software VP. */
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1+(IM-1)/NTAB)
#define EPS 1.2e-7
#define RNMX (1.0-EPS)

static float ran1(long *idum)
{
	int j;
	long k;
	static long iy=0;
	static long iv[NTAB];
	float temp;

	if (*idum <= 0 || !iy) {
		if (-(*idum) < 1) *idum=1;
		else *idum = -(*idum);
		for (j=NTAB+7;j>=0;j--) {
			k=(*idum)/IQ;
			*idum=IA*(*idum-k*IQ)-IR*k;
			if (*idum < 0) *idum += IM;
			if (j < NTAB) iv[j] = *idum;
		}
		iy=iv[0];
	}
	k=(*idum)/IQ;
	*idum=IA*(*idum-k*IQ)-IR*k;
	if (*idum < 0) *idum += IM;
	j=iy/NDIV;
	iy=iv[j];
	iv[j] = *idum;
	if ((temp=AM*iy) > RNMX) return RNMX;
	else return temp;
}
#undef IA
#undef IM
#undef AM
#undef IQ
#undef IR
#undef NTAB
#undef NDIV
#undef EPS
#undef RNMX

/* (C) Copr. 1986-92 Numerical Recipes Software VP. */
static float gasdev(long *idum)
{
	static int iset=0;
	static float gset;
	float fac,rsq,v1,v2;

	if  (iset == 0) {
		do {
			v1=2.0*ran1(idum)-1.0;
			v2=2.0*ran1(idum)-1.0;
			rsq=v1*v1+v2*v2;
		} while (rsq >= 1.0 || rsq == 0.0);
		fac=sqrt(-2.0*log(rsq)/rsq);
		gset=v1*fac;
		iset=1;
		return v2*fac;
	} else {
		iset=0;
		return gset;
	}
}

double mvnpdf2(double x1, double x2, double mu1, double mu2,
  double sig11, double sig12, double sig22)
{
  double d1 = x1 - mu1;
  double d2 = x2 - mu2;
  double det = sig11*sig22 - sig12*sig12;
  return exp(-(d1*(d1*sig11 + d2*sig12) + d2*(d1*sig12 + d2*sig22))/(2*det)) / (2*M_PI*sqrt(det));
}

double normpdf(double x, double mu, double sig)
{
  return exp(-(x - mu)*(x - mu)/(2*sig*sig))/(sig*sqrt(2*M_PI));
}

double normcdf(double x, double mu, double sig)
{
  return 0.5*(1. + erf((x - mu)/(sig*M_SQRT2)));
}

double normrand(double mu, double sig)
{
  return sig*(double)gasdev(&idum) + mu;
}

double unifrand(double a, double b)
{
  return a + (b - a)*ran1(&idum);
}



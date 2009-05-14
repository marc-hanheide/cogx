/*********************************************************************/
/*                                                                   */
/* FILE         gamma.cc                                             */
/* AUTHORS      Bastian Leibe                                        */
/* EMAIL        leibe@inf.ethz.ch                                    */
/*                                                                   */
/* CONTENT      A set of auxiliary routines to calculate the incom-  */
/*              plete gamma function used in the chi-square signifi- */
/*              cance estimate. The algorithms and code are taken    */
/*              from the book "Numerical Recipes in C", 2nd edition. */
/*                                                                   */
/* BEGIN        Fri Sep 07 2001                                      */
/* LAST CHANGE  Fri Sep 07 2001                                      */
/*                                                                   */
/*********************************************************************/


/****************/
/*   Includes   */
/****************/
#include <math.h>
#include <stdlib.h>        // for exit()
#include <iostream>

#include <libChiSquare/gamma.hh>

using namespace std;

/***********************/
/* Function Prototypes */
/***********************/
double gammln(double xx);
void   gser(double *gamser, double a, double x, double *gln);
void   gcf(double *gammcf, double a, double x, double *gln);
void   nrerror(char error_text[]);


double gammq(double a, double x)
  /***************************************************************/
  /* Returns the incomplete gamma function Q(a,x) = 1 - P(a,x).  */
  /*                                                             */
  /* Routine taken from the book "Numerical Recipes in C",       */
  /* 2nd ed., p. 219.                                            */
  /***************************************************************/
{
  double gamser, gammcf, gln;
  
  if ((x < 0.0) || (a <= 0.0)) {
    nrerror("Invalid arguments in routine gammq");
    exit(1);
  }
  if (x < (a+1.0)) {            /* use the series representation */
    gser(&gamser, a, x, &gln);
    return (1.0 - gamser);      /* and take its complement.      */
    
  } else {        /* Use the continuous fraction representation. */
    gcf(&gammcf, a, x, &gln);
    return gammcf;
  }
}


void gser(double *gamser, double a, double x, double *gln)
  /***************************************************************/
  /* Returns the incomplete gamma function P(a,x), evaluated by  */
  /* its series representation as gamser.                        */
  /* Also returns ln(gamma(a)) as gln.                           */
  /*                                                             */
  /* Routine taken from the book "Numerical Recipes in C",       */
  /* 2nd ed., pp. 219-220.                                       */
  /***************************************************************/
{
  int    n;
  double sum, del, ap;
  
  *gln = gammln(a);
  if (x <= 0.0) {
    if (x < 0.0) {
      nrerror("x less than 0 in routine gser");
      exit(1);
    }
    *gamser = 0.0;
    return;
    
  } else {
    ap  = a;
    del = sum = 1.0/a;
    for (n=1; n<=ITMAX; n++) {
      ++ap;
      del *= x/ap;
      sum += del;
      if (fabs(del) < fabs(sum)*EPS) {
	*gamser = sum*exp(-x + a*log(x) - (*gln));
	return;
      }
    }
    nrerror("a too large, ITMAX too small in routine gser");
    return;
  }
}


void gcf(double *gammcf, double a, double x, double *gln)
  /***************************************************************/
  /* Returns the incomplete gamma function Q(a,x), evaluated by  */
  /* its continued fraction representation as gammcf.            */
  /* Also returns ln(gamma(a)) as gln.                           */
  /*                                                             */
  /* Routine taken from the book "Numerical Recipes in C",       */
  /* 2nd ed., p. 220.                                            */
  /***************************************************************/
{
  int    i;
  double an, b, c, d, del, h;
  
  *gln = gammln(a);
  b = x + 1.0-a;    /* Set up for evaluating continuing fraction */
  c = 1.0/FPMIN;    /* by modified Lentz's method (chapt. 5.2 in */
  d = 1.0/b;        /* the book) with b0 = 0.                    */
  h = d;
  for (i=1; i<=ITMAX; i++) {          /* Iterate to convergence. */
    an = -i*(i-a);
    b += 2.0;
    d  = an*d + b;
    if (fabs(d) < FPMIN) d = FPMIN;
    c  = b + an/c;
    if (fabs(c) < FPMIN) c = FPMIN;
    d  = 1.0/d;
    del= d*c;
    h *= del;
    if (fabs(del - 1.0) < EPS) break;
  }
  if (i > ITMAX) {
    nrerror("a too large, ITMAX too small in gcf");
    exit(1);
  }
  *gammcf = exp(-x + a*log(x) - (*gln))*h;  /* Put factors in front. */
}


double gammln(double xx)
  /***************************************************************/
  /* Returns the value ln(gamma(xx)) for xx > 0.                 */
  /*                                                             */
  /* Routine taken from the book "Numerical Recipes in C",       */
  /* 2nd ed., p. 214.                                            */
  /***************************************************************/
{
  /* Internal accuracy will be done in double precision, a nice- */
  /* ty that you can omit if five-figure accuracy is good enough.*/
  double x, y, tmp, ser;
  static double cof[6] = {  76.18009172947146,
			   -86.50532032941677,
			    24.01409824083091,
			    -1.231739572450155,
			     0.1208650973866179e-2,
			    -0.5395239384953e-5 };
  int    j;
  
  y    = x = xx;
  tmp  = x + 5.5;
  tmp -= (x+0.5)*log(tmp);
  ser  = 1.000000000190015;
  for (j=0; j<=5; j++)
    ser += cof[j]/++y;
  return -tmp+log(2.5066282746310005*ser/x);
}


void nrerror(char error_text[])
  /* Numerical Recipes standard error handler */
{
  cerr << "Numerical Recipes run-time error..." << endl;
  cerr << "  " << error_text << endl;
  //   fprintf(stderr, "...now exiting to system...\n");
  //   exit(1);
}

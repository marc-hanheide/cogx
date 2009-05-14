/*********************************************************************/
/*                                                                   */
/* FILE         chisquare.cc                                         */
/* AUTHORS      Bastian Leibe                                        */
/* EMAIL        leibe@inf.ethz.ch                                    */
/*                                                                   */
/* CONTENT      A set of routines to compute different versions of   */
/*              the popular chi-square test for comparing two binned */
/*              sets. The algorithms and code are taken from the     */
/*              book "Numerical Recipes in C", 2nd edition.          */
/*                                                                   */
/* BEGIN        Fri Sep 07 2001                                      */
/* LAST CHANGE  Fri Sep 07 2001                                      */
/*                                                                   */
/*********************************************************************/

/****************/
/*   Includes   */
/****************/
#include <stdlib.h>  // for exit()
#include <math.h>
#include <iostream>

#include <libChiSquare/chisquare.hh>


void chstwo(double bins1[], double bins2[], int nbins, int knstrn,
	    double sum1, double sum2,
	    double *df, double *chsq, double *prob)
  /***************************************************************/
  /* Given the arrays bins1[0...nbins-1] and bins2[0...nbins-1], */
  /* containing two sets of binned data, and given the number of */
  /* constraints knstrn (normally 1 or 0), this routine returns  */
  /* the number of degrees of freedom df, the chi-square chsq,   */
  /* and the significance prob. A small value of prob indicates  */
  /* a significant difference between the two distributions      */
  /* bins1 and bins2. Note that bins1 and bins2 are both double  */
  /* arrays, although they will normally contain integer values. */
  /*                                                             */
  /* Routine taken from the book "Numerical Recipes in C",       */
  /* 2nd ed., pp. 620-623.                                       */
  /* Slightly modified to allow for double precision and diffe-  */
  /* rent number of data points in the two binned sets.          */
  /*   sum1/2 = number of data points in binned set 1/2          */ 
  /*            (or 1.0 if the two numbers are equal or don't    */
  /*            matter).                                         */
  /***************************************************************/
{
  int    j;
  double temp;
  double coeff1, coeff2;
  
  coeff1 = sqrt(sum2/sum1);
  coeff2 = sqrt(sum1/sum2);
  
  *df = nbins - knstrn;
  *chsq = 0.0;
  for (j=0; j<nbins; j++)
    if ((bins1[j] == 0.0) && (bins2[j] == 0.0))
      --(*df);       /* No data means one less degree of freedom */
    else {
      temp   = coeff1*bins1[j] - coeff2*bins2[j];
      *chsq += temp*temp / (bins1[j] + bins2[j]);
    }
  
  *prob = gammq(0.5*(*df), 0.5*(*chsq)); /* Chi-square prob function */
}


void chstwo(vector<float> bins1, vector<float> bins2, int knstrn,
	    double sum1, double sum2,
	    double *df, double *chsq, double *prob)
  /* modified version of chstwo for vector<float> arrays. */
{
  int    j;
  double temp;
  double coeff1, coeff2;
  
  int nbins = bins1.size();

  coeff1 = sqrt(sum2/sum1);
  coeff2 = sqrt(sum1/sum2);
  
  *df = nbins - knstrn;
  *chsq = 0.0;
  for (j=0; j<nbins; j++)
    if ((bins1[j] == 0.0) && (bins2[j] == 0.0))
      --(*df);       /* No data means one less degree of freedom */
    else {
      temp   = coeff1*bins1[j] - coeff2*bins2[j];
      *chsq += temp*temp / (bins1[j] + bins2[j]);
    }
  
  *prob = gammq(0.5*(*df), 0.5*(*chsq)); /* Chi-square prob function */
}


void chstwo_measure(double bins1[], double bins2[], int nbins, int knstrn,
                    double sum1, double sum2,
                    double *df, double *chsq)
  /***************************************************************/
  /* Given the arrays bins1[0...nbins-1] and bins2[0...nbins-1], */
  /* containing two sets of binned data, and given the number of */
  /* constraints knstrn (normally 1 or 0), this routine returns  */
  /* the number of degrees of freedom df, the chi-square chsq,   */
  /* and the significance prob. A small value of prob indicates  */
  /* a significant difference between the two distributions      */
  /* bins1 and bins2. Note that bins1 and bins2 are both double  */
  /* arrays, although they will normally contain integer values. */
  /*                                                             */
  /* Routine taken from the book "Numerical Recipes in C",       */
  /* 2nd ed., pp. 620-623.                                       */
  /* Slightly modified to allow for double precision and diffe-  */
  /* rent number of data points in the two binned sets.          */
  /*   sum1/2 = number of data points in binned set 1/2          */ 
  /*            (or 1.0 if the two numbers are equal or don't    */
  /*            matter).                                         */
  /***************************************************************/
{
  int    j;
  double temp;
  double coeff1, coeff2;
  
  coeff1 = sqrt(sum2/sum1);
  coeff2 = sqrt(sum1/sum2);
  
  *df = nbins - knstrn;
  *chsq = 0.0;
  for (j=0; j<nbins; j++)
    if ((bins1[j] == 0.0) && (bins2[j] == 0.0))
      --(*df);       /* No data means one less degree of freedom */
    else {
      temp   = coeff1*bins1[j] - coeff2*bins2[j];
      *chsq += temp*temp / (bins1[j] + bins2[j]);
    }
  
  //  *prob = gammq(0.5*(*df), 0.5*(*chsq)); /* Chi-square prob function */
}


void chstwo_measure(vector<float> bins1, vector<float> bins2, int knstrn,
                    double sum1, double sum2,
                    double *df, double *chsq)
  /* modified version of chstwo for vector<float> arrays. */
{
  int    j;
  double temp;
  double coeff1, coeff2;
  
  int nbins = bins1.size();

  coeff1 = sqrt(sum2/sum1);
  coeff2 = sqrt(sum1/sum2);
  
  *df = nbins - knstrn;
  *chsq = 0.0;
  for (j=0; j<nbins; j++)
    if ((bins1[j] == 0.0) && (bins2[j] == 0.0))
      --(*df);       /* No data means one less degree of freedom */
    else {
      temp   = coeff1*bins1[j] - coeff2*bins2[j];
      *chsq += temp*temp / (bins1[j] + bins2[j]);
    }
  
  //*prob = gammq(0.5*(*df), 0.5*(*chsq)); /* Chi-square prob function */
}


void chshom(double bins1[], double bins2[], int nbins, int knstrn,
                    double sum1, double sum2,
                    double *df, double *chsq, double *prob)
  /***************************************************************/
  /* Given the arrays bins1[0...nbins-1] and bins2[0...nbins-1], */
  /* containing two sets of binned data, and given the number of */
  /* constraints knstrn (normally 1 or 0), this routine returns  */
  /* the number of degrees of freedom df, the chi-square chsq,   */
  /* and the significance prob, calculated from the homogeneity  */
  /* test of Pearson's X^2. A small value of prob indicates      */
  /* a significant difference between the two distributions      */
  /* bins1 and bins2. Note that bins1 and bins2 are both double  */
  /* arrays, although they will normally contain integer values. */
  /*                                                             */
  /* The chi-square homogeneity test is applied when the test    */
  /* shall confirm whether two distributions are part of the     */
  /* same class (contrary to the usual chi-square test --as im-  */
  /* plemented in chstwo-- which checks whether one distribution */
  /* is the same as the other.                                   */
  /*                                                             */
  /*   sum1/2 = number of data points in binned set 1/2          */ 
  /*            (or 1.0 if the two numbers are equal or don't    */
  /*            matter).                                         */
  /***************************************************************/
{
  int    j;
  double temp;
  double coeff;
  
  *df   = nbins - knstrn;
  *chsq = 0.0;
  for (j=0; j<nbins; j++)
    if ((bins1[j] == 0.0) && (bins2[j] == 0.0))
      --(*df);       /* No data means one less degree of freedom */
    else {
      coeff  = (bins1[j] + bins2[j]) / (sum1 + sum2);
      temp   = bins1[j] - sum1*coeff;
      *chsq += temp*temp / (sum1*coeff);
      temp   = bins2[j] - sum2*coeff;
      *chsq += temp*temp / (sum2*coeff);
    }
  
  //*prob = 0.0;
  *prob = gammq(0.5*(*df), 0.5*(*chsq)); /* Chi-square prob function */
}


void chshom(vector<float> bins1, vector<float> bins2, int knstrn,
	    double sum1, double sum2,
	    double *df, double *chsq, double *prob)
  /* modified version of chshom for vector<float> arrays. */
{
  int    j;
  double temp;
  double coeff;

  int nbins = bins1.size();
  
  *df   = nbins - knstrn;
  *chsq = 0.0;
  for (j=0; j<nbins; j++)
    if ((bins1[j] == 0.0) && (bins2[j] == 0.0))
      --(*df);       /* No data means one less degree of freedom */
    else {
      coeff  = (bins1[j] + bins2[j]) / (sum1 + sum2);
      temp   = bins1[j] - sum1*coeff;
      *chsq += temp*temp / (sum1*coeff);
      temp   = bins2[j] - sum2*coeff;
      *chsq += temp*temp / (sum2*coeff);
    }
  
  //*prob = 0.0;
  *prob = gammq(0.5*(*df), 0.5*(*chsq)); /* Chi-square prob function */
}


void powerdiv(double bins1[], double bins2[], int nbins, int knstrn,
	      double lambda, double sum1, double sum2,
	      double *df, double *rc, double *prob)
  /***************************************************************/
  /* Computes the Power Divergence homogeneity test statistic    */
  /* corresponding to the specified value of lamda. Lambda is a  */
  /* continuous value that describes the type of the statistic   */
  /* to be calculated. Some special values of lambda are:        */
  /*   -2.0   => the Neyman-modified X^2 statistic NM^2          */
  /*   -1.0   => the modified Likelyhood-Ratio statistic GM^2    */
  /*   -0.5   => the Freeman-Tukey statistic F^2                 */
  /*    0.0   => the Likelyhood-Ratio statistic G^2              */
  /*    1.0   => the Pearson X^2 statistic                       */
  /* (Note that the values of exactly 0.0 and -1.0 are not       */
  /* allowed, as they would cause a division by zero.)           */
  /* The authors of                                              */
  /*   Timothy R.C. Read, Noel A.C Cressie,                      */
  /*   "Goodness-of-fit statistics for discrete multivariate     */
  /*   data", Springer, New York, 1988.                          */
  /* found that a value of lambda=2/3 is particularly suited     */
  /* for cases where the test situation (e.g. the type of the    */
  /* distribution) is not well known, and where the binned sets  */
  /* are very sparse.                                            */
  /*                                                             */
  /* Input parameters:                                           */
  /*   bins1/2= the two binned sets as arrays[0..nbins-1].       */
  /*   nbins  = size of the two sets (must be equal).            */
  /*   knstrn = number of constraints                            */
  /*            (=1 if the sets are normalized, else 0).         */
  /*   lambda = the type of the statistic to be calculated       */
  /*            (see above).                                     */
  /*   sum1/2 = number of data points in binned set 1/2.         */ 
  /*                                                             */
  /* Output parameters:                                          */
  /*   df     = the degrees of freedom                           */
  /*            (~number of non-zero bins)                       */
  /*   rc     = the result of the rc(lambda) statistics.         */
  /*   prob   = the significance probability calculated with a   */
  /*            chi-square probability function.                 */
  /*            (A small value of prob indicates a significant   */
  /*            difference between the two distributions.)       */
  /***************************************************************/
{
  int    j;
  double temp;
  double coeff;
  
  *df   = nbins - knstrn;
  *rc = 0.0;
  if ((lambda == 0.0) || (lambda == -1.0)) {
    cerr << "ERROR: lambda=" << lambda << " not allowed!" << endl;
    exit(1);
  }
  for (j=0; j<nbins; j++)
    if ((bins1[j] == 0.0) && (bins2[j] == 0.0))
      --(*df);       /* No data means one less degree of freedom */
    else {
      coeff  = (bins1[j] + bins2[j]) / (sum1 + sum2);
      *rc   +=  bins1[j] * (pow( bins1[j]/(sum1*coeff), lambda) - 1.0);
      *rc   +=  bins2[j] * (pow( bins2[j]/(sum2*coeff), lambda) - 1.0);
    }
  
  *rc  *= 2.0/(lambda * (lambda+1.0));
  *prob = gammq(0.5*(*df), 0.5*(*rc)); /* Chi-square prob function */
}

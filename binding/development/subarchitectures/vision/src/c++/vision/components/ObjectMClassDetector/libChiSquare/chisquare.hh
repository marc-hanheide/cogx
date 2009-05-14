/*********************************************************************/
/*                                                                   */
/* FILE         chisquare.hh                                         */
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

#ifndef LEIBE_CHISQUARE_HH
#define LEIBE_CHISQUARE_HH

#ifdef _USE_PERSONAL_NAMESPACES
//namespace Leibe {
#endif

/****************/
/*   Includes   */
/****************/
#include <vector>

#include <libChiSquare/gamma.hh>

using namespace std;

/***************************/
/*   Function Prototypes   */
/***************************/
void chstwo(double bins1[], double bins2[], int nbins, int knstrn,
            double sum1, double sum2,
            double *df, double *chsq, double *prob);
void chstwo(vector<float> bins1, vector<float> bins2, int knstrn,
            double sum1, double sum2,
            double *df, double *chsq, double *prob);

void chstwo_measure(double bins1[], double bins2[], int nbins, int knstrn,
                    double sum1, double sum2,
                    double *df, double *chsq);
void chstwo_measure(vector<float> bins1, vector<float> bins2, int knstrn,
                    double sum1, double sum2,
                    double *df, double *chsq);

void chshom(double bins1[], double bins2[], int nbins, int knstrn,
            double sum1, double sum2,
            double *df, double *chsq, double *prob);
void chshom(vector<float> bins1, vector<float> bins2, int knstrn,
            double sum1, double sum2,
            double *df, double *chsq, double *prob);

void powerdiv(double bins1[], double bins2[], int nbins, int knstrn,
              double lambda, double sum1, double sum2,
              double *df, double *rc, double *prob);


#ifdef _USE_PERSONAL_NAMESPACES
//}
#endif

#endif // LEIBE_CHISQUARE_HH

/*********************************************************************/
/*                                                                   */
/* FILE         gamma.hh                                             */
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

#ifndef LEIBE_GAMMA_HH
#define LEIBE_GAMMA_HH

#ifdef _USE_PERSONAL_NAMESPACES
//namespace Leibe {
#endif

/****************/
/*   Includes   */
/****************/

/*******************/
/*   Definitions   */
/*******************/
#define ITMAX 100         /* Maximum allowed number of iterations   */
#define EPS   3.0e-7      /* Relative accuracy                      */
#define FPMIN 1.0e-30     /* Number near the smallest representable */
/*                           floating point number                  */


/***************************/
/*   Function Prototypes   */
/***************************/
double gammq(double a, double x);



#ifdef _USE_PERSONAL_NAMESPACES
//}
#endif

#endif

/**
 * $Id: Math.cc,v 1.13 2007/03/25 21:35:57 mxz Exp mxz $
 */

#include "VisionCore.hh"
#include "Math.hh"

namespace Z
{
// table of log factorial
static const int FACT_TABSIZE = 10000;
static double log_fact_t[FACT_TABSIZE];
static double fact_t[FACT_TABSIZE];

/**
 * Init factorial and log factorial table.
 */
static void InitLogFactTab()
{
  log_fact_t[0] = 0;  // 0! = 1
  for(int i = 1; i < FACT_TABSIZE; i++)
    log_fact_t[i] = log_fact_t[i-1] + log((double)i);
  fact_t[0] = 1;  // 0! = 1
  for(int i = 1; i < FACT_TABSIZE; i++)
    fact_t[i] = fact_t[i-1]*(double)i;
}

void InitMath()
{
  InitLogFactTab();
  srand(2407771);
}

void ExitMath()
{
}

unsigned HammingDistance(unsigned a, unsigned b)
{
  a = a^b;
  b = 0;
  for(unsigned i = 0; i < 32; i++)
    b += (a >> i) & 0x1;
  return b;
}

/**
 * Clip line to rectangle (0,0)-(xmin, xmax) (including xmin and xmax).
 * Returns true if at least part of the line is inside, false if line is
 * completely outside.
 */
bool ClipLine(int xmax, int ymax, int *x1, int *y1, int *x2, int *y2)
{
  int xmin = 0, ymin = 0;
  // left
  if(*x1 >= xmin)    // don't clip point 1
  {
    if(*x2 < xmin)   // clip point 2
    {
      *y2 = *y2 + ((xmin - *x2)*(*y1 - *y2))/(*x1 - *x2);
      *x2 = xmin;
    }
  }
  else   // clip point 1
  {
    if(*x2 >= xmin)
    {
      *y1 = *y1 + ((xmin - *x1)*(*y2 - *y1))/(*x2 - *x1);
      *x1 = xmin;
    }
    else // both points are out of region
    {
      return false;
    }
  }
  // right
  if(*x1 <= xmax)    // don't clip point 1
  {
    if(*x2 > xmax)   // clip point 2
    {
      *y2 = *y2 - ((*x2 - xmax)*(*y2 - *y1))/(*x2 - *x1);
      *x2 = xmax;
    }
  }
  else   // clip point 1
  {
    if(*x2 <= xmax)
    {
      *y1 = *y1 - ((*x1 - xmax)*(*y1 - *y2))/(*x1 - *x2);
      *x1 = xmax;
    }
    else // both points are out of region
    {
      return false;
    }
  }
  // top
  if(*y1 >= ymin)    // don't clip point 1
  {
    if(*y2 < ymin)   // clip point 2
    {
      *x2 = *x2 + ((ymin - *y2)*(*x1 - *x2))/(*y1 - *y2);
      *y2 = ymin;
    }
  }
  else   // clip point 1
  {
    if(*y2 >= ymin)
    {
      *x1 = *x1 + ((ymin - *y1)*(*x2 - *x1))/(*y2 - *y1);
      *y1 = ymin;
    }
    else // both points are out of region
    {
      return false;
    }
  }
  // bottom
  if(*y1 <= ymax)    // don't clip point 1
  {
    if(*y2 > ymax)   // clip point 2
    {
      *x2 = *x2 - ((*y2 - ymax)*(*x2 - *x1))/(*y2 - *y1);
      *y2 = ymax;
    }
  }
  else   // clip point 1
  {
    if(*y2 <= ymax)
    {
      *x1 = *x1 - ((*y1 - ymax)*(*x1 - *x2))/(*y1 - *y2);
      *y1 = ymax;
    }
    else // both points are out of region
    {
      return false;
    }
  }
  return true;
}

/**
 * Factorial of n (n!).
 */
double Fact(int n)
{
  if(n < 0 || n >= FACT_TABSIZE)
    throw Except(__HERE__, "invalid argument %d, valid range is 0..%d\n", n,
        FACT_TABSIZE-1);
  return fact_t[n];
}

/**
 * Natural logarithm of factorial of n.
 *  log(n!)
 */
double LogFact(int n)
{
  if(n < 0 || n >= FACT_TABSIZE)
    throw Except(__HERE__, "invalid argument %d, valid range is 0..%d\n", n,
        FACT_TABSIZE-1);
  return log_fact_t[n];
}

/**
 * Natural logarithm of the binomial coefficient of n and k.
 *       n
 *  log( k )
 */
double LogBinCoef(int n, int k) throw(Except)
{
  if(k < 0 || k > n)
    throw Except(__HERE__, "k < 0 or k > n: n = %d, k = %d", n, k);
  return LogFact(n) - LogFact(k) - LogFact(n - k);
}

/**
 * Natural logarithm of binomial distribution density function.
 *        l    
 * log[ ( k )*p^k*(1-p)^(l-k) ]
 * TODO: deprecated
 */
double LogBinDist(int l, int k, double p)
{
  return LogBinCoef(l, k) + (double)k*log(p) + (double)(l - k)*log(1. - p);
}

/**
 * Natural logarithm of binomial distribution density function.
 *        l    
 * log[ ( k )*p^k*(1-p)^(l-k) ]
 */
double LogBinomialPDF(int l, int k, double p)
{
  return LogBinCoef(l, k) + (double)k*log(p) + (double)(l - k)*log(1. - p);
}

double BinomialPDF(int l, int k, double p)
{
  return exp(LogBinomialPDF(l, k, p));
}

/**
 * Natural logarithm of binomial cumulative distribution function.
 *    k        l    
 * Sum  log[ ( k )*p^k*(1-p)^(l-k) ]
 *   i=0
 */
double LogBinomialCDF(int l, int k, double p)
{
  return log(BinomialCDF(l, k, p));
}

double BinomialCDF(int l, int k, double p)
{
  double res = 0.;
  for(; k >= 0; k--)
    res += BinomialPDF(l, k, p);
  return res;
}

double LogBinomialCDF_tail(int l, int k, double p)
{
  return log(BinomialCDF_tail(l, k, p));
}


extern "C" {
extern float betai(float a, float b, float x);
}
double BinomialCDF_tail(int l, int k, double p)
{
  return betai(k, l - k + 1, (float)p);
}

/**
 * Significance that exactly k points support an m-parametric model of size l,
 * where an arbitrary point has probability p of lying on the model.
 */
double Significance(int m, int k, int l, double p)
{
  try
  {
    // note: actually the correct probability (that at least k points support)
    // would be
    // P(k>=l) = Sum(BinDist(l,i,p)) for i = k..l
    // but for small p BinDist() falls off so quickly, that the later parts
    // do barely matter.
    return LogBinCoef(k, m) - LogBinDist(l, k, p);
  }
  catch(...)
  {
    return 0.;
  }
}

/**
 * Subtract the `struct timeval' values x - y.
 * Return 1 if the difference is negative, otherwise 0.
 * see http://www.delorie.com/gnu/docs/glibc/libc_428.html
 */
int timeval_subtract(struct timeval *result, struct timeval *x,
  struct timeval *y)
{
  // Perform the carry for the later subtraction by updating y.
  if(x->tv_usec < y->tv_usec)
  {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_usec - y->tv_usec > 1000000)
  {
    int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }
  // now tv_usec is certainly positive
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;
  return x->tv_sec < y->tv_sec;
}

/**
 * Returns timespecs x - y as double.
 */
double timespec_diff(struct timespec *x, struct timespec *y)
{
  /*timespec res = {x->tv_sec - y->tv_sec, x->tv_nsec - y->tv_nsec};
  if(res.tv_nsec >= 1000000000)
  {
    res.tv_nsec -= 1000000000;
    res.tv_sec++;
  }
  if(res.tv_nsec < 0)
  {
    res.tv_nsec += 1000000000;
    res.tv_sec--;
  }
  return (double)res.tv_sec + 1e-9*(double)res.tv_nsec;*/
  /* TODO: make the above clearer version handle nsec overflows of more than one
   * sec, see below*/
  if(x->tv_nsec < y->tv_nsec)
  {
    int nsec = (y->tv_nsec - x->tv_nsec) / 1000000000 + 1;
    y->tv_nsec -= 1000000000 * nsec;
    y->tv_sec += nsec;
  }
  if(x->tv_nsec - y->tv_nsec > 1000000000)
  {
    int nsec = (x->tv_nsec - y->tv_nsec) / 1000000000;
    y->tv_nsec += 1000000000 * nsec;
    y->tv_sec -= nsec;
  }
  return (double)(x->tv_sec - y->tv_sec) +
    (double)(x->tv_nsec - y->tv_nsec)/1000000000.;
}

/**
 * Poisson probability distribution function.
 * f: i -> exp^-alpha * alpha^i / i!
 */
double PoissonPDF(int i, double alpha)
{
  if(i < 0)
    return 0.;
  else
    return exp(-alpha)*pow(alpha, (double)i)/Fact(i);
}

/**
 * Poisson cumulative distribution function.
 * f: k -> Sum( exp^-alpha * alpha^i / i! )  i=0..k
 */
double PoissonCDF(int k, double alpha)
{
  if(k < 0)
    return 0.;
  else
  {
    int i;
    double p = 0., pa = 1., fi = 1;
    for(i = 0; i <= k; i++)
    {
      p += pa/fi;
      pa *= alpha;
      fi *= (double)(i+1);
    }
    return exp(-alpha)*p;
  }
}

}


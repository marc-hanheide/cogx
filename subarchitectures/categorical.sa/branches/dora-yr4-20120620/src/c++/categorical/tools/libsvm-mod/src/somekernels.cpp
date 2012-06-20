#include "localkernels.h"


double chiSquared(const svm_node *px, const svm_node *py, double gamma)
{
  // compute chi square kernel w.r.t. the sparse data structure
  double sum = 0.0;
  while(px->index != -1 && py->index != -1)
  {
    if(px->index == py->index)
    {
      double tmp= px->value - py->value;
      sum+= tmp*tmp / fabs(px->value + py->value);
      ++px;
      ++py;
    }
    else if(px->index > py->index)
    {
      sum+= fabs(py->value);
      ++py;
    }
    else
    {
      sum+= fabs(px->value);
      ++px;
    }
  }

  while(py->index != -1)
  {
    sum+= fabs(py->value);
    ++py;
  }

  while(px->index != -1)
  {
    sum+= fabs(px->value);
    ++px;
  }

  return exp(-gamma * sum);
}

double generalizedGauss(const svm_node *px, const svm_node *py, double b, double a, double gamma)
{
  // compute generalized gaussian kernel w.r.t. sparse data structure
  double sum = 0.0;
  while(px->index != -1 && py->index != -1)
  {
    if(px->index == py->index)
    {
      sum+= pow( fabs( pow(px->value, a) - pow(py->value, a)),b);
      ++px;
      ++py;
    }
    else if(px->index > py->index)
    {
      sum+= pow(fabs(py->value), a*b);
      ++py;
    }
    else
    {
      sum+= pow(fabs(px->value), a*b);
      ++px;
    }
  }

  while(py->index != -1)
  {
    sum+= pow(fabs(py->value), a*b);
    ++py;
  }

  while(px->index != -1)
  {
    sum+= pow(fabs(px->value), a*b);
    ++px;
  }

  return exp(-gamma * sum);
}



double intersection(const svm_node *px, const svm_node *py)
{
  // compute intersection kernel w.r.t. the sparse data structure
  double sum = 0.0;

  while(px->index != -1 && py->index != -1)
  {
    if(px->index == py->index)
    {
      if( px->value <= py->value )
      {
        sum+= px->value;
      }
      else
      {
        sum+= py->value;
      }

      ++px;
      ++py;
    }
    else if(px->index > py->index)
    {
      ++py;
    }
    else
    {
      ++px;
    }
  }

  while(px->index != -1)
  {
    ++px;
  }
  while(py->index != -1)
  {
    ++py;
  }
  return  sum;
}

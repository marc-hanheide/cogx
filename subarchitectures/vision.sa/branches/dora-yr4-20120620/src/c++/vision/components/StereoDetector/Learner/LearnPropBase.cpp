/**
 * @file LearnPropBase.cpp
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Learn one property distribution
 */

#include <vector>
#include <stdio.h>
#include <math.h>

#include "LearnPropBase.h"

namespace Z
{

/**
 * @brief Constructor of LearnPropBase
 */
LearnPropBase::LearnPropBase()
{
  locked = false;
  distribution_validity = false;
}


/**
 * @brief Destructor of LearnPropBase
 */
LearnPropBase::~LearnPropBase()
{}

/**
 * @brief Reset
 */
void LearnPropBase::Reset()
{
  printf("LearnPropBase::Reset: Warning: Empty Prop-Base!\n");
  pos_data.clear();
  neg_data.clear();
  locked = false;
  distribution_validity = false;
}

/**
 * @brief Add values to the distribution
 * @param pos Positive values
 * @param neg Negative values
 */
void LearnPropBase::AddValues(const std::vector<double> &pos, const std::vector<double> &neg)
{
  if(locked) return;
  for(unsigned i=0; i<pos.size(); i++)
    pos_data.push_back(pos[i]);
  for(unsigned i=0; i<neg.size(); i++)
    neg_data.push_back(neg[i]);
  distribution_validity = false;
}

/**
 * @brief Add positive values to the distribution
 * @param pos Positive values
 */
void LearnPropBase::AddPosValues(const std::vector<double> &pos)
{
  if(locked) return;
  for(unsigned i=0; i<pos.size(); i++)
    pos_data.push_back(pos[i]);
  distribution_validity = false;
}


/**
 * @brief Add negative values to the distribution
 * @param neg Negative values
 */
void LearnPropBase::AddNegValues(const std::vector<double> &neg)
{
  if(locked) return;
  for(unsigned i=0; i<neg.size(); i++)
    neg_data.push_back(neg[i]);
  distribution_validity = false;
}
  
/**
 * @brief Calculate the distribution with mean value and variance
 */
void LearnPropBase::CalculateDistribution()
{
printf("LearnPropBase::CalculateDistribution: number of pos/neg samples: %u / %u!\n", pos_data.size(), neg_data.size());
  if(locked)
  {
    printf("LearnPropBase::CalculateDistribution: Warning: PropBase is locked!\n");
    return;
  }

  pos_meanValue = 0;
  neg_meanValue = 0;
  pos_variance = 0;
  neg_variance = 0;
  
  if(pos_data.size() < 2 || neg_data.size() < 2) 
    return;
  
  for(unsigned i=0; i<pos_data.size(); i++)
    pos_meanValue += pos_data[i];
  pos_meanValue /= pos_data.size();
  
  for(unsigned i=0; i<neg_data.size(); i++)
    neg_meanValue += neg_data[i];
  neg_meanValue /= neg_data.size();
  
  for(unsigned i=0; i<pos_data.size(); i++)
    pos_variance += pow((pos_data[i]-pos_meanValue), 2);
  pos_variance /= pos_data.size()-1;
  pos_st_deviation = sqrt(pos_variance);
  
  for(unsigned i=0; i<neg_data.size(); i++)
    neg_variance += pow((neg_data[i]-neg_meanValue), 2);
  neg_variance /= neg_data.size();
  neg_st_deviation = sqrt(neg_variance);
  
  distribution_validity = true;
}

double LearnPropBase::GetPosMeanValue()
{
  if(!distribution_validity && !locked) 
    CalculateDistribution();
  return pos_meanValue;
}

double LearnPropBase::GetPosVariance()
{
  if(!distribution_validity && !locked) 
    CalculateDistribution();
  return pos_variance;
}

double LearnPropBase::GetPosStDeviation()
{
  if(!distribution_validity && !locked) 
    CalculateDistribution();
  return pos_st_deviation;
}

double LearnPropBase::GetNegMeanValue()
{
  if(!distribution_validity && !locked) 
    CalculateDistribution();
 return neg_meanValue;
}
  
double LearnPropBase::GetNegVariance()
{
  if(!distribution_validity && !locked) 
    CalculateDistribution();
  return neg_variance;
}

double LearnPropBase::GetNegStDeviation()
{
  if(!distribution_validity && !locked) 
    CalculateDistribution();
  return neg_st_deviation;
}

/**
 * @brief Calculate the probability of a positive event.
 * @param val Observed value
 * @return Probability for the observed value
 */
double LearnPropBase::GetProbability(const double &val)
{
  if(!distribution_validity && !locked) 
    CalculateDistribution();

  /// p(true|x) = p(x|true)*p(true) / p(x)
  ///           = p(x|true)*p(true) / SUM(p(x|true or false)))
  ///           = p(x|true)*p(true) / (p(x|true) + p(x|false))
  ///           = p(x|true) / (p(x|true) + p(x|false))
  
  double pos_c = 1/(pos_st_deviation*sqrt(2*M_PI));
  double pos_s = -0.5*pow(((val-pos_meanValue)/pos_st_deviation), 2);
  double pos_fx = pos_c*exp(pos_s);

  double neg_c = 1/(neg_st_deviation*sqrt(2*M_PI));
  double neg_s = -0.5*pow(((val-neg_meanValue)/neg_st_deviation), 2);
  double neg_fx = neg_c*exp(neg_s);

  double p = pos_fx / (pos_fx + neg_fx);

//   printf("  val: %4.4f\n", val);
//   printf("  Pos: mean: %4.4f, std_devi: %4.4f\n", pos_meanValue, pos_st_deviation);
//   printf("  Neg: mean: %4.4f, std_devi: %4.4f\n", neg_meanValue, neg_st_deviation);
//   printf("  p(x|true): %4.4f\n", pos_fx);
//   printf("  p(x|false): %4.4f\n", neg_fx);
//   printf("     => p = %6.6f\n", p);
  
  return p;
}


/**
 * @brief Calculate the cdf value for the standard normal distribution.
 * @param x Observed value
 * @return Returns the probability value for the observed value      
 */
double LearnPropBase::phi(double x)
{
  printf("LearnPropBase::phi: Warning: antiquated!\n");

  // constants
  double a1 =  0.254829592;
  double a2 = -0.284496736;
  double a3 =  1.421413741;
  double a4 = -1.453152027;
  double a5 =  1.061405429;
  double p  =  0.3275911;

  // Save the sign of x
  int sign = 1;
  if (x < 0)
      sign = -1;
  x = fabs(x)/sqrt(2.0);

  // A&S formula 7.1.26
  double t = 1.0/(1.0 + p*x);
  double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

  return 0.5*(1.0 + sign*y);
}

/**
 * @brief Set the distribution values and do not calculat them.
 * @param pmv Positive mean value
 * @param pv Positive variance
 * @param psd Positive standard deviation
 * @param nmv Negative mean value
 * @param nv Negative variance
 * @param nsd Negative standard deviation
 */
void LearnPropBase::SetDistributionValues(double pmv, double pv, double psd, double nmv, double nv, double nsd)
{
  locked = true;
  pos_meanValue = pmv;
  pos_variance = pv;
  pos_st_deviation = psd;
  neg_meanValue = nmv;
  neg_variance = nv;
  neg_st_deviation = nsd;
}



}












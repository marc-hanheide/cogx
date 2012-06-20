/**
 * @file LearnPropBase.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Learn one property distribution
 */

#ifndef Z_LEARN_PROP_BASE_H
#define Z_LEARN_PROP_BASE_H

#include <vector>

namespace Z
{

/**
 * @brief Class LearnPropBase
 */
class LearnPropBase
{
private:
  bool locked;
  bool distribution_validity;
  
  std::vector<double> pos_data;
  std::vector<double> neg_data;

  double pos_meanValue;
  double pos_variance;  
  double pos_st_deviation;
  double neg_meanValue;
  double neg_variance;  
  double neg_st_deviation;
  
  void Reset();
  void CalculateDistribution();
  
  double phi(double x);
//   void testPhis();
  
public:
  LearnPropBase();
  ~LearnPropBase();
  
  void AddValues(const std::vector<double> &pos, const std::vector<double> &neg);
  void AddPosValues(const std::vector<double> &pos);
  void AddNegValues(const std::vector<double> &neg);
  
  double GetPosMeanValue();
  double GetPosVariance();
  double GetPosStDeviation();
  double GetNegMeanValue();
  double GetNegVariance();
  double GetNegStDeviation();
  double GetProbability(const double &val);
  
  void SetDistributionValues(double pmv, double pv, double psd, double nmv, double nv, double nsd);
};

}

#endif
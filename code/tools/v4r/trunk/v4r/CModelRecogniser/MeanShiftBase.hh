/**
 * $Id$
 */

#ifndef P_MEAN_SHIFT_BASE_HH
#define P_MEAN_SHIFT_BASE_HH

#include <iostream>
#include <opencv2/core/core.hpp>



namespace P
{

using namespace std;

class MeanShiftBase
{
private:


public:
  MeanShiftBase() {}
  ~MeanShiftBase() {}

  virtual void Cluster(cv::Mat_<double> &samples, vector<vector<unsigned> > &clusters) {};
};

}

#endif


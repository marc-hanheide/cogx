/**
 * $Id$
 */

#ifndef P_MEAN_SHIFT_HH
#define P_MEAN_SHIFT_HH

#include <iostream>
#include <vector>
#include <opencv2/core/core.hpp>
#include "ClusterSample.hh"
#include "IMCluster.hh"
#include "MeanShiftBase.hh"



namespace P
{

using namespace std;

class MeanShift : public MeanShiftBase
{
private:
  double sqrLambda;
  double invSqrSigma;
  double sqrThrPruning;
  double initPcent;
  unsigned maxIter;
  double maxDist;

  void AddSampesToCluster(vector<ClusterSample *> &samples, IMCluster* cluster);
  void ComputeMean(IMCluster *cluster);
  int UpdateMean(vector<ClusterSample *> &samples, IMCluster* cluster);
  void InitClusters(vector<ClusterSample *> &samples, vector<IMCluster* > &clusters);

  inline bool IsConverge(double *d1, double *d2, unsigned dSize);
  inline double TruncatedGaussian( double sqrDist);


public:
  MeanShift();
  MeanShift(double lambda, double sigma, double thrPrune, double initP,unsigned thrIter=100, double epsConverge=PMath::eps);
  ~MeanShift();
  void Cluster(vector<ClusterSample *> &samples, vector<IMCluster* > &clusters);
  virtual void Cluster(cv::Mat_<double> &samples, vector<vector<unsigned> > &clusters);
};




/*********************** INLINE METHODES **************************/

inline bool MeanShift::IsConverge(double *d1, double *d2, unsigned dSize)
{
  for (unsigned i=0; i<dSize; i++)
    if (fabs(*d1-*d2) > maxDist)
      return false;

  return true;
}

inline double MeanShift::TruncatedGaussian( double sqrDist)
{
  if (sqrDist < sqrLambda)
  {
    return exp(-invSqrSigma*sqrDist);
  }
  
  return 0.; 
}




}

#endif


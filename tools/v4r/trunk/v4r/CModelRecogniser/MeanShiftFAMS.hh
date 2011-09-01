/**
 * $Id$
 */

#ifndef P_MEAN_SHIFT_FAMS_HH
#define P_MEAN_SHIFT_FAMS_HH

#include <iostream>
#include <map>
#include <vector>
#include <opencv2/core/core.hpp>
#include "fams.h"
#include "v4r/PMath/PMath.hh"
#include "MeanShiftBase.hh"



namespace P
{


class MeanShiftFAMS : public MeanShiftBase
{
public:
  class Parameter
  {
  public:
    int K, L, k_neigh;
    int no_lsh, find_kl;
    float epsilon;
    int Kmin, Kjump, Kmax;
    int Lmax;
    float width;
    int jump;
    double percent;
   
    Parameter()
     : K(14), L(16), k_neigh(200), no_lsh(0), find_kl(0),
       epsilon(0.05), Kmin(10), Kjump(2), Kmax(K), Lmax(L),
       width(-1), jump(1), percent(.0) {}
  };

private:

  char *pilotfile;

  vector<double> modes;
  unsigned nModes,nDim;
   
  void ConvertData(FAMS *fams, double *data, unsigned num, unsigned dim);
  void CopyModes(FAMS *fams, unsigned &numModes);
  
  inline double DistSqr(double *d1, double *d2, unsigned dim);


public:
  Parameter param;

  MeanShiftFAMS(Parameter _param=Parameter());
  ~MeanShiftFAMS();
  double* Operate(double *data, unsigned num, unsigned dim, unsigned &numModes, unsigned *labels);
  virtual void Cluster(cv::Mat_<double> &samples, vector<vector<unsigned> > &clusters);
};




/*********************** INLINE METHODES **************************/

inline double MeanShiftFAMS::DistSqr(double *d1, double *d2, unsigned dim)
{
  double distsqr=0;

  for (unsigned i=0; i<dim; i++, d1++, d2++)
  {
    distsqr += PMath::Sqr(*d1 - *d2);
  }

  return distsqr;
}







}

#endif


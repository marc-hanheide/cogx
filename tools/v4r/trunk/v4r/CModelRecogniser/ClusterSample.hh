/**
 * $Id$
 */

#ifndef P_CLUSTER_SAMPLE_HH
#define P_CLUSTER_SAMPLE_HH

#include <limits.h>
#include <vector>

namespace P
{

class ClusterSample
{
protected:

public:
  unsigned id;
  double *vec;
  unsigned size;

  ClusterSample() : id(UINT_MAX), vec(0), size(0) {}
  ClusterSample(double *v, unsigned s) : id(UINT_MAX), vec(v), size(s) {}
  ClusterSample(double *v, unsigned s, unsigned _id) : id(_id), vec(v), size(s) {}
  virtual double* GetVec(){return vec;}
  virtual unsigned GetSize(){return size;}
};

static void DeleteClusterSamples(std::vector<ClusterSample *> &samples);

/*********************************** INLINE *********************************/

void DeleteClusterSamples(std::vector<ClusterSample *> &samples)
{
  for (unsigned i=0; i<samples.size(); i++)
    delete samples[i];
  samples.clear();
}

}

#endif


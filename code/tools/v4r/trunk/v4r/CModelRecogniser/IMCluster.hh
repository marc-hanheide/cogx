/**
 * $Id$
 */

#ifndef P_IMCLUSTER_HH
#define P_IMCLUSTER_HH

#include <limits.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include "ClusterSample.hh"
#include "v4r/PMath/PMath.hh"

namespace P
{

class IMCluster
{
public:

  double *vec;
  unsigned size;

  double sqrSigma;

  std::vector<ClusterSample *> samples;

  IMCluster() : vec(0), size(0), sqrSigma(0.) {}
  IMCluster( double *v, unsigned s ) : vec(0), sqrSigma(0)
  {
    Alloc(s);
    Copy(v);
  }
  IMCluster( double *v, unsigned s, ClusterSample *sample) : vec(0), sqrSigma(0)
  {
    Alloc(s);
    Copy(v);
    samples.push_back(sample);
  };
  ~IMCluster()
  {
    if (vec!=0) delete[] vec;
  };

  inline unsigned ClusterSize(){return samples.size();}
  inline unsigned GetSize(){return size;}
  inline double* GetVec(){return vec;}
  inline void Alloc(unsigned s);
  inline void Copy( double *s);
  inline void GetCopy(double *d);
  inline void Add( double *v);
  inline void Div( double num);
  inline void Mul( double num);
  inline void AddMul( double *v, double num);
  inline void SetZero();
  inline double DistSqr(double *v);
  inline double CombinedSqrSigma(double *v);
  inline void Insert( double *v,  double sqrSigma, ClusterSample *sample); 
  inline double NormSqr(double *v);
  inline double Norm(double *v);
};

void DeleteIMClusters(std::vector<IMCluster *> &clusters);


/************************************ INLINE METHODES ********************************/

inline void IMCluster::Alloc(unsigned s)
{
  if (vec!=0) delete[] vec;
  size=s;
  vec = new double[size];
}

inline void IMCluster::Copy(double *s)
{
  double *d=vec;
  for (register unsigned i=0; i<size; i++)
    *d++=*s++;
}

inline void IMCluster::GetCopy( double *d)
{
  double *s=vec;
  for (unsigned i=0; i<size; i++)
    *d++=*s++;
}

inline double IMCluster::NormSqr(double *v)
{
  double norm=0;

  for (unsigned z=0; z< size; z++)
    norm += PMath::Sqr(*v++);

  return norm;
}

inline double IMCluster::Norm(double *v)
{
  double norm=0;

  for (unsigned z=0; z< size; z++)
    norm += PMath::Sqr(*v++);

  return sqrt(norm);
}

inline void IMCluster::Add(double *v)
{
  register  double *d = vec;

  for (unsigned i = 0; i < size; i++) 
    *d++ += *v++;
}

inline void IMCluster::AddMul( double *v, double num)
{
  register  double *d = vec;

  for (unsigned i = 0; i < size; i++, d++, v++)
    *d += ((*v) * num);
}

inline void IMCluster::Div(double num)
{
  register double *d=vec;

  for (unsigned z=0; z< size; z++)
    *d++ /= num;
}

inline void IMCluster::Mul(double num)
{
  register double *d=vec;

  for (unsigned z=0; z< size; z++)
    *d++ *= num;
}

inline void IMCluster::SetZero()
{
  register double *d=vec;

  for (unsigned z=0; z< size; z++)
    *d++ = 0.;
}

inline double IMCluster::CombinedSqrSigma(double *v)
{
  return 1. / (ClusterSize()+1.) * (ClusterSize()*sqrSigma + ClusterSize()/(ClusterSize()+1.) * DistSqr(v));
}

inline double IMCluster::DistSqr(double *v)
{
  double dif;
  double distsq = 0;
  double *v2 = vec;

  for (unsigned i = 0; i < size; i++) 
  {
    dif = *v2++ - *v++;
    distsq += dif * dif;
  }
  return distsq;
}

inline void IMCluster::Insert(double *v, double sqrSigma,  ClusterSample *sample)
{
  samples.push_back(sample);

  //compute new sigma
  this->sqrSigma = sqrSigma;

  //compute new mean model of two clusters
  Mul(ClusterSize());
  Add(v);
  Div(ClusterSize()+1.);
}


}


#endif





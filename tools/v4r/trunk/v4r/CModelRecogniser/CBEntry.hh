/**
 * $Id$
 */

#ifndef P_CB_ENTRY_HH
#define P_CB_ENTRY_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <opencv2/core/core.hpp>
#include <vector>
#include <map>
#include "v4r/PMath/PMath.hh"
#include "View.hh"


namespace P
{


class CBEntry
{
public:
  float sqrThr;

  vector<float> data;

  map<unsigned, vector<unsigned> > idxViewKeys;     //<idxView, vector<idxKey> >
  const unsigned &N;
  unsigned ni;
  float wi;

  CBEntry(unsigned &_N, float _sqrThr);
  CBEntry(unsigned &_N, float *d, unsigned size, unsigned vidx, unsigned kidx, float _sqrThr);
  ~CBEntry();

  void InsertMeanShift(vector<cv::Ptr<View> > &views, unsigned vidx, unsigned kidx, float invSqrSigma);

  inline unsigned mi(unsigned vidx){return idxViewKeys[vidx].size();}
  inline unsigned Ni(){return idxViewKeys.size();}

  inline void copyTo(float *d);
  inline void SetZero();
  inline void Mul(float d);
  inline void Add(const CBEntry &d);
  inline void AddMul(float *v, float num);
  inline float DistSqr(const CBEntry &d);

  static void SetZero(float *d, unsigned size);
  static void Mul(float m, float *d, unsigned size);
  static void Add(float *d1, float *d2, float *r, unsigned size);
  static float DistSqr(float *d1, float *d2, unsigned size);
};






/*************************** INLINE METHODES **************************/
inline void CBEntry::copyTo(float *d)
{
  for (unsigned i=0; i<data.size(); i++,d++)
    *d = data[i];
}

inline void CBEntry::AddMul(float *v, float num)
{
  for (unsigned i=0; i<data.size(); i++, v++)
    data[i] += ((*v) * num);
}


inline void CBEntry::SetZero()
{
  for (unsigned i=0; i<data.size(); i++)
    data[i] = 0;
}

inline void CBEntry::Mul(float d)
{
  for (unsigned i=0; i<data.size(); i++)
    data[i] *= d;
}

inline void CBEntry::Add(const CBEntry &d)
{
  if (data.size() != d.data.size())
    throw runtime_error ("CBEntry::Add : Wrong descriptor size!");

  for (unsigned i=0; i<data.size(); i++)
    data[i] += d.data[i];
}

inline float CBEntry::DistSqr(const CBEntry &d)
{
  if (data.size()==0 || d.data.size() != data.size())
    return FLT_MAX;

  float distsq=0;

  for (unsigned i=0; i<data.size(); i++)
    distsq += PMath::Sqr(data[i] - d.data[i]);

  return distsq;
}


inline void CBEntry::SetZero(float *d, unsigned size)
{
  float *d_end = d+size;
  for (;d!=d_end; d++)
    (*d) = 0.;
}

inline void CBEntry::Mul(float m, float *d, unsigned size)
{
  float *d_end = d+size;
  for (;d!=d_end; d++)
    (*d) *= m;
}

inline void CBEntry::Add(float *d1, float *d2, float *r, unsigned size)
{
  float *d_end = d1+size;
  for (;d1!=d_end; d1++, d2++, r++)
    *r = *d1 + *d2;
}

inline float CBEntry::DistSqr(float *d1, float *d2, unsigned size)
{
  float distsq=0;

  float *d_end = d1+size;
  for (;d1!=d_end; d1++, d2++)
    distsq += PMath::Sqr(*d1 - *d2);

  return distsq;
}




} //--END--

#endif


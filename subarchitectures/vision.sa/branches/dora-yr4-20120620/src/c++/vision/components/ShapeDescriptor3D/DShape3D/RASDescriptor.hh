/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_RAS_DESCRIPTOR_HH
#define P_RAS_DESCRIPTOR_HH

#include "PNamespace.hh"
#include "Array.hh"
#include "opencv/cxcore.h"
#include "KeypointDescriptor.hh"
#include "Vector3.hh"
#include <string>

namespace P
{


class RASDescriptor
{
public:
  float *data;

  int size;
  int sa;
  int ss;

  RASDescriptor();
  ~RASDescriptor();

  inline void Set(int sizeAngle, int sizeScale);
  inline void Clear();
  inline unsigned Size(){return size;}

  inline void Insert(double relAngle, double relScale);
  inline void Normalise();

  static float Compare(RASDescriptor *ras1, RASDescriptor *ras2);
  static void SaveDescriptor(ofstream &os, RASDescriptor *ras, char *name=0);
  static void LoadDescriptor(ifstream &is, RASDescriptor *ras, string &name);

};


/*********************** INLINE METHODES **************************/

inline void RASDescriptor::Set(int sizeAngle, int sizeScale)
{
  if (data!=0) delete[] data;

  sa = sizeAngle;
  ss = sizeScale;
  size = sa*ss;
  data = new float[sa*ss];
}

inline void RASDescriptor::Clear()
{
  for (unsigned  i=0; i<Size(); i++)
    data[i] = 0.;
}

/**
 * insert relative ange and relative scale of two planes to the ras-histogram
 * (two dimensional)
 */
inline void RASDescriptor::Insert(double relAngle, double relScale)
{
  double a = ScaleAngle_0_2pi(relAngle)*((double)sa)/two_pi;
  // bin into which value falls
  int i = (int)floor(a);
  int i1, i2;
  double w1, w2;
  // if value left of center of bin
  if(a - (double)i <= 0.5)
  {
    // consider bin[i] and its left neighbour
    i2 = i;
    i1 = (i == 0 ? sa - 1 : i - 1);
    w2 = a - (double)i + 0.5;
    w1 = 1. - w2;
  }
  // if value right of center of bin
  else
  {
    // consider bin[i] and its right neighbour
    i1 = i;
    i2 = (i == sa - 1 ? 0 : i + 1);
    w1 = (double)(i + 1) - a + 0.5;
    w2 = 1. - w1;
  }
  // NOTE: the above could be made more elegant
  // HACK: here we assume that we don't use scale, i.e. ss == 1
  assert(ss == 1);
  data[i1] += w1;
  data[i2] += w2;

  /*int a[3], s[3];

  a[1] = (int)(ScaleAngle_0_2pi(relAngle)*((double)sa)/two_pi);
  a[0] = (a[1]==0?sa-1:a[1]-1);
  a[2] = (a[1]==sa-1?0:a[1]+1);
printf("insert angle %lf -> %d\n", relAngle, a[1]);
  relScale = fabs(relScale);
  s[1] = (int)(relScale<1. ? ((double)ss)*relScale : ((double)ss)*(1./relScale));
  s[0] = (s[1]==0?INT_MAX:s[1]-1);
  s[2] = (s[1]==ss-1?INT_MAX:s[1]+1);

  for (unsigned i=0; i<3; i++)
    for (unsigned j=0; j<3; j++)
      if (s[i]!=INT_MAX)
        data[s[i]*sa + a[j]] += 1.;*/
}

/**
 * insert relative ange and relative scale of two planes to the ras-histogram
 * (one dimensional)
 */
/*inline void RASDescriptor::Insert(double relAngle, double relScale)
{
  int a[3], s[3];

  a[1] = (int)ScaleAngle_0_2pi(relAngle)*sa/two_pi;
  a[0] = (a[1]==0?sa-1:a[1]-1);
  a[2] = (a[1]==sa-1?0:a[1]+1);

  relScale = fabs(relScale);

  s[1] = (int)(relScale<1. ? ss*relScale : ss*(1./relScale));
  s[0] = (s[1]==0?INT_MAX:s[1]-1);
  s[2] = (s[1]==ss-1?INT_MAX:s[1]+1);

  for (int i=0; i<3; i++)
    data[a[i]] += 1.;

  for (int i=0; i<3; i++)
    if (s[i]!=INT_MAX)
      data[sa+s[i]] += 1.;
}*/

inline void RASDescriptor::Normalise()
{
  float sum=0;
  for (unsigned i=0; i<Size(); i++)
    sum += data[i];

  if (sum>0.)
    for (unsigned i=0; i<Size(); i++) 
      data[i] /= sum;
}


}

#endif


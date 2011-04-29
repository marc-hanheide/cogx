/**
 * $Id$
 */

#ifndef P_DISTIDX_HH
#define P_DISTIDX_HH

namespace P
{


class DistIdx
{
public:
  double dist;
  unsigned idx;

  DistIdx(){};
  DistIdx(double d, unsigned i) : dist(d), idx(i) {};
  ~DistIdx(){};
};


/**
 * for sorting an array increasing dists
 */
inline int CmpDistIdxAsc(const void *a, const void *b)
{ 
  if ( ((DistIdx*)a)->dist < ((DistIdx*)b)->dist)
    return -1;  // a is first
  else 
    return 1 ;  // b is first
} 

/**
 * for sorting an array decreasing dists
 */

inline int CmpDistIdxDsc(const void *a, const void *b)
{ 
  if ( ((DistIdx*)a)->dist > ((DistIdx*)b)->dist)
    return -1;  // a is first
  else 
    return 1 ;  // b is first
} 



}

#endif


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
static const bool CmpDistIdxAsc(const DistIdx &a, const DistIdx &b)
{ 
  return (a.dist < b.dist);
} 



}

#endif


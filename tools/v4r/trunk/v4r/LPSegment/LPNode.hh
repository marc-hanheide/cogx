/**
 * $Id$
 */

#ifndef P_LPNODE_HH
#define P_LPNODE_HH

#include "v4r/CEdge/Array.hh"

namespace P
{


class LPNode
{
public:
  int x, y;
  float cost;
  bool e;                  //expanded?
  double T;                 //total cost 

  LPNode *B;
  P::Array<LPNode*> N; //neighbours

  LPNode() {};
  LPNode(int _x, int _y, float c) : x(_x), y(_y), cost(c), e(false), T(DBL_MAX), B(0) {};
  LPNode(int _x, int _y) : x(_x), y(_y), e(false), T(DBL_MAX), B(0) {};
  ~LPNode(){};
};

}


#endif


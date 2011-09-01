#ifndef P_MATCHPAIRS_HH
#define P_MATCHPAIRS_HH
  
#include <iostream>
#include "PKeypoint.hh"

namespace P
{

class MatchPairs
{
public:
  unsigned idxObject;
  vector<unsigned> idxViews;
  vector<vector<PKeypoint*> > ptsModel;   // model points sorted by views
  vector<vector<PKeypoint*> > ptsImage;   // according image points
  MatchPairs() {};
  MatchPairs(unsigned oidx) : idxObject(oidx) {};
};

}

#endif

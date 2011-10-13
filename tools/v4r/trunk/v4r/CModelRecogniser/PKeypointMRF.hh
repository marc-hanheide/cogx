/**
 * $Id$
 */

#ifndef P_PKEYPOINT_MRF_HH
#define P_PKEYPOINT_MRF_HH

#include <iostream>
#include <fstream>
#include <float.h>
#include <set>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "PKeypoint.hh"


namespace P
{

class PKeypointMRF : public PKeypoint
{
public:
  std::set<unsigned> links;
  std::vector<float> cost;                         // cost of assigning labels (data cost)
  std::vector<float> tmpCost;

  PKeypointMRF();
  PKeypointMRF(const PKeypoint &k);
  PKeypointMRF(const cv::KeyPoint &k);
  ~PKeypointMRF();

  inline void InsertLink(std::vector<PKeypointMRF> &keys, unsigned lid);
  inline void ReleaseLinks(std::vector<PKeypointMRF> &keys);
};





/*************************** INLINE METHODES **************************/
inline void PKeypointMRF::InsertLink(std::vector<PKeypointMRF> &keys, unsigned lid)
{
  links.insert(lid);
  keys[lid].links.insert(id);
}

inline void PKeypointMRF::ReleaseLinks(std::vector<PKeypointMRF> &keys)
{
  std::set<unsigned>::iterator it;

  for (it = links.begin(); it!=links.end(); it++)
    keys[(*it)].links.erase(id);

  links.clear();
}


} //--END--

#endif


/**
 * $Id$
 * Johann Prankl, 2010-06-30
 * prankl@acin.tuwien.ac.at
 */

#ifndef P_KEYPOINT_PAIR_HH
#define P_KEYPOINT_PAIR_HH

#include "Keypoint.hh"
#include "KeypointDescriptor.hh"

namespace P
{


class KeypointPair
{
public:
  Keypoint *k1;
  Keypoint *k2;

  KeypointPair() : k1(0), k2(0) {};
  KeypointPair(Keypoint *_k1, Keypoint *_k2) : k1(_k1), k2(_k2) {};
  ~KeypointPair(){};

  inline KeypointDescriptor* K1(){return (KeypointDescriptor*)k1;}
  inline KeypointDescriptor* K2(){return (KeypointDescriptor*)k2;}
};

}


#endif


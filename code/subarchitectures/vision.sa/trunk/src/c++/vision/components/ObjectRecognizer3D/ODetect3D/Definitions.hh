/**
 * $Id$
 */

#ifndef P_DEFINITIONS_HH
#define P_DEFINITIONS_HH

#include "PNamespace.hh"

//for debugging
#define DEBUG

#define NUM_THREADS 2



namespace P
{

class Def
{
public:
  static const int DO_MAX_RANSAC_TRIALS = 10000;
  static const bool DO_USE_SECOND_MATCH = true;//false;     // accept match second nearest match / threshold
  static const float DO_MIN_DESC_DIST = 8.;          // descriptor distance for object detection 
  static const double DO_RANSAC_ETA0 = 0.01;         //failure probability
  static const double DO_RANSAC_INL_DIST = 1.5; //1.;
  static const float DO_CLUSTER_SIGMA_THR=.2;         // cluster threshold for codebook
  static const double DO_TIME_MEAN = 5.;
};



}
#endif

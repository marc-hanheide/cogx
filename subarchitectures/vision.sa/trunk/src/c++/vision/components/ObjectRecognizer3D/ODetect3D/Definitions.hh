/**
 * $Id$
 */

#ifndef P_DEFINITIONS_HH
#define P_DEFINITIONS_HH

#include "PNamespace.hh"

//for debugging
//#define DEBUG

#define NUM_THREADS 2



namespace P
{

class Def
{
public:
  static const int DO_MAX_RANSAC_TRIALS = 1000;
  static const int DO_MATCHER = 2;     //0 = use match second nearest match, 1=threshold, 2=gpu
  static const float DO_MIN_DESC_DIST = 8.;          // descriptor distance for object detection 
  static const double DO_RANSAC_ETA0 = 0.01;         //failure probability
  static const double DO_RANSAC_INL_DIST = 2; //1.;
  static const float DO_CLUSTER_SIGMA_THR=0.2;         // cluster threshold for codebook
  static const double DO_TIME_MEAN = 5.;
  static const double DISTANCE_RATIO = 1.;   // least squares is only accepted if pose does not change very much
};



}
#endif

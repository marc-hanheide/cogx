/**
 * $Id$
 */

#ifndef P_LP_DEFINITIONS_HH
#define P_LP_DEFINITIONS_HH

#include "v4r/CEdge/PNamespace.hh"

//for debugging
#define DEBUG
#define DEBUG_SEG


namespace P
{
  //log polar segmentation
  static const unsigned LP_CANNY_HIGH = 150;
  static const bool LP_USE_COLOUR_EDGES = true;
  static const bool LP_USE_RGB = false;             //otherwise uv colour prior is used
  static const bool LP_USE_COLOUR_PRIOR = false;//true;
  static const bool LP_USE_WEIGHT_PRIOR = true;
  static const bool LP_USE_SQR_COST = true;
  static const bool LP_USE_MASK = true;
  static const float LP_SIGMA_MASK = 25.;//30.;       //we use a gaussian mask with ...
  static const float LP_WEIGHT_PRIOR = .5;
  static const float LP_WEIGHT_COLOUR = .5;      // only used if both priors are true;
  static const double LP_MAGNITUDE_SCALE = 50;      // for log polar images
  static const float LP_COST_PENALIZE = 20.;        // penalize if there is no edge
  static const float LP_MASK_PENALIZE = 50.;        // if LP_USE_WEIGHT_MASK = true
  static const float LP_COST_OFFSET = .2;    // no edge threshold
  static const float LP_MASK_OFFSET = .001;
  static const float PROB_FG_WEIGHT = 2.;           // we give a stronger weight to fg
  static const float LP_SAMPLE_DISTANCE = 2.;//5.;       // ... for LPSegment2



}
#endif

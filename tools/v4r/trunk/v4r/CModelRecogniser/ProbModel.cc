/**
 * $Id$
 * TODO:
 * - Compute prob values
 */


#include "ProbModel.hh"




namespace P 
{



/************************************************************************************
 * Constructor/Destructor
 */

ProbModel::ProbModel()
{ 
}

ProbModel::~ProbModel()
{
}

/**
 * Transform the confidence value to a learned probability
 */
double ProbModel::ConfToProb(const double conf)
{
  return (conf>1. ? 1. : conf);
}

/**
 * Prdict the probability of a TP recognition result with a changed view point
 * @param angle angle between learnded view point and view point to predict [rad]
 */
double ProbModel::PredictProbFromAngle(const double angle)
{
  return 1-angle;
}


}  // -- THE END --













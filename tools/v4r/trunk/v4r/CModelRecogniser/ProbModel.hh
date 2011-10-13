/**
 * $Id$
 */

#ifndef P_CONF_VALUES_HH
#define P_CONF_VALUES_HH

#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "v4r/PMath/PVector.hh"
#include "v4r/PMath/PMatrix.hh"


namespace P
{

class ProbModel
{
private:

public:

  ProbModel();
  ~ProbModel();

  double ConfToProb(double conf);
  double PredictProbFromAngle(double angle);
};





/*************************** INLINE METHODES **************************/


} //--END--

#endif


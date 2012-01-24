//
// = FILENAME
//    HSSMeas.hh
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef HSSMeas_hh
#define HSSMeas_hh

#include "Eigen/Core"

namespace HSS {

class Meas {
public:

  std::string description;

  int statepos;
  int refpos;
  Eigen::Vector3d Z;
  Eigen::Matrix3d R;

  double width; // For doors

  Eigen::Vector3d innov;
  double chi2;

public:

  Meas()
    :description(""),
     statepos(-1),
     refpos(-1),
     width(-1),
     chi2(-1)
  {
    Z.setZero();
    R.setZero();
    innov.setZero();
  }

  int getStatePos() const { return statepos; }
  int getStatePosX() const { return statepos+0; }
  int getStatePosY() const { return statepos+1; }
  int getStatePosTheta() const { return statepos+2; }

}; // class Meas

}; // namespace HSS

#endif // HSSMeas_hh

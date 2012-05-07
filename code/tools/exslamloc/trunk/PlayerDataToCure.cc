//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt
/*----------------------------------------------------------------------*/

#include "PlayerDataToCure.hh"

using namespace Cure;
using namespace PlayerCc;


Cure::Pose3D 
PlayerDataToCure::getPose(Position2dProxy &pp)
{
  Pose3D odo;
  odo.setTime(Cure::Timestamp(pp.GetDataTime()));
  odo.setX(pp.GetXPos());
  odo.setY(pp.GetYPos());
  odo.setTheta(pp.GetYaw());

  return odo;
}

Cure::SICKScan 
PlayerDataToCure::getScan(LaserProxy &lp)
{
  double ranges[722];
  memset(ranges, 0, 722*sizeof(double));

  for (int i = 0; i < lp.GetCount(); i++) {
    ranges[i] = lp.GetRange(i);
  }

  double angleStep = (lp.GetMaxAngle()-lp.GetMinAngle())/(lp.GetCount()-1);

  Cure::SICKScan scan;
  scan.import(Cure::Timestamp(lp.GetDataTime()),
              lp.GetCount(),     // number of points
              ranges,            // distances [m]
              angleStep,         // angleStep [rad]
              lp.GetMinAngle(),  // startAngle [rad]
              lp.GetMaxRange(),  // max range [m]
              0.001);            // scan resolution [m]

  return scan;
}


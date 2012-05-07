//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt
/*----------------------------------------------------------------------*/

#ifndef PlayerDataToCure_hh
#define PlayerDataToCure_hh

#include <SensorData/SICKScan.hh>
#include <Transformation/Pose3D.hh>
#include <libplayerc++/playerc++.h>

namespace Cure {
  namespace PlayerDataToCure {
    Cure::SICKScan getScan(PlayerCc::LaserProxy &lp);
    Cure::Pose3D getPose(PlayerCc::Position2dProxy &pp);
  };
};

#endif


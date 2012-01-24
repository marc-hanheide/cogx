//
// = FILENAME
//    
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef CureHWUtils_hpp
#define CureHWUtils_hpp

#include <string>

#include <SensorData/LaserScan2d.hh>
#include <Transformation/Pose3D.hh>

#include <Laser.hpp>
#include <Robotbase.hpp>

namespace CureHWUtils {

  void convScan2dToCure(const Laser::Scan2d &sIn, Cure::LaserScan2d &sOut);
  Cure::LaserScan2d convScan2dToCure(const Laser::Scan2d &scan);

  void convScan2dFromCure(const Cure::LaserScan2d &sIn, Laser::Scan2d &sOut);
  Laser::Scan2d convScan2dFromCure(const Cure::LaserScan2d &scan);

  void convOdomToCure(const Robotbase::Odometry &odIn, Cure::Pose3D &odOut);
  Cure::Pose3D convOdomToCure(const Robotbase::Odometry &odom);

  Robotbase::Odometry convOdomFromCure(const Cure::Pose3D &odom);
  void convOdomFromCure(const Cure::Pose3D &odIn, Robotbase::Odometry &odOut);

}; // namespace CureHWUtils

#endif // CureHWUtils_hpp

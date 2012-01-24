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

#include "CureHWUtils.hpp"

void
CureHWUtils::convScan2dToCure(const Laser::Scan2d &sIn,
                              Cure::LaserScan2d &sOut)
{
  if (sIn.ranges.empty()) {
    sOut.setTime(Cure::Timestamp(sIn.time.s, sIn.time.us));
    return;
  }

  double r[sIn.ranges.size()];
  for (unsigned int i = 0; i < sIn.ranges.size(); i++) r[i] = sIn.ranges[i];

  sOut.import(Cure::Timestamp(sIn.time.s, sIn.time.us),
             sIn.ranges.size(),
             r,
             sIn.angleStep,
             sIn.startAngle,
             sIn.maxRange,
             sIn.rangeRes);
}

Cure::LaserScan2d
CureHWUtils::convScan2dToCure(const Laser::Scan2d &scan)
{
  Cure::LaserScan2d ret;
  convScan2dToCure(scan, ret);
  return ret;
}

void
CureHWUtils::convScan2dFromCure(const Cure::LaserScan2d &sIn,
                                Laser::Scan2d &sOut)
{
  sOut.time.s = sIn.getTime().Seconds;
  sOut.time.us = sIn.getTime().Microsec;

  sOut.ranges.resize(sIn.getNPts());
  for (int i = 0; i < sIn.getNPts(); i++) {
    sOut.ranges[i] = sIn.getRange(i);
  }

  sOut.angleStep = sIn.getAngleStep();
  sOut.startAngle = sIn.getStartAngle();
  sOut.maxRange = sIn.getMaxRange();
  sOut.minRange = 0.04;
  sOut.rangeRes = sIn.getRangeResolution();
}

Laser::Scan2d 
CureHWUtils::convScan2dFromCure(const Cure::LaserScan2d &scan)
{
  Laser::Scan2d ret;
  convScan2dFromCure(scan, ret);
  return ret;

}

void
CureHWUtils::convOdomToCure(const Robotbase::Odometry &odIn,
                            Cure::Pose3D &odOut)
{
  odOut.setTime(Cure::Timestamp(odIn.time.s, odIn.time.us));

  if (!odIn.odompose.empty()) {
    odOut.setX(odIn.odompose[0].x);
    odOut.setY(odIn.odompose[0].y);
    odOut.setTheta(odIn.odompose[0].theta);
  }
}


Cure::Pose3D
CureHWUtils::convOdomToCure(const Robotbase::Odometry &odom)
{
  Cure::Pose3D ret;
  convOdomToCure(odom, ret);
  return ret;
}

void
CureHWUtils::convOdomFromCure(const Cure::Pose3D &odIn,
                              Robotbase::Odometry &odOut)
{
  odOut.time.s = odIn.getTime().Seconds;
  odOut.time.us = odIn.getTime().Microsec;

  odOut.odompose.resize(1);
  odOut.odompose[0].x = odIn.getX();
  odOut.odompose[0].y = odIn.getY();
  odOut.odompose[0].theta = odIn.getTheta();
}

Robotbase::Odometry
CureHWUtils::convOdomFromCure(const Cure::Pose3D &odom)
{
  Robotbase::Odometry ret;
  convOdomFromCure(odom, ret);
  return ret;
}

//
// = FILENAME
//    SlamProcessFake.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Nikolaus Demmel (based on SlamProcess by Chandana Paul, Patric Jensfelt)
//
// = COPYRIGHT
//    Copyright (c) 2012 Nikolaus Demmel
//                  2009 Patric Jensfelt
//                  2007 Chandana Paul, Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "SlamProcessFake.hpp"

#include <CureHWUtils.hpp>

#include <Map/FeatureData.hh>
#include <Geometry/Line2D.hh>
#include <Map/WrappedSLAM.hh>
#include <Map/RLDisplayFeatureMap.hh>
#include <Transformation/Pose3D.hh>
#include <AddressBank/ConfigFileReader.hh>
#include <Utils/HelpFunctions.hh>
#include <Utils/CureDebug.hh>
#include <SensorData/SensorData.hh>

using namespace cast;
using namespace std;
using namespace navsa;

extern "C" {
cast::interfaces::CASTComponentPtr newComponent() {
  return new SlamProcessFake();
}
}

SlamProcessFake::SlamProcessFake() {
}

SlamProcessFake::~SlamProcessFake() {
}

void SlamProcessFake::configure(const map<string, string>& config) {
  log("configure");

  setConfig(config);

  _mapFilename = parseOptionPath("-m", "tmpmap.metric");
  _configFilename = parseOptionPath("-c", "");

  if (_configFilename == "") {
    log("No config file configured");
  }

}

void SlamProcessFake::runComponent() {
  log("runComponent");

  ifstream tmp(_mapFilename.c_str());
  if (_mapFilename != "" && tmp.is_open()) {
    tmp.close();
    Cure::MapBank bank(10);
    Cure::FeatureMap map(&bank);
    Cure::MeasurementSet ms;
    Cure::ConfigFileReader cfg;
    if (!(cfg.init(_configFilename.c_str()) == 0 && map.config(cfg, ms) == 0)) {
      error("Failed to config with \"%s\"", _configFilename.c_str());
    } else {
      if (map.loadMap(_mapFilename) != 0) {
        error("Failed to load map \"%s\"", _mapFilename.c_str());
      } else {
        writeLineMapToWorkingMemory(map);
      }
    }
  }

  setupPushOdometry(*this, -1);
}

void SlamProcessFake::receiveOdometry(const Robotbase::Odometry &castOdom) {
  if (castOdom.odompose.empty()) {
    getLogger()->warn("Odometry struct contained no odompose which is needed");
    return;
  }

  NavData::RobotPose2dPtr p = new NavData::RobotPose2d;

  const Robotbase::Pose2d &opose = castOdom.odompose[0];

  p->time = castOdom.time;
  p->x = opose.x;
  p->y = opose.y;
  p->theta = opose.theta;

  updateRobotPoseInWM(p);
}

void SlamProcessFake::updateRobotPoseInWM(const NavData::RobotPose2dPtr &pose) {
  if (_robotPoseIdString == "") {
    _robotPoseIdString = newDataID();
    addToWorkingMemory<NavData::RobotPose2d> (_robotPoseIdString, pose);
    debug("Added RobotPose to WM");
  } else {
    overwriteWorkingMemory<NavData::RobotPose2d> (_robotPoseIdString, pose);
    debug("Overwriting RobotPose in WM x=%.3f y=%.3f theta=%.3f t=%ld.%06ld",
        pose->x, pose->y, pose->theta, pose->time.s, pose->time.us);
  }
}

void SlamProcessFake::writeLineMapToWorkingMemory(Cure::FeatureMap &fm) {
  log("writeLineMapToWorkingMemory");

  list<Cure::Line2D> walls;
  if (fm.getLine2DWalls(walls) != 0) {
    error("Failed to get linemap");
    return;
  }
  log("Has %d lines in the map", walls.size());

  NavData::LineMapPtr lineMap = new NavData::LineMap();
  lineMap->time = getCASTTime();
  if (walls.size() > 0) {
    for (list<Cure::Line2D>::iterator w = walls.begin(); w != walls.end(); ++w) {
      lineMap->lines.push_back(NavData::LineMapSegement());
      lineMap->lines.back().start.x = w->StartPoint.getX();
      lineMap->lines.back().start.y = w->StartPoint.getY();
      lineMap->lines.back().end.x = w->EndPoint.getX();
      lineMap->lines.back().end.y = w->EndPoint.getY();
    }
  }

  if ("" == _lineMapIdString) {
    _lineMapIdString = newDataID();
    addToWorkingMemory<NavData::LineMap> (_lineMapIdString, lineMap);
  } else {
    overwriteWorkingMemory<NavData::LineMap> (_lineMapIdString, lineMap);
  }

}


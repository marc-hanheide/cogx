//
// = FILENAME
//    CureDataLogger.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "CureDataLogger.hpp"
#include <Laser.hpp>
#include <Robotbase.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

// Cure includes
#include <SensorData/LaserScan2d.hh>
#include <Transformation/Pose3D.hh>

using namespace cast;
using namespace navsa;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new CureDataLogger();
  }
}

CureDataLogger::CureDataLogger()
{
  m_Odom = 0;
  m_Scan = 0;
  m_World = 0;

  Cure::Timestamp ct;
  ct.setToCurrentTime();

  char buf[32];
  sprintf(buf, "%ld", ct.Seconds);
  m_FilenameTime = buf;
}

CureDataLogger::~CureDataLogger()
{
}

void
CureDataLogger::configure(const std::map<std::string, std::string> &config)
{

}

void CureDataLogger::start()
{
  // Hook up changes to the robot pose to a callback function
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
                  new MemberFunctionChangeReceiver<CureDataLogger>(this,
                                        &CureDataLogger::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<CureDataLogger>(this,
                                        &CureDataLogger::newRobotPose));
}

void CureDataLogger::runComponent()
{
  setupPushScan2d(*this);
  setupPushOdometry(*this);
}

void CureDataLogger::newRobotPose(const cast::cdl::WorkingMemoryChange &objID)
{
  debug("newRobotPose called");

  boost::shared_ptr<cast::CASTData<NavData::RobotPose2d> > oobj =
    getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);

  double x = oobj->getData()->x;
  double y = oobj->getData()->y;
  double a = oobj->getData()->theta;

  Cure::Pose3D cp;
  cp.setTime(Cure::Timestamp(oobj->getData()->time.s,
                             oobj->getData()->time.us));
  cp.setX(x);
  cp.setY(y);
  cp.setTheta(a);

  // FIXME: Get the uncertainty and write world Pose3D to file

  if (m_World == 0) {
    m_World = new Cure::FileAddress;
    std::string filename = "worldpose-" + m_FilenameTime + ".tdf";
    if (m_World->openWriteFile(filename) != 0) {
      println("Failed to open worldpose file to write to");
      delete m_World;
    } else {
      log("Opened world pose file to write to \"%s\"", filename.c_str());
    }
  }

  if (m_World) {
    m_World->write(cp);
    std::flush(m_World->WriteFile);
  }
}

void
CureDataLogger::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  if (m_Odom == 0) {
    m_Odom = new Cure::FileAddress;
    std::string filename = "odom-" + m_FilenameTime + ".tdf";
    if (m_Odom->openWriteFile(filename) != 0) {
      println("Failed to open odometry file to write to");
      delete m_Odom;
    } else {
      log("Opened odom file to write to \"%s\"", filename.c_str());
    }
  }

  if (m_Odom) {
    m_Odom->write(cureOdom);
    std::flush(m_Odom->WriteFile);
  }
}


void
CureDataLogger::receiveScan2d(const Laser::Scan2d &castScan)
{
  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);

  if (m_Scan == 0) {
    m_Scan = new Cure::FileAddress;
    std::string filename = "scans-" + m_FilenameTime + ".tdf";
    if (m_Scan->openWriteFile(filename) != 0) {
      println("Failed to open scan file to write to");
      delete m_Scan;
    } else {
      log("Opened scan file to write to \"%s\"", filename.c_str());
    }
  }

  if (m_Scan) {
    m_Scan->write(cureScan);
    std::flush(m_Scan->WriteFile);
  }
}


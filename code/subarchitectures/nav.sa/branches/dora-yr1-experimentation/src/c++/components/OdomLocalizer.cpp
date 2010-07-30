//
// = FILENAME
//    OdomLocalizer.cpp
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

#include "OdomLocalizer.hpp"

#include <CureHWUtils.hpp>

// Cure include
#include <AddressBank/ConfigFileReader.hh>


using namespace cast;
using namespace std;
using namespace navsa;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new OdomLocalizer();
  }
}

OdomLocalizer::OdomLocalizer()
  : m_Interval(0.2),
    m_RobotPoseIdString(""),
    m_WriteInterval(1.0)
{}

OdomLocalizer::~OdomLocalizer() 
{
  if (m_DontWriteFiles) writeRobotPoseFile();
}

void OdomLocalizer::stop()
{
  if (m_DontWriteFiles) writeRobotPoseFile();
}

void OdomLocalizer::configure(const map<string,string>& _config) 
{
  map<string,string>::const_iterator it;

  it = _config.find("--interval");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_Interval;
  }
  println("Will try to output RobotPose at every %.2f second",
          m_Interval);

  m_DontWriteFiles = (_config.find("--no-file-output") != _config.end());

  it = _config.find("--robotpose-file");
  if (it != _config.end()) {
    Cure::ConfigFileReader cfg;
    std::string cfgFile(it->second);
    if (cfg.init(cfgFile) == 0) {
      if (cfg.getRobotPose(m_Pose)){
        println("A robotpose file was specified \"%s\", but no robotpose was found. EXITING", cfgFile.c_str());
        std::abort();
      }
    } else {
      println("Could not open the robotpose file, starting in 0,0,0");
    }
  } else {
    println("No robotpose file specified, starts in 0,0,0");
  }

  println("Initial pose of the robot x=%.2f y=%.2f theta=%.2fdeg",
          m_Pose.getX(), m_Pose.getY(), m_Pose.getTheta()*180/M_PI);

  m_GotFirstOdom = false;
}

void OdomLocalizer::runComponent() 
{  
  setupPushOdometry(*this, m_Interval);
  log("I am running!");
}

void OdomLocalizer::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  if (castOdom.odompose.empty()) {
    println("WARNING: Odometry struct contained no odompose which is needed");
    return;
  }

  const Robotbase::Pose2d &p = castOdom.odompose[0];

  debug("Got odometry x=%.3f, y=%.3f, theta=%.3f t=%ld.%06ld",
        p.x, p.y, p.theta, (long)castOdom.time.s, (long)castOdom.time.us);

  Cure::Pose3D odom;
  CureHWUtils::convOdomToCure(castOdom, odom);
  
  if (!m_GotFirstOdom) {
    m_GotFirstOdom = true;
  } else {
    
    // Calculate the difference in odometry, ie the motion
    Cure::Pose3D diff;
    diff.minusPlus(m_LastOdom, odom);
    
    // Calculate the new robot pose
    m_Pose.add(m_Pose, diff);
  }
  m_LastOdom = odom;

  // Write estimated robot pose data into robot pose IDL structure
  NavData::RobotPose2dPtr pRP = new NavData::RobotPose2d;
  pRP->time.s = castOdom.time.s;
  pRP->time.us = castOdom.time.us;
  pRP->x = m_Pose.getX();
  pRP->y = m_Pose.getY();
  pRP->theta = m_Pose.getTheta();

  if(m_RobotPoseIdString == ""){
    m_RobotPoseIdString = newDataID();
    addToWorkingMemory<NavData::RobotPose2d>(m_RobotPoseIdString, 
                                             pRP); // sync!
    debug("Added RobotPose to WM");
  }else{
    overwriteWorkingMemory<NavData::RobotPose2d>(m_RobotPoseIdString, 
                                                 pRP);
    char buf[256];
    sprintf(buf, "Overwriting RobotPose in WM x=%.3f y=%.3f theta=%.3f t=%ld.%06ld",
            p.x, p.y, p.theta, (long)castOdom.time.s,
            (long)castOdom.time.us);
    debug(buf);
  }

  if (!m_DontWriteFiles &&
      (!m_WriteTimer.isRunning() ||
       m_WriteTimer.split() > m_WriteInterval)) {
    writeRobotPoseFile();
  }
}

void OdomLocalizer::writeRobotPoseFile()
{
  std::fstream fsl;
  fsl.open("robotpose.ccf", std::ios::out);
  if (fsl > 0) {
    double ang[3];
    m_Pose.getAngles(ang);
    fsl << "ROBOTPOSE\n";
    fsl << m_Pose.getX() << " "
        << m_Pose.getY() << " "
        << m_Pose.getZ() << " "
        << ang[0] << " "
        << ang[1] << " "
        << ang[2] << std::endl;
  }
  fsl.close();
}

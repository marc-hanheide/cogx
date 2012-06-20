//
// = FILENAME
//    LaserRobotServerDummy.cpp
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

#include "LaserRobotServerDummy.hpp"

using namespace Robotbase;
using namespace Laser;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new LaserRobotServerDummy();
  }
}

LaserRobotServerDummy::LaserRobotServerDummy()
{
  m_MinPushInterval = -1;

  m_RobotServerName = "RobotbaseServer";
  m_LaserServerName = "LaserServer";
}

LaserRobotServerDummy::~LaserRobotServerDummy()
{}

void 
LaserRobotServerDummy::configure(const std::map<std::string,std::string> & config)
{
  println("configure");

  std::map<std::string,std::string>::const_iterator it;

  if ((it = config.find("--robot-server-name")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerName;
  }
  log("Using m_RobotServerName=%s", m_RobotServerName.c_str());

  if ((it = config.find("--laser-server-name")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_LaserServerName;
  }
  log("Using m_LaserServerName=%s", m_LaserServerName.c_str());

  if ((it = config.find("--min-push-interval")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_MinPushInterval;
  }
  log("Using m_MinPushInterval=%fs", m_MinPushInterval);

  // Start the interface that allows clients to connect to this server
  // the Ice way
  try {

    Ice::Identity id;
    id.name = m_LaserServerName;
    id.category = "LaserServer";
    
    println("name: " + id.name);
    println("category: " + id.category);

    //add as a separate object
    getObjectAdapter()->add(this, id);

    println("server registered");
    
  } catch (const Ice::Exception& e) {
    std::cerr << e << std::endl;
  } catch (const char* msg) {
    std::cerr << msg << std::endl;
  }

  // Start the interface that allows clients to connect to this server
  // the Ice way
  try {

    Ice::Identity id;
    id.name = m_RobotServerName;
    id.category = "RobotbaseServer";
    
    println("name: " + id.name);
    println("category: " + id.category);

    getObjectAdapter()->addFacet(new RobotServerHelper(this), 
                                 getIceIdentity(), 
                                 "RobotbaseServer"); 

    println("server registered");
    
  } catch (const Ice::Exception& e) {
    std::cerr << e << std::endl;
  } catch (const char* msg) {
    std::cerr << msg << std::endl;
  }


}

void 
LaserRobotServerDummy::start() 
{
  println("start");
}
  
void
LaserRobotServerDummy::stop()
{}

void 
LaserRobotServerDummy::runComponent()
{
  println("runComponent started");


  while (isRunning()) {

    Robotbase::Pose2d p;
    p.x = 1.1;
    p.y = 1.2;
    p.theta = 1.3;
    
    m_Odom.odompose.clear();
    m_Odom.odompose.push_back(p);

    int n = 361;
    m_Scan.ranges.resize(n);
    for (unsigned int i = 0; i < m_Scan.ranges.size(); i++) {
      m_Scan.ranges[i] = 5;
    }
    m_Scan.startAngle = -M_PI_2;
    m_Scan.angleStep = M_PI / (m_Scan.ranges.size() - 1);
    m_Scan.maxRange = 8;
    m_Scan.minRange = 0.01;
    m_Scan.rangeRes = 0.001;

    m_Odom.time = m_Scan.time = getCASTTime();
    
    for (unsigned int i = 0; i < m_PushOdomClients.size(); i++)  {
      println("pushed odom");
      m_PushOdomClients[i].prx->receiveOdometry(m_Odom);
    }

    for (unsigned int i = 0; i < m_PushScanClients.size(); i++)  {
      println("pushed scan");
      m_PushScanClients[i].prx->receiveScan2d(m_Scan);
    }

    sleepComponent(50);
  }
}

Robotbase::Odometry
LaserRobotServerDummy::pullOdometry()
{
  println("pullOdometry");
  return m_Odom;
}

void 
LaserRobotServerDummy::execMotionCommand(const ::Robotbase::MotionCommand& cmd)
{
  println("execMotionCommand (IGNORED) v=%.2fm/s w=%.3frad/s",
              cmd.speed, cmd.rotspeed);    
}
  

void
LaserRobotServerDummy::registerOdometryPushClient(const Robotbase::OdometryPushClientPrx& c, 
                                                  Ice::Double desiredFreq)
{
  println("registerScan2dPushClient");

  OdometryClient client;
  client.prx = c;
  client.freq = desiredFreq;
  m_PushOdomClients.push_back(client);
}

Laser::Scan2d 
LaserRobotServerDummy::pullScan2d(const Ice::Current&)
{
  println("pullScan2d");
  return m_Scan;
}

void
LaserRobotServerDummy::registerScan2dPushClient(const Laser::Scan2dPushClientPrx& c, 
                                            Ice::Double desiredFreq, 
                                            const Ice::Current&)
{
  println("registerScan2dPushClient");

  Scan2dClient client;
  client.prx = c;
  client.freq = desiredFreq;
  m_PushScanClients.push_back(client);
}


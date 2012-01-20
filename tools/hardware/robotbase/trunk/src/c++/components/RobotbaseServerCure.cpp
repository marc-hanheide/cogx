//
// = FILENAME
//    RobotbaseServerCure.cpp
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

#include "RobotbaseServerCure.hpp"

using namespace Robotbase;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new RobotbaseServerCure();
  }
}

RobotbaseServerCure::RobotbaseServerCure()
{
  m_MinPushInterval = -1;

  m_MotionCmdGiven = false;
  m_MotionCmd.speed = 0;
  m_MotionCmd.rotspeed = 0;
}

RobotbaseServerCure::~RobotbaseServerCure()
{}

void 
RobotbaseServerCure::configure(const std::map<std::string,std::string> & config)
{
  println("configure");

  std::map<std::string,std::string>::const_iterator it;

  if ((it = config.find("--min-push-interval")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_MinPushInterval;
  }
  log("Using m_MinPushInterval=%fs", m_MinPushInterval);

  if ((it = config.find("--cure-config-file")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_CfgFile;
  }
  log("Using m_CfgFile=%s", m_CfgFile.c_str());

  JoystickDrivable::configure(config);

  registerIceServer<cast::CASTComponent, RobotbaseServer>(getComponentPointer());
}

void 
RobotbaseServerCure::start() 
{
  println("start in");

  if (m_P2OSDevice != 0) {
    m_P2OSDevice->disconnect();    
    delete m_P2OSDevice;
  }

  m_P2OSDevice = new Cure::Hal::P2OSSerialInterface;

  if (m_P2OSDevice->config(m_CfgFile) != 0) {
    println("\n\nERROR: Could not configure with \"%s\"\n\n",
           m_CfgFile.c_str());           
    std::exit(1);
  }

  if (m_P2OSDevice->connect()) {
   println("\n\nERROR: Failed to connect to P2OS\n\n");
   std::exit(1);
  }

  println("Successfully started");
}
  
void
RobotbaseServerCure::stop()
{  
  println("stop");

  if (m_P2OSDevice != 0) {
    
    Cure::Hal::P2OSMsgMotionCmd cmd(0,0);
    m_P2OSDevice->sendToDevice(cmd);
    
    m_P2OSDevice->disconnect();
    delete m_P2OSDevice;
    m_P2OSDevice = 0;
  }
}

void 
RobotbaseServerCure::runComponent()
{
  println("runComponent started");

  while (isRunning()) {

    bool gotOdom = false;

    // Wait for new data from the robot
    int ret = m_P2OSDevice->waitSerialBuffer(1.0);

    if (ret == 1) {

      // We got data
      if (m_P2OSDevice->readSerialBuffer() >= 0) {
        bool first = true;
        
        int ret = 0;
        while((ret = m_P2OSDevice->processRecvBuffer(first))) {
          first = false;
          if (ret > 0) {
            switch (m_P2OSDevice->getLastMsgID()) {
            case Cure::Hal::P2OSMsgSIP_ID:

              gotOdom = true;
              
              Cure::Pose3D tmp(m_P2OSDevice->getOdometry());
              Robotbase::Pose2d p;
              p.x = tmp.getX();
              p.y = tmp.getY();
              p.theta = tmp.getTheta();
              
              m_Odom.odompose.clear();
              m_Odom.odompose.push_back(p);
              
              // Fixme, get better estimate of the timestamp
              m_Odom.time = getCASTTime();

              if (m_MotionCmdGiven) {
                // Send motion command as well
                Cure::Hal::P2OSMsgMotionCmd cmd(m_MotionCmd.speed, 
                                                m_MotionCmd.rotspeed);
                if (m_Joydrive) {
                  if (m_JoydriveRotOnly) {
                    cmd.setTransVel(0);
                    cmd.setSteerVel(m_JoyW);
                  } else if (m_JoydriveTransOnly) {
                    cmd.setTransVel(m_JoyV);
                    cmd.setSteerVel(0);
                  } else {
                    cmd.setTransVel(m_JoyV);
                    cmd.setSteerVel(m_JoyW);
                  }
                }
                m_P2OSDevice->sendToDevice(cmd);
                break;
              }
            }
          }
        }
      } else if (ret == -1) {
        println("\n\n\n ERROR CureHalRobotbaseDevice::serviceDevice stopping device\n\n\n");
        
        if (m_P2OSDevice != 0) {
          Cure::Hal::P2OSMsgMotionCmd cmd(0,0);
          m_P2OSDevice->sendToDevice(cmd);
          
          m_P2OSDevice->disconnect();
          delete m_P2OSDevice;
          m_P2OSDevice = 0;
        }
      
        break;
      }
      
      if (gotOdom) {

        for (unsigned int i = 0; i < m_PushClients.size(); i++)  {          

          if ( isRunning() && 
               (!m_PushClients[i].timer.isRunning() ||
                (m_PushClients[i].timer.split() >= m_PushClients[i].interval) ) ) {
            debug("pushed odom to client %d", i);    
            m_PushClients[i].timer.restart();
            m_PushClients[i].prx->receiveOdometry(m_Odom);
          }          
        }

      }
    }
  }
}

Robotbase::Odometry
RobotbaseServerCure::pullOdometry(const Ice::Current&)
{
  println("pullOdometry");
  return m_Odom;
}

void 
RobotbaseServerCure::execMotionCommand(const ::Robotbase::MotionCommand& cmd,
                                         const Ice::Current&)
{
  println("execMotionCommand v=%.2fm/s w=%.3frad/s",cmd.speed,cmd.rotspeed);
  m_MotionCmdGiven = true;
  m_MotionCmd = cmd;
}
  

void
RobotbaseServerCure::registerOdometryPushClient(const Robotbase::OdometryPushClientPrx& c, 
                                                Ice::Double desiredInterval, 
                                                const Ice::Current&)
{
  println("registerScan2dPushClient");

  OdometryClient client;
  client.prx = c;
  client.interval = desiredInterval;
  m_PushClients.push_back(client);
}


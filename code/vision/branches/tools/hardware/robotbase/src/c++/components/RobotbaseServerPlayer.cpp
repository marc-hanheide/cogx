//
// = FILENAME
//    RobotbaseServerPlayer.cpp
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

#include "RobotbaseServerPlayer.hpp"

#include <CASTUtils.hpp>

using namespace Robotbase;
using namespace cast;
using namespace cast::cdl;


/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new RobotbaseServerPlayer();
  }
}

RobotbaseServerPlayer::RobotbaseServerPlayer()
{
  m_PlayerClient = 0;
  m_Position = 0;

  m_PlayerHost = "localhost";
  m_PlayerPort = PlayerCc::PLAYER_PORTNUM;
  m_PlayerPosDeviceId = 0;

  m_EnableMotors = true;

  m_MinPushInterval = -1;

  m_IceServerName = "RobotbaseServer";

  m_RandData = false;
}

RobotbaseServerPlayer::~RobotbaseServerPlayer()
{}

void 
RobotbaseServerPlayer::configure(const std::map<std::string,std::string> & config)
{
  log("configure");

  std::map<std::string,std::string>::const_iterator it;

  if ((it = config.find("--server-name")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_IceServerName;
  }
  log("Using m_IceServerName=%s", m_IceServerName.c_str());

  if ((it = config.find("--min-push-interval")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_MinPushInterval;
  }
  log("Using m_MinPushInterval=%fs", m_MinPushInterval);

  if ((it = config.find("--player-host")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_PlayerHost;
  }
  log("Using m_PlayerHost=%s", m_PlayerHost.c_str());

  if ((it = config.find("--enable-motors")) != config.end()) {
    std::istringstream str(it->second);
    m_EnableMotors = true;
  }
  if ((it = config.find("--disable-motors")) != config.end()) {
    std::istringstream str(it->second);
    m_EnableMotors = false;
  }
  log("Using m_EnableMotors=%d", (int)m_EnableMotors);

  if ((it = config.find("--rand-data")) != config.end()) {
    m_RandData = true;
  }
  log("Using m_RandData=%d", (int)m_RandData);

  JoystickDrivable::configure(config);

  // Start the interface that allows clients to connect to this server
  // the Ice way
  try {

    Ice::Identity id;
    id.name = m_IceServerName;
    id.category = "RobotbaseServer";
    
    //add as a separate object
    getObjectAdapter()->add(this, id);

    log("server registered");
    
  } catch (const Ice::Exception& e) {
    std::cerr << e << std::endl;
  } catch (const char* msg) {
    std::cerr << msg << std::endl;
  }
}

void 
RobotbaseServerPlayer::start() 
{
  log("start");
  if (!m_RandData) {
    m_PlayerClient = new PlayerCc::PlayerClient(m_PlayerHost, m_PlayerPort);

    m_PlayerClient->SetDataMode(PLAYER_DATAMODE_PULL);
    m_PlayerClient->SetReplaceRule(true, PLAYER_MSGTYPE_DATA);

    m_Position = new PlayerCc::Position2dProxy(m_PlayerClient, 
                                               m_PlayerPosDeviceId);

    if (m_EnableMotors) m_Position->SetMotorEnable(true);
  }
}
  
void
RobotbaseServerPlayer::stop()
{}

void 
RobotbaseServerPlayer::runComponent()
{
  log("runComponent started");


  bool joyDriveState = false;
  while (isRunning()) {

    if (!m_RandData) {
 
      m_PlayerClient->Read();
            
      Robotbase::Pose2d p;
      p.x = m_Position->GetXPos();
      p.y = m_Position->GetYPos();
      p.theta = m_Position->GetYaw();
      
      m_Odom.odompose.resize(1);
      m_Odom.odompose[0] = p;
      
      // Fixme, get better estimate of the timestamp
      /*
      CASTTime timeNow;
      timeNow = getCASTTime();

      Cure::Timestamp timeNowCure(timeNow.s, timeNow.us);
      m_TimeFilter.addNewUnfilteredTimestamp(timeNowCure.getDouble());
      Cure::Timestamp timeNowCureFilt(m_TimeFilter.getEstimatedTimestamp());

      CASTTime timeNowFilt;
      timeNowFilt.s = timeNowCureFilt.Seconds;
      timeNowFilt.us = timeNowCureFilt.Microsec;

      m_Odom.time = m_TimeOffset - timeNowFilt;
      */
      m_Odom.time = getCASTTime();

      if (m_Joydrive) {
        if (m_JoydriveRotOnly) {
          m_Position->SetSpeed(0, m_JoyW);
        } else if (m_JoydriveTransOnly) {
          m_Position->SetSpeed(m_JoyV, 0);
        } else {
          m_Position->SetSpeed(m_JoyV, m_JoyW);
        }
        
      } else if (joyDriveState) {
        // Was in joydrive state last iteration, should send stop
        m_Position->SetSpeed(0,0);
      }
      joyDriveState = m_Joydrive;
    } else {
      m_Odom.odompose.resize(1);
      m_Odom.odompose[0].x += 0.01;
      m_Odom.time = getCASTTime();
      sleepComponent(100);
    }

    for (unsigned int i = 0; i < m_PushClients.size(); i++)  {

      //nah: added isRunning check to prevent hang on CAST stop()
      if ( isRunning() && (!m_PushClients[i].timer.isRunning() ||
			   m_PushClients[i].timer.split() >= m_PushClients[i].interval )) {
        debug("pushed odom to client %d", i);    
        m_PushClients[i].timer.restart();
        m_PushClients[i].prx->receiveOdometry(m_Odom);
      }

    }

  }

}

Robotbase::Odometry
RobotbaseServerPlayer::pullOdometry(const Ice::Current&)
{
  debug("pullOdometry");
  return m_Odom;
}

void 
RobotbaseServerPlayer::execMotionCommand(const ::Robotbase::MotionCommand& cmd,
                                         const Ice::Current&)
{
  if (m_Position && !m_RandData) {
    if (m_EnableMotors) {
      debug("execMotionCommand v=%.2fm/s w=%.3frad/s",cmd.speed,cmd.rotspeed);
      if (!m_Joydrive) {
        m_Position->SetSpeed(cmd.speed, cmd.rotspeed);
      }
    } else  {
      debug("execMotionCommand (DISABLED MOTORS) v=%.2fm/s w=%.3frad/s",
              cmd.speed, cmd.rotspeed);
    }
  } else {
      debug("execMotionCommand (NOT CONNECTED) v=%.2fm/s w=%.3frad/s",
              cmd.speed, cmd.rotspeed);    
  }
}
  

void
RobotbaseServerPlayer::registerOdometryPushClient(const Robotbase::OdometryPushClientPrx& c, 
                                                Ice::Double desiredInterval, 
                                                const Ice::Current&)
{
  println("registerScan2dPushClient");

  OdometryClient client;
  client.prx = c;
  client.interval = desiredInterval;
  m_PushClients.push_back(client);
}


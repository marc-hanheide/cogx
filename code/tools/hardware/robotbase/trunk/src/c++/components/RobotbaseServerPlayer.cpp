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

#include <cast/core/CASTUtils.hpp>

#include <Utils/HelpFunctions.hh>

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

  // Initialize with no limts really
  m_MaxV = 10;
  m_MaxW = 10;
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

  if ((it = config.find("--max-v")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_MaxV;
  }
  log("Using m_MaxV=%.2fm/s", m_MaxV);

  if ((it = config.find("--max-w")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_MaxW;
  }
  log("Using m_MaxW=%.2frad/s", m_MaxW);

  if ((it = config.find("--rand-data")) != config.end()) {
    m_RandData = true;
  }

  m_OverRideJoystickEnable = false;
  if ((it = config.find("--no-joystick")) != config.end()) {
    m_OverRideJoystickEnable = true;
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

  //FIXME a really big hack to see if we're running in simulation or not      
  bool useWallclockTimestamps = true;
  const cast::interfaces::TimeServerPrx timeServer(getTimeServer());
  if(isRunning()) {
    m_PlayerClient->Read();
    
    //timestamp on the data as an int
    long timestamp = (long) m_Position->GetDataTime();
    
    //current time
    timeval tvNow;  
    gettimeofday(&tvNow, NULL);  
    
    //compare the difference in timestamps
    long secDiff = abs(tvNow.tv_sec - timestamp);
    //we can be quite loose here as we're either going to be very close or a long way off
    if(secDiff > 10) {
      //if the timestamps are a long way apart then we're probably running on a monotonic clock from stage
      useWallclockTimestamps = false;             
      log("running in simulation, not attempting to sync timestamps");
    }
    else {
      log("trying to synchronise using current time of day");
    }               
  }
  

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
      
      //if we're guessing that the player timestamp is based on gettimeofday
      if(useWallclockTimestamps) {
	//create a CAST time from the double value      
	m_Odom.time = timeServer->fromTimeOfDayDouble(m_Position->GetDataTime());	
// 	ostringstream outStream;
// 	outStream<<"player cast time: "<<m_Odom.time<<endl;
// 	outStream<<"current cast time: "<<getCASTTime();
// 	println(outStream.str());	
      }
      //else we have no idea!
      else {
	// FIXME get a better estimate of the time for the scan
	//CASTTime timeNow(getCASTTime());
	m_Odom.time = getCASTTime(); //m_TimeOffset - timeNow;
      }


      // alper: to disable Joystick, in some laptops with internal accelerometer
      // causes to be seen as joystick and produces weird results.
      if (m_Joydrive && !m_OverRideJoystickEnable) {
        if (m_JoydriveRotOnly) {
          m_Position->SetSpeed(0, m_JoyW);
        } else if (m_JoydriveTransOnly) {
          m_Position->SetSpeed(m_JoyV, 0);
        } else {
          m_Position->SetSpeed(m_JoyV, m_JoyW);
        }
        
      } else if (joyDriveState && !m_OverRideJoystickEnable) {
        // Was in joydrive state last iteration, should send stop
        m_Position->SetSpeed(0,0);
      }
      joyDriveState = m_Joydrive;
    } else {
      m_Odom.odompose.resize(1);
      m_Odom.odompose[0].x += 0.01;
      //nah: removed this... why the extra assignment without offset?
      //      m_Odom.time = getCASTTime();
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

      double v = cmd.speed;
      double w = cmd.rotspeed;     

      Cure::HelpFunctions::limitAndSetValueSymm(v, m_MaxV);
      Cure::HelpFunctions::limitAndSetValueSymm(w, m_MaxW);

      debug("execMotionCommand v=%.2fm/s w=%.3frad/s (got v=%.2fm/s w=%.3frad/s",v,w,cmd.speed,cmd.rotspeed);
      if (!m_Joydrive) {
        m_Position->SetSpeed(v,w);
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


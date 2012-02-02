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
#include <fstream>
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

  m_RandData = false;

  // Initialize with no limts really
  m_MaxV = 10;
  m_MaxW = 10;
  m_MaxJoyV = 10;
  m_MaxJoyW = 10;
}

RobotbaseServerPlayer::~RobotbaseServerPlayer()
{}

void 
RobotbaseServerPlayer::configure(const std::map<std::string,std::string> & config)
{
  log("configure");

  std::map<std::string,std::string>::const_iterator it;

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

  if ((it = config.find("--max-joyv")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_MaxJoyV;
  }

  log("Using m_MaxJoyV=%.2fm/s", m_MaxJoyV);

  if ((it = config.find("--max-joyw")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_MaxJoyW;
  }
  log("Using m_MaxJoyW=%.2frad/s", m_MaxJoyW);




  if ((it = config.find("--rand-data")) != config.end()) {
    m_RandData = true;
  }

  m_saveToFile = false;
  if ((it = config.find("--save-to-file")) != config.end()) {
    m_saveToFile = true;

    if ((it = config.find("--save-directory")) != config.end()) {
    	std::istringstream str(it->second);
     str >> m_saveDirectory;
 	//check if the last char is / if yes remove
     if (m_saveDirectory[m_saveDirectory.size()-1] == '/'){
    	 m_saveDirectory.erase(m_saveDirectory.size()-1);
     }
     log("Will be saving scans to %s", m_saveDirectory.c_str());
    }
    else
    {
    	log("You haven't specified a save directory! (usage: --save-directory)");
    	abort();
    }
  }



  m_OverRideJoystickEnable = false;
  if ((it = config.find("--no-joystick")) != config.end()) {
    m_OverRideJoystickEnable = true;
  }
  log("Using m_RandData=%d", (int)m_RandData);

  Robotbase::RobotbaseServerPtr servant = new RobotbaseServerI(this);
//  registerIceServer<cast::CASTComponent, RobotbaseServer>(getComponentPointer());
  registerIceServer<RobotbaseServer, RobotbaseServer>(servant);
  JoystickDrivable::configure(config);
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
      
      lockComponent();
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
        Cure::HelpFunctions::limitAndSetValueSymm(m_JoyV, m_MaxJoyV);
        Cure::HelpFunctions::limitAndSetValueSymm(m_JoyW, m_MaxJoyW);


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
      unlockComponent();
    } else {
      lockComponent();
      m_Odom.odompose.resize(1);
      m_Odom.odompose[0].x += 0.01;
      unlockComponent();
      //nah: removed this... why the extra assignment without offset?
      //      m_Odom.time = getCASTTime();
      sleepComponent(100);
    }
    saveOdomToFile(m_Odom);
    lockComponent();
    for (unsigned int i = 0; i < m_PushClients.size(); i++)  {

      //nah: added isRunning check to prevent hang on CAST stop()
      if ( isRunning() && (!m_PushClients[i].timer.isRunning() ||
			   m_PushClients[i].timer.split() >= m_PushClients[i].interval )) {
        log("pushed odom to client %d", i);    
        m_PushClients[i].timer.restart();
        m_PushClients[i].prx->receiveOdometry(m_Odom);
      }

    }
    unlockComponent();

  }

}


void RobotbaseServerPlayer::saveOdomToFile(Robotbase::Odometry odom){
	
	// NOTE: the speed and encoder fields are not set so we don't save them
	if (odom.odompose.size() == 0)
		return;
  
         char buf[256];
         sprintf(buf,"%s/odom_%ld_%ld", m_saveDirectory.c_str(), (long int)odom.time.s, (long int)odom.time.us);
         std::ofstream odomfile;

         odomfile.open (buf);
         odomfile << odom.odompose[0].x << " " << odom.odompose[0].y
        		 << " " << odom.odompose[0].theta;
         odomfile << std::endl;
         odomfile.close();
}

Robotbase::Odometry
RobotbaseServerPlayer::RobotbaseServerI::pullOdometry(const Ice::Current&)
{
  log("pullOdometry");
  return svr->m_Odom;
}

void 
RobotbaseServerPlayer::RobotbaseServerI::execMotionCommand
  (const ::Robotbase::MotionCommand& cmd, const Ice::Current&)
{
  svr->execMotionCommand(cmd);
}

void
RobotbaseServerPlayer::execMotionCommand(const ::Robotbase::MotionCommand& cmd)
{
  if (m_Position && !m_RandData) {
    if (m_EnableMotors) {

      double v = cmd.speed;
      double w = cmd.rotspeed;     

      Cure::HelpFunctions::limitAndSetValueSymm(v, m_MaxV);
      Cure::HelpFunctions::limitAndSetValueSymm(w, m_MaxW);

      log("execMotionCommand v=%.2fm/s w=%.3frad/s (got v=%.2fm/s w=%.3frad/s",v,w,cmd.speed,cmd.rotspeed);
      if (!m_Joydrive) {
        m_Position->SetSpeed(v,w);
      }
    } else  {
      log("execMotionCommand (DISABLED MOTORS) v=%.2fm/s w=%.3frad/s",
              cmd.speed, cmd.rotspeed);
    }
  } else {
      log("execMotionCommand (NOT CONNECTED) v=%.2fm/s w=%.3frad/s",
              cmd.speed, cmd.rotspeed);    
  }
}
  

void
RobotbaseServerPlayer::RobotbaseServerI::registerOdometryPushClient
(const Robotbase::OdometryPushClientPrx& c, 
 Ice::Double desiredInterval, 
 const Ice::Current&)
{
  println("registerScan2dPushClient");

  OdometryClient client;
  client.prx = c;
  client.interval = desiredInterval;

  svr->lockComponent();
  svr->m_PushClients.push_back(client);
  svr->log("Added push client %i", svr->m_PushClients.size()-1);
  svr->unlockComponent();
}


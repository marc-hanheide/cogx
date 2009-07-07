#include "SonarServerPlayer.hpp"

#include <cstdlib>
#include <sstream>

#include <cast/core/CASTUtils.hpp>

using namespace Sonar;
using namespace cast;
using namespace cast::cdl;
using namespace std;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new SonarServerPlayer();
  }
}

SonarServerPlayer::SonarServerPlayer()
{
  m_PlayerClient = 0;
  m_Sonar = 0;

  m_PlayerHost = "localhost";
  m_PlayerPort = PlayerCc::PLAYER_PORTNUM;
  m_PlayerPosDeviceId = 0;

//   m_MinPushInterval = -1;

}

SonarServerPlayer::~SonarServerPlayer()
{}

void 
SonarServerPlayer::configure(const std::map<std::string,std::string> & config)
{
  //  println("configure");

  std::map<std::string,std::string>::const_iterator it;

//   if ((it = config.find("--min-push-interval")) != config.end()) {
//     std::istringstream str(it->second);
//     str >> m_MinPushInterval;
//   }
//   log("Using m_MinPushInterval=%fs", m_MinPushInterval);

  m_TimeOffset.s = 0;
  m_TimeOffset.us = 0;
  if ((it = config.find("--time-offset")) != config.end()) {
    std::istringstream str(it->second);
    double tmp;
    str >> tmp;

    m_TimeOffset.s = (long)tmp;
    m_TimeOffset.us = (long)(1e6*(tmp - m_TimeOffset.s));
  }
  log("Using m_TimeOffset=%ld.%06lds", 
      (long)m_TimeOffset.s, (long)m_TimeOffset.us);

  if ((it = config.find("--player-host")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_PlayerHost;
  }
  log("Using m_PlayerHost=%s", m_PlayerHost.c_str());


}

void 
SonarServerPlayer::start() 
{
  
  m_PlayerClient = new PlayerCc::PlayerClient(m_PlayerHost, m_PlayerPort);
  
  m_PlayerClient->SetDataMode(PLAYER_DATAMODE_PULL);
  m_PlayerClient->SetReplaceRule(true, PLAYER_MSGTYPE_DATA);
  
  m_Sonar = new PlayerCc::SonarProxy(m_PlayerClient, m_PlayerPosDeviceId);

  //register server using new cast method
  registerIceServer<cast::CASTComponent,SonarServer>(getComponentPointer());

}

void
SonarServerPlayer::stop() {
  delete m_Sonar;
  m_Sonar = NULL;
  delete m_PlayerClient;
  m_PlayerClient = NULL;
}

void 
SonarServerPlayer::runComponent()
{
  //FIXME a really big hack to see if we're running in simulation or not      
  bool useWallclockTimestamps = true;
  const cast::interfaces::TimeServerPrx timeServer(getTimeServer());
  if(isRunning()) {
    m_PlayerClient->Read();
    
    //timestamp on the data as an int
    long timestamp = (long) m_Sonar->GetDataTime();
    
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
  
    

  while (isRunning()) {

    //must be locked while reading data
    lockComponent();

    m_PlayerClient->Read();
    
    //if we're guessing that the player timestamp is based on gettimeofday
    if(useWallclockTimestamps) {
      //create a CAST time from the double value      
      m_Scan.time = timeServer->fromTimeOfDayDouble(m_Sonar->GetDataTime());	
      // 	ostringstream outStream;
      // 	outStream<<"player cast time: "<<m_Scan.time<<endl;
      // 	outStream<<"current cast time: "<<getCASTTime();
      // 	println(outStream.str());	
    }
    //else we have no idea!
    else {
      // FIXME get a better estimate of the time for the scan
      CASTTime timeNow;
      timeNow = getCASTTime();
      m_Scan.time = m_TimeOffset - timeNow;
    }


    m_Scan.ranges.resize(m_Sonar->GetCount());
    for (unsigned int i = 0; i < m_Sonar->GetCount(); i++) {
      m_Scan.ranges[i] = m_Sonar->GetScan(i);
    }
    
    unlockComponent();    
  }    
}

Sonar::SonarScan2d 
SonarServerPlayer::pullSonarScan2d(const Ice::Current&)
{
  //debug("pullSonarScan2d");
  return m_Scan;
}

// void
// SonarServerPlayer::registerSonarScan2dPushClient(const Sonar::SonarScan2dPushClientPrx& c, 
// 						 Ice::Double desiredInterval, 
// 						 const Ice::Current&) {
//   SonarScan2dClient client;
//   client.prx = c;
//   client.interval = desiredInterval;
//   m_PushClients.push_back(client);
// }


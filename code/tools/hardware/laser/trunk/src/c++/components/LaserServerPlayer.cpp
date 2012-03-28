//
// = FILENAME
//    LaserServerPlayer.cpp
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

#include "LaserServerPlayer.hpp"

#include <cstdlib>
#include <sstream>
#include <fstream>
#include <cast/core/CASTUtils.hpp>

using namespace Laser;
using namespace cast;
using namespace cast::cdl;
using namespace std;

/**
 * The function called to create a new instance of our component.
 */
extern "C" {
  cast::CASTComponentPtr 
  newComponent() {
    return new LaserServerPlayer();
  }
}

LaserServerPlayer::LaserServerPlayer()
{
  m_PlayerClient = 0;
  m_Laser = 0;

  m_PlayerHost = "localhost";
  m_PlayerPort = PlayerCc::PLAYER_PORTNUM;
  m_PlayerPosDeviceId = 0;

  m_MinPushInterval = -1;

  m_RandData = false;

  m_ProbFakeMinOutlier = -1;
  m_ProbFakeMaxOutlier = -1;
}

LaserServerPlayer::~LaserServerPlayer()
{}

void 
LaserServerPlayer::configure(const std::map<std::string,std::string> & config)
{
  println("configure");

  std::map<std::string,std::string>::const_iterator it;

  if ((it = config.find("--min-push-interval")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_MinPushInterval;
  }
  log("Using m_MinPushInterval=%fs", m_MinPushInterval);

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

  if ((it = config.find("--rand-data")) != config.end()) {
    m_RandData = true;
  }
  log("Using m_RandData=%d", (int)m_RandData);

  if ((it = config.find("--prob-fake-max-outlier")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_ProbFakeMaxOutlier;
  }
  log("Using m_ProbFakeMaxOutlier=%.5f", m_ProbFakeMaxOutlier);

  if ((it = config.find("--prob-fake-min-outlier")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_ProbFakeMinOutlier;
  }
  log("Using m_ProbFakeMinOutlier=%.5f", m_ProbFakeMinOutlier);

  m_Discr = -1;
  if ((it = config.find("--discretization")) != config.end()) {
    std::istringstream str(it->second);
    str >> m_Discr;
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

  if (m_Discr > 0) {
    log("Using m_Discr=%fm", m_Discr);
  } else {
    log("No additional artificial discretization on range data");
  }

  Laser::LaserServerPtr servant = new LaserServerI(this);
//  registerIceServer<cast::CASTComponent, LaserServer>(getComponentPointer());
  registerIceServer<LaserServer, LaserServer>(servant);
}

void LaserServerPlayer::saveScanToFile(Laser::Scan2d scan){

         char buf[256];
         sprintf(buf,"%s/laserscan_%ld_%ld", m_saveDirectory.c_str(),(long int)scan.time.s, (long int)scan.time.us);
         std::ofstream scanfile;

         scanfile.open (buf);

         scanfile << scan.angleStep << " " << scan.minRange << " " << scan.maxRange
        		 << " " << scan.rangeRes << " " << scan.startAngle << " ";
         for (int i = 0; i < scan.ranges.size(); i++){
        	 scanfile << scan.ranges[i] << " ";
         }
         scanfile << std::endl;
         scanfile.close();
}


void 
LaserServerPlayer::start() 
{
  println("start");
  if (!m_RandData) {
    m_PlayerClient = new PlayerCc::PlayerClient(m_PlayerHost, m_PlayerPort);

    m_PlayerClient->SetDataMode(PLAYER_DATAMODE_PULL);
    m_PlayerClient->SetReplaceRule(true, PLAYER_MSGTYPE_DATA);

    m_Laser = new PlayerCc::LaserProxy(m_PlayerClient, m_PlayerPosDeviceId);
  }
}
  
void
LaserServerPlayer::stop()
{}

void 
LaserServerPlayer::runComponent()
{
  //println("runComponent started");


  //FIXME a really big hack to see if we're running in simulation or not      
  bool useWallclockTimestamps = true;
  const cast::interfaces::TimeServerPrx timeServer(getTimeServer());
  if(isRunning()) {
    m_PlayerClient->Read();
    
    //timestamp on the data as an int
    long timestamp = (long) m_Laser->GetDataTime();
    
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

    if (!m_RandData) {
      m_PlayerClient->Read();

      //if we're guessing that the player timestamp is based on gettimeofday
      if(useWallclockTimestamps) {
	//create a CAST time from the double value      
	m_Scan.time = timeServer->fromTimeOfDayDouble(m_Laser->GetDataTime());	
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

      m_Scan.ranges.resize(m_Laser->GetCount());
      for (unsigned int i = 0; i < m_Laser->GetCount(); i++) {
        m_Scan.ranges[i] = m_Laser->GetRange(i);
      }
      m_Scan.startAngle = m_Laser->GetMinAngle();
      m_Scan.angleStep = (m_Laser->GetMaxAngle() - m_Laser->GetMinAngle()) / 
        (m_Laser->GetCount() - 1);
      m_Scan.maxRange = m_Laser->GetMaxRange();
      m_Scan.minRange = 0.01;
      //m_Scan.rangeRes = m_Laser->GetRangeRes();
      m_Scan.rangeRes = 0.01;
      
//       log("max range %f",m_Scan.maxRange);
//       log("range res %f",m_Scan.rangeRes);
//       log("count %d",m_Laser->GetCount());
//       log("min ang %f",m_Laser->GetMinAngle());
//       log("max ang %f",m_Laser->GetMaxAngle());

      if (fabs(m_Scan.startAngle) < 1e-3) m_Scan.startAngle = 0;

      if (m_ProbFakeMinOutlier > 0) {
        for (unsigned int i = 0; i < m_Laser->GetCount(); i++) {
          if ( (rand() / (RAND_MAX + 1.0)) < m_ProbFakeMinOutlier ) {
            m_Scan.ranges[i] = 0.5 * m_Scan.minRange;
          }
        }        
      }

      if (m_ProbFakeMinOutlier > 0) {
        for (unsigned int i = 0; i < m_Laser->GetCount(); i++) {
          if ( (rand() / (RAND_MAX + 1.0)) < m_ProbFakeMaxOutlier ) {
            m_Scan.ranges[i] = m_Scan.maxRange;
          }
        }        
      }      

    } else {

      sleepComponent(150);

      m_Scan.time = getCASTTime();
      m_Scan.ranges.resize(361);
      m_Scan.startAngle = 0;
      m_Scan.angleStep = M_PI/(m_Scan.ranges.size()-1);
      m_Scan.maxRange = 8;
      m_Scan.minRange = 0.04;
      m_Scan.rangeRes = 0.001;
      m_Scan.ranges[0] = 8.0 * (rand() / (RAND_MAX + 1.0));      
      for (unsigned int i = 1; i < m_Scan.ranges.size(); i++) {
        if (rand() / (RAND_MAX + 1.0) < 0.97) {
          m_Scan.ranges[i] = (m_Scan.ranges[i-1] + 
                              0.4 * 2 * ((rand() / (RAND_MAX + 1.0)) - 0.5));
          if (m_Scan.ranges[i] > m_Scan.maxRange) 
            m_Scan.ranges[i] = m_Scan.maxRange;
          if (m_Scan.ranges[i] < 1) m_Scan.ranges[i] = 1;
        } else {
          m_Scan.ranges[i] = 8.0 * (rand() / (RAND_MAX + 1.0));
        }
      }
    }

    if (m_Discr > 0) {
      for (unsigned int i = 1; i < m_Scan.ranges.size(); i++) {
        m_Scan.ranges[i] = m_Discr * int(m_Scan.ranges[i] / m_Discr + 0.5);
      }
    }

    if(m_saveToFile){
      saveScanToFile(m_Scan);
    }

    if(isRunning()) {
      // FIXME @demmel 22.03.2012:
      // using lockComponent() is a design flaw since this is held when the
      // component is stopped. Thus even with the isRunning() check there is a
      // race condition when the component is stopped. We should use an extra
      // mutex, not the component mutex.
      lockComponent();
      for (unsigned int i = 0; i < m_PushClients.size(); i++)  {
        if ( isRunning() &&
             (!m_PushClients[i].timer.isRunning() ||
              (m_PushClients[i].timer.split() >= m_PushClients[i].interval) ) ) {
          log("pushed scan to client %d", i);
          m_PushClients[i].timer.restart();
          m_PushClients[i].prx->receiveScan2d(m_Scan);
        }
      }
      unlockComponent();
    }
  }
}

Laser::Scan2d 
LaserServerPlayer::LaserServerI::pullScan2d(const Ice::Current&)
{
  debug("pullScan2d");
  return svr->m_Scan;
}

void
LaserServerPlayer::LaserServerI::registerScan2dPushClient(const Laser::Scan2dPushClientPrx& c, 
                                            Ice::Double desiredInterval, 
                                            const Ice::Current&)
{
  println("registerScan2dPushClient");

  Scan2dClient client;
  client.prx = c;
  client.interval = desiredInterval;
  svr->lockComponent();
  svr->m_PushClients.push_back(client);
  svr->log("Added push client %i", svr->m_PushClients.size()-1);
  svr->unlockComponent();
}


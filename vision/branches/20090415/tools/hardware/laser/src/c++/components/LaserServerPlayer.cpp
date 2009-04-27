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

#include <CASTUtils.hpp>

using namespace Laser;
using namespace cast;
using namespace cast::cdl;

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

  m_IceServerName = "LaserServer";

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

  // Start the interface that allows clients to connect to this server
  // the Ice way
  try {

    Ice::Identity id;
    id.name = m_IceServerName;
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
  println("runComponent started");


  while (isRunning()) {

    if (!m_RandData) {
      m_PlayerClient->Read();

      // FIXME get a better estimate of the time for the scan
      /*
      CASTTime timeNow;
      timeNow = getCASTTime();
      m_Scan.time = m_TimeOffset - timeNow;
      */
      m_Scan.time = getCASTTime();

      m_Scan.ranges.resize(m_Laser->GetCount());
      for (unsigned int i = 0; i < m_Laser->GetCount(); i++) {
        m_Scan.ranges[i] = m_Laser->GetRange(i);
      }
      m_Scan.startAngle = m_Laser->GetMinAngle();
      m_Scan.angleStep = (m_Laser->GetMaxAngle() - m_Laser->GetMinAngle()) / 
        (m_Laser->GetCount() - 1);
      m_Scan.maxRange = m_Laser->GetMaxRange();
      m_Scan.minRange = 0.01;
      m_Scan.rangeRes = m_Laser->GetRangeRes();
      
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

      sleepComponent(150);
    }

      
    for (unsigned int i = 0; i < m_PushClients.size(); i++)  {

      if ( isRunning() && 
           (!m_PushClients[i].timer.isRunning() ||
            (m_PushClients[i].timer.split() >= m_PushClients[i].interval) ) ) {
        debug("pushed scan to client %d", i);    
        m_PushClients[i].timer.restart();
        m_PushClients[i].prx->receiveScan2d(m_Scan);
      }
    }

  }
}

Laser::Scan2d 
LaserServerPlayer::pullScan2d(const Ice::Current&)
{
  debug("pullScan2d");
  return m_Scan;
}

void
LaserServerPlayer::registerScan2dPushClient(const Laser::Scan2dPushClientPrx& c, 
                                            Ice::Double desiredInterval, 
                                            const Ice::Current&)
{
  println("registerScan2dPushClient");

  Scan2dClient client;
  client.prx = c;
  client.interval = desiredInterval;
  m_PushClients.push_back(client);
}


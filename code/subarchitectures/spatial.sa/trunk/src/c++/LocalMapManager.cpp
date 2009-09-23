//
// = FILENAME
//    LocalMapManager.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#include "LocalMapManager.hpp"
#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#include <AddressBank/ConfigFileReader.hh>
#include <RobotbaseClientUtils.hpp>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new LocalMapManager();
  }
}

LocalMapManager::LocalMapManager()
{
  cure_debug_level = -10;

  m_RobotServerHost = "localhost";
}

LocalMapManager::~LocalMapManager() 
{ }



void LocalMapManager::configure(const map<string,string>& _config) 
{
  map<string,string>::const_iterator it = _config.find("-c");
  if (it== _config.end()) {
    println("configure(...) Need config file (use -c option)\n");
    std::abort();
  }
  std::string configfile = it->second;

  Cure::ConfigFileReader cfg;
  if (cfg.init(configfile)) {
    println("configure(...) Failed to open with \"%s\"\n",
            configfile.c_str());
    std::abort();
  }  

  if (cfg.getSensorPose(1, m_LaserPoseR)) {
    println("configure(...) Failed to get sensor pose");
    std::abort();
  } 

  m_MaxLaserRange = 5.0;
  it = _config.find("--laser-range");
  if (it != _config.end()) {
    m_MaxLaserRange = (atof(it->second.c_str()));
  }

  it = _config.find("--robot-server-host");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_RobotServerHost;
  }

  m_lgm = new Cure::LocalGridMap<unsigned char>(200, 0.1, '2', Cure::LocalGridMap<unsigned char>::MAP1);
  m_Glrt  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm);
  //m_Explorer->setExplorationConfinedByGateways(true);


  if (_config.find("--no-x-window") == _config.end()) {
    m_Displaylgm = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm);
    println("Will use X window to show the map");
  } else {
    m_Displaylgm = 0;
    println("Will NOT use X window to show the map");
  }

  m_RobotServer = RobotbaseClientUtils::getServerPrx(*this,
                                                     m_RobotServerHost);
} 

void LocalMapManager::start() 
{
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
		  new MemberFunctionChangeReceiver<LocalMapManager>(this,
								  &LocalMapManager::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<LocalMapManager>(this,
								  &LocalMapManager::newRobotPose));  
  
  log("LocalMapManager started");
  
}

void LocalMapManager::runComponent() 
{
  setupPushScan2d(*this, 0.1);
  setupPushOdometry(*this);

  log("I am running!");
  
  while(isRunning()){
    if (m_Displaylgm) {
      Cure::Pose3D currentPose = m_TOPP.getPose();
      m_Displaylgm->updateDisplay(&currentPose);
    }

    usleep(250000);
  }
}

void LocalMapManager::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{

  shared_ptr<CASTData<NavData::RobotPose2d> > oobj =
    getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);
  
  //FIXME
//   m_SlamRobotPose.setTime(Cure::Timestamp(oobj->getData()->time.s,
//                                           oobj->getData()->time.us));
  m_SlamRobotPose.setX(oobj->getData()->x);
  m_SlamRobotPose.setY(oobj->getData()->y);
  m_SlamRobotPose.setTheta(oobj->getData()->theta);
  
  Cure::Pose3D cp = m_SlamRobotPose;
  m_TOPP.defineTransform(cp);
  
}

void LocalMapManager::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  Cure::Pose3D cureOdom;
  CureHWUtils::convOdomToCure(castOdom, cureOdom);

  debug("Got odometry x=%.2f y=%.2f a=%.4f t=%.6f",
        cureOdom.getX(), cureOdom.getY(), cureOdom.getTheta(),
        cureOdom.getTime().getDouble());
  
  m_TOPP.addOdometry(cureOdom);
  
  m_CurrPose = m_TOPP.getPose();
}

void LocalMapManager::receiveScan2d(const Laser::Scan2d &castScan)
{
  debug("Got scan with n=%d and t=%ld.%06ld",
        castScan.ranges.size(), 
        (long)castScan.time.s, (long)castScan.time.us);

  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);

  if (m_TOPP.isTransformDefined()) {
    
    Cure::Pose3D scanPose;
    if (m_TOPP.getPoseAtTime(cureScan.getTime(), scanPose) == 0) {
      m_Mutex.lock();
      m_LMap.addScan(cureScan, m_LaserPoseR, scanPose);
      m_Mutex.unlock();

      Cure::Pose3D lpW;
//      m_lgm->setValueInsideCircle(scanPose.getX(), scanPose.getY(),
//                                  0.5*Cure::NavController::getRobotWidth(), 
//                                  '0');
      lpW.add(scanPose, m_LaserPoseR);
      m_Glrt->addScan(cureScan, lpW, m_MaxLaserRange);      
    }
  }
}

//
// = FILENAME
//    SlamProcess.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Chandana Paul
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2007 Chandana Paul
//                  2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "SlamProcess.hpp"

#include <CureHWUtils.hpp>

#include <Map/FeatureData.hh>
#include <Geometry/Line2D.hh>
#include <Map/WrappedSLAM.hh>
#include <Map/RLDisplayFeatureMap.hh>
#include <Transformation/Pose3D.hh> 
#include <AddressBank/ConfigFileReader.hh>
#include <Utils/HelpFunctions.hh>
#include <Utils/CureDebug.hh>
#include <SensorData/SensorData.hh>

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
    return new SlamProcess();
  }
}

SlamProcess::SlamProcess()
  : m_TimeMapLastSaved(0)
{
  m_PP = 0;
  m_MapFilename = "";

  m_SaveMapInterval = 1.0; // s

  m_WriteMapToWorkingMemory = true;
  m_LineMapIdString = "";

  m_LastOdom.setTime(0);
  m_LastUsedOdom.setTime(0);
  
  m_RobotPoseIdString = "";

  m_DontWriteFiles = false;

  m_ScansToIgnoreBeforeStart = 1;

  m_MaxScanRate = 5;
}

SlamProcess::~SlamProcess() 
{ 
  if (m_RunningSLAM && !m_DontWriteFiles) {
    ((Cure::WrappedSLAM*)m_PP)->saveMap(m_MapFilename);
  }
}

void SlamProcess::stop() 
{
  storeDataToFile();
}

void SlamProcess::configure(const map<string,string>& _config) 
{
  m_DontWriteFiles = (_config.find("--no-file-output") != _config.end());
  if (m_DontWriteFiles) log("Will NOT output files");
  else log("Will output files with robotpose and map"); 

  m_usePeekabot = false;
    if (_config.find("--usepeekabot") != _config.end())
      m_usePeekabot= true;

  map<string,string>::const_iterator it;

  std::string configfile = "";
  it = _config.find("-c");
  if(it == _config.end()) {
    printf("SlamThread::configure need name of config file (-c option)\n");
    exit(0);
  } else {
    configfile = it->second;
  }

  Cure::ConfigFileReader cfg;
  if (cfg.init(configfile)) {
    printf("configure failed to config with \"%s\"\n", configfile.c_str());
    exit(0);
  }  

  if (cfg.getSensorPose(1, m_LaserPoseR)) {
    println("configure(...) Failed to get sensor pose");
    std::abort();
  } 

  std::string usedCfgFile;
  double sigmaR;
  if (cfg.getDouble("SIGMA_R", true, sigmaR, usedCfgFile) == 0) {
    m_Lsq.setSigmaR(sigmaR);
    m_Seg.setSigmaR(sigmaR);
    log("Using sigmaR=%fm", sigmaR);
  }


  m_MapFilename = "";
  it = _config.find("-m");
  if(it != _config.end()) {
    m_MapFilename = it->second;
  }

  std::ifstream tmp(m_MapFilename.c_str());
  if (m_MapFilename != "" && tmp.is_open()) {
    tmp.close();
    Cure::WrappedLocalization *loc = new Cure::WrappedLocalization;
    loc->dontDisplay();
    m_RunningSLAM = false;
    m_PP = loc;

    if (loc->config(configfile)) {
      printf("SlamThread::configure Failed to config localization with \"%s\"\n",
	  configfile.c_str());
      m_PP = 0;
    } else if ( loc->loadMap(m_MapFilename) != 0) {
      printf("SlamThread::configure Failed to load map \"%s\"\n",
	  configfile.c_str());
      m_PP = 0;
    }

    if (m_PP == 0) {
      delete loc;
    }
  }

  m_PbPort = 5050;
  cfg.getRoboLookHost(m_PbHost);
  std::string tmpStr;
  if (cfg.getString("PEEKABOT_HOST", true, tmpStr, usedCfgFile) == 0) {
    m_PbHost = tmpStr;
  }

  if (m_PP == 0) {
    Cure::WrappedSLAM *slam = new Cure::WrappedSLAM;
    slam->dontDisplay();
    m_PP = slam;
    m_RunningSLAM = true;
    slam->getPose();
    if ( slam->config(configfile) ) {
      log(string("SlamThread::configure Failed to config slam with \"")
	  + configfile + string("\"\nExiting"));
      exit(0);
    }
  }

  if (m_MapFilename == "") {
    m_MapFilename = "tmpmap.metric";
  }

  it = _config.find("--num-wasted-first-scans");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_ScansToIgnoreBeforeStart;
  }
  printf("Will ignore first %d scans to make sure simulation is started",
      m_ScansToIgnoreBeforeStart);

  it = _config.find("--max-scan-rate");
  if (it != _config.end()) {
    std::istringstream str(it->second);
    str >> m_MaxScanRate;
  }
  if (m_MaxScanRate < 0) {
    printf("Will get scans as fast as possible");
  } else {
    printf("Want scans at at most %.1fHz rate", m_MaxScanRate);
  }

  m_NotRunningYet = true;
  m_InitedFakeRoutine = false;

  m_UpdateWithoutMotion = true;
  if (_config.find("--update-without-motion") != _config.end()) {
    m_UpdateWithoutMotion = true;
  }
  if (_config.find("--dont-update-without-motion") != _config.end()) {
    m_UpdateWithoutMotion = false;
  }

  printf("SlamProcess::configure successful\n");
}

void SlamProcess::runComponent() 
{  
  if(m_usePeekabot){
    while(!m_PeekabotClient.is_connected()){
      sleepComponent(1000);
      connectPeekabot();
      if (!isRunning())
        return;
    }
  }

  setupPushOdometry(*this, -1);
  if (m_MaxScanRate < 0) {
    setupPushScan2d(*this, -1);
  } else {
    setupPushScan2d(*this, 1/m_MaxScanRate);
  }
  log("I am running!");
}

void SlamProcess::receiveOdometry(const Robotbase::Odometry &castOdom)
{
  if (castOdom.odompose.empty()) {
    println("WARNING: Odometry struct contained no odompose which si needed");
    return;
  }


  const Robotbase::Pose2d &p = castOdom.odompose[0];

  char buf[256];
  sprintf(buf, "Got odometry x=%.3f, y=%.3f, theta=%.3f t=%ld.%06ld",
          p.x, p.y, p.theta, (long)castOdom.time.s, (long)castOdom.time.us);
  log(buf);

   Cure::Pose3D odom;  
   CureHWUtils::convOdomToCure(castOdom, odom);
  
   const double timeDiffToWarn = 0.5;
   if ( (m_LastOdom.getTime() > 0) &&
        (odom.getDoubleTime() - m_LastOdom.getDoubleTime() > timeDiffToWarn) ) {
     log("WARNING: Very long time between odometry timestamps %fs",
         odom.getDoubleTime() - m_LastOdom.getDoubleTime());
   }
  
   if (m_OdomTimer.isRunning()) {
     if (m_OdomTimer.stop() > timeDiffToWarn) {
       log("WARNING: receiveOdomData called very seldom (%fs since last), bad sign!!", m_OdomTimer.stop());
     }
   }
   m_OdomTimer.restart();

   m_LastOdom = odom;

   if (m_LastUsedOdom.getTime() == Cure::Timestamp(0) ||
       hypot(m_LastUsedOdom.getX() - odom.getX(), 
             m_LastUsedOdom.getY() - odom.getY()) > 1e-3 ||
       fabs(Cure::HelpFunctions::angleDiffRad(m_LastUsedOdom.getTheta(), 
                                              odom.getTheta())) > 1e-2) {
     m_RobotIsMoving = true;

     IceUtil::Mutex::Lock lock(m_Mutex);

     m_LastUsedOdom = odom;
   } else {
     m_RobotIsMoving = false;
   }
   
   if (m_NotRunningYet) {
     return;
   }

   
   debug("addOdom(x=%.2f y=%.2f a=%.2f t=%ld.%06ld)",
         odom.getX(), odom.getY(), odom.getTheta(), 
         odom.getTime().Seconds, odom.getTime().Microsec);
   {
     IceUtil::Mutex::Lock lock(m_Mutex);
     m_PP->addOdometry(odom);
   }

    if (m_usePeekabot){
        m_SLAMPoseProxy.set_pose(m_PP->getPose().getX(), m_PP->getPose().getY(), 2, m_PP->getPose().getTheta(), 0, 0);
    }
}

void SlamProcess::processScan2d(const Laser::Scan2d &castScan)
{
  if (m_ScansToIgnoreBeforeStart > 0) {
    m_ScansToIgnoreBeforeStart--;
    return;
  }
  
  if (castScan.ranges.empty()) {
    println("Empty scan, ignoring it");
    return;
  }

  debug("Got scan n=%d, r[0]=%.3f, r[n-1]=%.3f t=%ld.%06ld",
        castScan.ranges.size(), castScan.ranges[0], 
        castScan.ranges[castScan.ranges.size()-1],
        (long)castScan.time.s, (long)castScan.time.us);
  
  if (m_WriteMapToWorkingMemory) {
    writeLineMapToWorkingMemory(false);
  }

  Cure::LaserScan2d cureScan;
  CureHWUtils::convScan2dToCure(castScan, cureScan);  

  double timeDiffToWarn = 0.5;
  if (m_MaxScanRate>0) timeDiffToWarn += 1.0 / m_MaxScanRate;
  if (m_ScanTimer.isRunning()) {
    if (m_ScanTimer.stop() > timeDiffToWarn) {
      log("WARNING: receiveScanData called very seldom (%fs since last), bad sign!!", m_ScanTimer.stop());
    }
  }
  m_ScanTimer.restart();
        
  if ((m_NotRunningYet || m_RobotIsMoving || m_UpdateWithoutMotion) ||
      m_RobotPoseIdString.empty()) {
    
    // It seems that the WrappedXXXX component somehow does not
    // initilaize properly until it get a measurement. To fix that we
    // feed in a fake measurementset with a fake odometry which is a
    // copy of the last odometry but with the time set to exactly the
    // same as the one for the measurementset which is the same as the
    // laser scan. We need to wait for odometry before we can do
    // anything

    if (m_NotRunningYet) {

      if (m_LastUsedOdom.getTime() > 0) {

        println("Performing fake measurement update to kick start");

	{
	  IceUtil::Mutex::Lock lock(m_Mutex);

	  debug("pose est before fake x=%.2f y=%.2f a=%.4f t=%.6f",
	      m_PP->getPose().getX(), 
	      m_PP->getPose().getY(), 
	      m_PP->getPose().getTheta(),
	      m_PP->getPose().getTime().getDouble());

	  if (!m_InitedFakeRoutine) {
	    m_LastTimestampWhileNotRunningYet = m_PP->getPose().getTime();
	    m_InitedFakeRoutine = true;
	  }

	  Cure::Pose3D fakeOdom(m_LastUsedOdom);

	  fakeOdom.setTime(cureScan.getTime());
	  m_PP->addOdometry(fakeOdom);

	  Cure::MeasurementSet ms;
	  ms.setNumberOfElements(1);
	  ms.Measurements[0].MeasurementType = 0;
	  ms.Measurements[0].SensorType = Cure::SensorData::SENSORTYPE_SICK;
	  ms.Measurements[0].SensorID=0;
	  ms.Measurements[0].Key=0;
	  ms.Measurements[0].Z.reallocate(2,0);
	  ms.Measurements[0].BoundingBox.reallocate(0);
	  ms.Measurements[0].V.reallocate(4,0);
	  ms.Measurements[0].W.reallocate(0,2);
	  ms.Measurements[0].CovV.reallocate(0);
	  ms.setTime(cureScan.getTime());

	  m_PP->addMeasurementSet(ms);

	  debug("pose est after fake x=%.2f y=%.2f a=%.4f t=%.6f",
	      m_PP->getPose().getX(), 
	      m_PP->getPose().getY(), 
	      m_PP->getPose().getTheta(),
	      m_PP->getPose().getTime().getDouble());

	  m_NotRunningYet = (m_PP->getPose().getTime() ==
	      m_LastTimestampWhileNotRunningYet);      
	  m_LastTimestampWhileNotRunningYet = m_PP->getPose().getTime();
	}
      }

      if (m_NotRunningYet) {
        println("Still waiting for SLAM to initialize");
      } else {
        println("Fake measurement kick start finished x=%.2f y=%.2f a=%.4f t=%.6f",
                m_PP->getPose().getX(), 
                m_PP->getPose().getY(), 
                m_PP->getPose().getTheta(),
                m_PP->getPose().getTime().getDouble());

        updateRobotPoseInWM();
      }

    } else {

      debug("addXXXX(#=%d a0=%.2f aS=%.4f r0=%.2f rEnd=%.2f t=%ld.%06ld)",
            cureScan.getNPts(), cureScan.getStartAngle(), 
            cureScan.getAngleStep(), cureScan.getRange(0),
            cureScan.getRange(cureScan.getNPts()-1),
            cureScan.getTime().Seconds, cureScan.getTime().Microsec);

      CASTTimer timer(true);
      Cure::MeasurementSet measSet;
      int n = extractMeasSet(cureScan, measSet);
      double dt = timer.split();
      timer.stop();
      
      debug("Took %.3fs to extract %d lines, %.3fs to update (tot %.3fs)",
            dt, n, timer.stop()-dt, timer.stop());
      {
	IceUtil::Mutex::Lock lock(m_Mutex);
	m_PP->addMeasurementSet(measSet);
      }


      log("pose est after addXXXX x=%.2f y=%.2f a=%.4f t=%.6f",
          m_PP->getPose().getX(), 
          m_PP->getPose().getY(), 
          m_PP->getPose().getTheta(),
          m_PP->getPose().getTime().getDouble());

      updateRobotPoseInWM();
      
      storeDataToFile();
      
      if (m_WriteMapToWorkingMemory) {
        writeLineMapToWorkingMemory(true);
      }
    }

  } else {

      debug("No update, since not moving");

  }
}

int
SlamProcess::extractMeasSet(Cure::LaserScan2d &cureScan,
                            Cure::MeasurementSet &measSet)
{
  m_Seg.segment(cureScan);
  m_Lsq.doFit(m_Seg.m_Segments, cureScan);
  debug("Extracted %d lines with RANSAC+LSQ", m_Lsq.m_Lines.size());
  
  Cure::RedundantLine2DRep::makeMeasurementSetFromLines(m_Lsq.m_Lines,
                                                        measSet,
                                                        &m_Seg.m_Segments,
                                                        &cureScan);

  return m_Lsq.m_Lines.size();
}

void SlamProcess::updateRobotPoseInWM()
{
  // Retrieve currently estimated robot pose
  m_Mutex.lock();
  Cure::Pose3D pose = m_PP->getPose();
  m_Mutex.unlock();

  // Write estimated robot pose data into robot pose IDL structure
  NavData::RobotPose2dPtr pRP = new NavData::RobotPose2d;
  pRP->time.s = pose.getTime().Seconds;
  pRP->time.us = pose.getTime().Microsec;
  pRP->x = pose.getX();
  pRP->y = pose.getY();
  pRP->theta = pose.getTheta(); 

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
            pose.getX(), pose.getY(), pose.getTheta(),
            pose.getTime().Seconds, pose.getTime().Microsec);
    debug(buf);
  }
}

void SlamProcess::storeDataToFile()
{
  Cure::Timestamp currTime;
  currTime.setToCurrentTime();

  if ((currTime - m_TimeMapLastSaved).getDouble() > m_SaveMapInterval) {
    if (m_RunningSLAM) {
      if (!m_DontWriteFiles) {
	IceUtil::Mutex::Lock lock(m_Mutex);

        ((Cure::WrappedSLAM*)m_PP)->saveMap(m_MapFilename);
      }
      
      m_WriteMapToWorkingMemory = true;
    }

    if (!m_DontWriteFiles) {
      std::fstream fsl;
      fsl.open("robotpose.ccf", std::ios::out);
      if (fsl > 0) {
        m_Mutex.lock();
        Cure::Pose3D p = m_PP->getPose();
        m_Mutex.unlock();
        double ang[3];
        p.getAngles(ang);
        fsl << "ROBOTPOSE\n";
        fsl << p.getX() << " "
            << p.getY() << " "
            << p.getZ() << " "
            << ang[0] << " "
            << ang[1] << " "
            << ang[2] << std::endl;
      }
      fsl.close();
    }
    
    m_TimeMapLastSaved = currTime;
  }
}

int
SlamProcess::writeLineMapToWorkingMemory(bool overwrite)
{
  if (m_PP == 0) return -1;

  Cure::FeatureMap *fm = 0;
  if (m_RunningSLAM) {
    fm = &(((Cure::WrappedSLAM*)m_PP)->m_Map);
  } else {
    fm = &(((Cure::WrappedLocalization*)m_PP)->m_Map);
  }
  
  std::list<Cure::Line2D> walls;
  if (fm->getLine2DWalls(walls) != 0) {
    debug("Failed to get linemap");
    return -2;
  }
  debug("Has %d lines in the map", walls.size());

  // Get the current pose which we use to get the timestamp
  m_Mutex.lock();
  Cure::Pose3D cp = m_PP->getPose();
  m_Mutex.unlock();

  NavData::LineMapPtr lineMap = new NavData::LineMap();
  lineMap->time.s = cp.getTime().Seconds;
  lineMap->time.us = cp.getTime().Microsec;
  if (walls.size() > 0) {
    int i = 0;
    for (std::list<Cure::Line2D>::iterator w = walls.begin();
         w != walls.end(); w++) {
      lineMap->lines.push_back(NavData::LineMapSegement());
      lineMap->lines.back().start.x = w->StartPoint.getX();
      lineMap->lines.back().start.y = w->StartPoint.getY();
      lineMap->lines.back().end.x = w->EndPoint.getX();
      lineMap->lines.back().end.y = w->EndPoint.getY();
      i++;
    }
  }

  if (overwrite) {
    overwriteWorkingMemory<NavData::LineMap>(m_LineMapIdString,
                                             lineMap); // synch!
    char buf[128];
    sprintf(buf, "Overwriting wm linemap object \"%s\" with %d lines ", 
            m_LineMapIdString.c_str(), walls.size());
    debug(buf);
  } else {
    m_LineMapIdString = newDataID();
    addToWorkingMemory<NavData::LineMap>(m_LineMapIdString, lineMap);
    char buf[128];
    sprintf(buf, "Adding wm linemap object \"%s\" with %d lines ", 
            m_LineMapIdString.c_str(), walls.size());
    debug(buf);
  }

  m_WriteMapToWorkingMemory = false;

  return 0;
}

void SlamProcess::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again?) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);
    
    m_PeekabotClient.connect(m_PbHost, m_PbPort);

        m_ScanAngFOV = M_PI/180.0*240;
        m_ScanMaxRange = 5.6;

      m_SLAMPoseProxy.add(m_PeekabotClient, "SLAM_Pose",peekabot::REPLACE_ON_CONFLICT);
      m_SLAMPoseProxy.set_scale(0.1,0.04,0.04);

    log("Connection to Peekabot established");

    
  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}

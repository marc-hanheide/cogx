//
// = FILENAME
//    SlamProcess.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Chandana Paul, Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2007 Chandana Paul, Patric Jensfelt
//                  2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef SlamProcess_hpp
#define SlamProcess_hpp

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include <cast/architecture/ManagedComponent.hpp>
#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>
#include <NavData.hpp>
#include <Laser.hpp>
#include <Robotbase.hpp>
#include <cast/core/CASTTimer.hpp>

// Cure stuff
#include <Map/PoseProvider.hh>
#include <SensorData/LaserScan2d.hh>
#include <Map/WrappedSLAM.hh>
#include <Map/WrappedLocalization.hh>
#include <Transformation/Pose3D.hh>
#include <AddressBank/ConfigFileReader.hh>
#include <Utils/RoboLookProxy.h>
#include <Sensory/RANSACSegmentor.hh>
#include <Sensory/LsqLineFitter.hh>

namespace navsa {

/**
 * This is a CAST managed process that runs SLAM.
 *
 * @author Chandana Paul and Patric Jensfelt
 *
 * Command line options:
 * 
 * @param -c cure config file containing information about sensor position, 
 *   odometry parameters, initial robot pose, etc
 * @param -m optional map file. If the file exist it will be loaded and
 *   this component will operate in localization mode, otherwise the map
 *   built by SLAM will be saved under this name
 * @param --num-wasted-first-scans num scans to ignore before starting 
 *   (hack to make sure simulation has started). This is necessary sometimes
 *   when you specify a new position for the robot in stage which seems to
 *   take some time before it kicks in
 * @param --no-file-output   pass this argument if you do not want the 
 *   mapfile and the robotpose.ccf file to be written
 * @param --update-without-motion pass this option if you want the robot to 
 *   perform an pose update even if the robot has not moved
 * @param --max-scan-rate specify how many scans per second (default 5Hz) 
 *   we want at most to the slam process. Give negative number to
 *   get as fast as possible
 */
class SlamProcess : public cast::ManagedComponent,
                    public Scan2dReceiver,
                    public OdometryReceiver
{
public:
  SlamProcess();
  virtual ~SlamProcess();
  
  virtual void runComponent();
  virtual void stop();

  void processScan2d(const Laser::Scan2d &scan);
  inline void receiveScan2d(const Laser::Scan2d &castScan)
  {
    lockComponent();
    processScan2d(castScan);
    unlockComponent();
  }
  void receiveOdometry(const Robotbase::Odometry &odom);

 protected:
  virtual void configure(const std::map<std::string, std::string>& _config);

  void updateRobotPoseInWM();
  int writeLineMapToWorkingMemory(bool overwrite);
  int extractMeasSet(Cure::LaserScan2d &cureScan, Cure::MeasurementSet &mSet);

  void connectPeekabot();
 private:
    bool m_usePeekabot;

  void storeDataToFile();

  IceUtil::Mutex m_Mutex;

  // If true we are running in slam mode, else we run in localization
  // mode
  bool m_RunningSLAM;

  // Segmentation method
  Cure::RANSACSegmentor m_Seg;

  // Line fitting method
  Cure::LsqLineFitter m_Lsq;

  // This is a pointer to either a localization or a slam objects
  Cure::PoseProvider *m_PP;

  // The string that identifies the robotpose in the working memory
  std::string m_RobotPoseIdString;

  // True when we have to write the map to working memory
  bool m_WriteMapToWorkingMemory;

  // The string that identifies the map in the working memory
  std::string m_LineMapIdString;

  // The name of the map file we read from / write to
  std::string m_MapFilename;

  // Maximum desired scan rate from the server
  double m_MaxScanRate;

  // Interval with which we save map data to file and write it to
  // working memory
  double m_SaveMapInterval;

  // Time when the map was saved to file and written to working memory
  // last
  Cure::Timestamp m_TimeMapLastSaved;

  bool m_InitedFakeRoutine;
  Cure::Timestamp m_LastTimestampWhileNotRunningYet;
  bool m_NotRunningYet;

  int m_ScansToIgnoreBeforeStart;

  cast::CASTTimer m_OdomTimer;
  cast::CASTTimer m_ScanTimer;

  Cure::Pose3D m_LastOdom;
  Cure::Pose3D m_LastUsedOdom;

  bool m_RobotIsMoving;

  bool m_DontWriteFiles;

  bool m_UpdateWithoutMotion;

  double m_ScanAngFOV;
  double m_ScanMaxRange;
  Cure::SensorPose m_LaserPoseR;
  peekabot::PeekabotClient m_PeekabotClient;
  std::string m_PbHost;
  int m_PbPort;
  peekabot::CubeProxy m_SLAMPoseProxy;

};

}; // namespace navsa

#endif // SlamProcess_hpp

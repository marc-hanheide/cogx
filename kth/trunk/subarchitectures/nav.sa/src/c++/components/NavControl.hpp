//
// = FILENAME
//    NavControl.hpp
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

#ifndef NavControl_hpp
#define NavControl_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>
#include <NavData.hpp>

#include <string>
#include <map>

#include <SensorData/LaserScan2d.hh>
#include <Navigation/NavGraph.hh>
#include <Navigation/NavController.hh>
#include <Navigation/FrontierExplorer.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <NavX/XDisplayLocalGridMap.hh>
#include <Map/TransformedOdomPoseProvider.hh>
#include <Navigation/LocalMap.hh>
#include <SensorData/SensorPose.hh>

namespace navsa {

/**
 * This class wraps the class Cure::NavController which contains
 * navigation functionality aided by a navigation graph (acquired via
 * Working Memory). The assumes that you provide a configuration file
 * that specifies the geometry of the robot.
 *
 * @param -c cure config file to define the robot shape, sensor pose, etc
 * @param --explore-range the sensor range when performing exploration (default 1m)
 * @param --robot-server-host the ice server name for the robot server (default RobotbaseServer)
 * @param --no-x-window do not show the window with the grid map
 *
 * @author Patric Jensfelt
 * @see
 */
class NavControl : public cast::ManagedComponent ,
                   public Scan2dReceiver,
                   public OdometryReceiver,
                   public Cure::NavController,                  
                   public Cure::NavControllerEventListener,
		   public Cure::FrontierExplorerEventListener
{

public:

  NavControl ();
  virtual ~NavControl();

  virtual void runComponent();
  virtual void start();

  void abortTask(int taskID);
  void doneTask(int taskID);
  void failTask(int taskID, int error);
  void explorationDone(int taskID, int status);

protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  virtual void taskAdopted(const std::string &_taskID) {};
  virtual void taskRejected(const std::string &_taskID) {};

  double m_MaxExplorationRange; 

  Cure::LocalMap m_LMap;
  Cure::NavGraph m_NavGraph;

  Cure::GridLineRayTracer<unsigned char>* m_Glrt;
  Cure::XDisplayLocalGridMap<unsigned char>* m_Displaylgm;
  Cure::FrontierExplorer* m_Explorer;

  IceUtil::Mutex m_Mutex;
  Cure::LocalGridMap<unsigned char>* m_lgm;

  Cure::TransformedOdomPoseProvider m_TOPP;

  Cure::Pose3D m_SlamRobotPose;
  Cure::Pose3D m_CurrPose;
  Cure::SensorPose m_LaserPoseR;
  
  NavData::InternalCommandType m_commandType;
  double m_commandX;
  double m_commandY;
  double m_commandR;
  double m_commandTheta;
  double m_commandDistance;
  long m_commandAreaId;
  long m_commandNodeId;
  bool ExplorationConfinedByGateways;
  double m_DefTolPos;
  double m_DefTolRot;
  double m_TolPos;
  double m_TolRot;
  bool m_ready;

  class PersonData {
  public:
    std::string m_WMid;
    NavData::PersonPtr m_data;
  };

  // The people that are currently in view
  std::vector<PersonData> m_People;

  // Index of the current person to follow
  int m_CurrPerson;
  std::string m_CurrPersonWMid;

  enum TaskStatus {
    NothingToDo = -1, // no nav ctrl command to execute
    NewTask,          // nav ctrl command got, but not started yet
    ExecutingTask,    // nav ctrl command started
    TaskFinished      // nav ctrl command finished, completion needs posting
  };

  int m_taskId;
  IceUtil::Mutex m_taskStatusMutex; // protects flags and info about task
  bool m_currentTaskIsExploration; // says when doneTask events must be ignored
  TaskStatus m_taskStatus;

  // To return information
  cast::cdl::WorkingMemoryAddress m_CurrentCmdAddress;
  NavData::Completion m_CurrentCmdFinalCompletion;
  NavData::StatusError m_CurrentCmdFinalStatus;

  std::string m_RobotServerHost;
  Robotbase::RobotbaseServerPrx m_RobotServer;

  int m_NumInhibitors;
  bool m_SentInhibitStop;

protected:
  /* 
   * This functions is called with m_taskStatusMutex locked
   */
  void changeCurrentCommandCompletion(const NavData::Completion &value,
                                      const NavData::StatusError &status);

private:
  void execCtrl(Cure::MotionAlgorithm::MotionCmd&);
  void receiveScan2d(const Laser::Scan2d &castScan);
  void receiveOdometry(const Robotbase::Odometry &castOdom);
  void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
  void newNavCtrlCommand(const cast::cdl::WorkingMemoryChange &objID);
  void newNavGraph(const cast::cdl::WorkingMemoryChange &objID);
  void newInhibitor(const cast::cdl::WorkingMemoryChange &objID);
  void deleteInhibitor(const cast::cdl::WorkingMemoryChange &objID);
  void newPersonData(const cast::cdl::WorkingMemoryChange &objID);
  void deletePersonData(const cast::cdl::WorkingMemoryChange &objID);
}; 

}; // namespace navsa

#endif // NavControl_hpp

//
// = FILENAME
//    SpatialControl.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Chandana Paul
//    Patric Jensfelt
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2007 Chandana Paul
//                  2009 Patric Jensfelt
//                  2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

//NOTE: This file "adapted" (nicked) from nav.sa version with NavCommand structs.
//SpatialData doesn't have everything that NavData does; the extraneous
//portions of this code have been commented out.

#ifndef SpatialControl_hpp
#define SpatialControl_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <Scan2dReceiver.hpp>
#include <PointCloudClient.h>
#include <OdometryReceiver.hpp>
#include <NavData.hpp>

#include <string>
#include <queue>
#include <vector>
#include <map>

#include <SensorData/LaserScan2d.hh>
#include <Navigation/NavGraph.hh>
#include <Navigation/NavController.hh>
#include <Navigation/FrontierExplorer.hh>
#include "GridLineRayTracerModified.hh"
#include <NavX/XDisplayLocalGridMap.hh>
#include <Map/TransformedOdomPoseProvider.hh>
#include <Navigation/LocalMap.hh>
#include <PTZ.hpp>
#include <SensorData/SensorPose.hh>
#include <FrontierInterface.hpp>

namespace spatial {

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
class SpatialControl : public cast::ManagedComponent ,
                   public Scan2dReceiver,
                   public OdometryReceiver,
                   public Cure::NavController,                  
                   public Cure::NavControllerEventListener,
		   						 public Cure::FrontierExplorerEventListener,
		   						 public Cure::LocalMap,
									 public cast::PointCloudClient
{
  private:
    class FrontierServer: public FrontierInterface::FrontierReader {
      virtual FrontierInterface::FrontierPtSeq getFrontiers(const Ice::Current &_context) {
				FrontierInterface::FrontierPtSeq ret =
					m_pOwner->getFrontiers();
				return ret;
      }
      SpatialControl *m_pOwner;
      FrontierServer(SpatialControl *owner) : m_pOwner(owner)
      {}
      friend class SpatialControl;
    };
    friend class FrontierServer;


    class MapServer: public SpatialData::MapInterface {
      private:
        virtual bool isCircleObstacleFree(double x, double y, double radius, const Ice::Current &_context) {
          if(!m_pOwner->m_lgm)
            throw "No map exists. Can not check for obstacles.";

          return m_pOwner->m_lgm->isCircleObstacleFree(x,y,radius);
        }

        virtual bool isPointReachable(double xW, double yW, const Ice::Current &_context);
        virtual SpatialData::BoolSeq arePointsReachable(const SpatialData::CoordinateSeq& points, const Ice::Current &_context);
        SpatialControl *m_pOwner;
        MapServer(SpatialControl *owner) : m_pOwner(owner) {}
        friend class SpatialControl;
    };
    friend class MapServer;



public:

  SpatialControl ();
  virtual ~SpatialControl();

  virtual void runComponent();
  virtual void start();

  void abortTask(int taskID);
  void doneTask(int taskID);
  void failTask(int taskID, int error);
  void explorationDone(int taskID, int status);
  const Cure::LocalGridMap<unsigned char>& getLocalGridMap();

protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  virtual void taskAdopted(const std::string &_taskID) {};
  virtual void taskRejected(const std::string &_taskID) {};
  virtual bool isPointReachable(double xW, double yW);
  virtual std::vector<bool> arePointsReachable(std::vector<std::vector<double> > points);

  void blitHeightMap(Cure::LocalGridMap<unsigned char>& lgm, int minX, int maxX, int minY, int maxY);
  void updateGridMaps();

  double m_MaxExplorationRange; 

  Cure::LocalMap m_LMap;
  Cure::NavGraph m_NavGraph;
  bool m_bNoNavGraph;

  Cure::GridLineRayTracer<unsigned char>* m_Glrt; // This is from our customized cure raytracer. GridLineRayTracerModified.hh in spatial.sa

  Cure::XDisplayLocalGridMap<unsigned char>* m_Displaylgm;
  Cure::FrontierExplorer* m_Explorer;
  Cure::XDisplayLocalGridMap<unsigned char>* m_displayBinaryMap;


  IceUtil::Mutex m_Mutex;
  IceUtil::Mutex m_MapsMutex;
  IceUtil::Mutex m_ScanQueueMutex;
  Cure::LocalGridMap<unsigned char>* m_lgm;
  Cure::LocalGridMap<unsigned char>* m_lgmLM; // LGM to display LocalMap (m_LMap)
  Cure::LocalGridMap<double>* m_lgmKH; // Kinect height map

  Cure::LocalGridMap<unsigned char>* m_binaryMap;

  IceUtil::Monitor<IceUtil::Mutex> m_LScanMonitor;
	std::queue<Cure::LaserScan2d> m_LScanQueue;	

  int m_Npts;
	double m_StartAngle;
	double m_AngleStep;
  

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

  ptz::PTZInterfacePrx m_ptzInterface;

  bool m_firstScanAdded;

  bool m_UsePointCloud;
  double m_obstacleMinHeight;
  double m_obstacleMaxHeight;

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

  FrontierInterface::FrontierPtSeq getFrontiers();
}; 

}; // namespace spatial

#endif // SpatialControl_hpp

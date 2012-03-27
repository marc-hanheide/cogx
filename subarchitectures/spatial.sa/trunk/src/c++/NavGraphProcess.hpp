//
// = FILENAME
//   NavGraphProcess.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef NavGraphProcess_hpp
#define NavGraphProcess_hpp

#include <string>
#include <deque>

#include <cast/architecture/ManagedComponent.hpp>
#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>
#include <NavData.hpp>
#include <Laser.hpp>

// Cure includes
#include <Navigation/NavGraph.hh>
#include <Map/TransformedOdomPoseProvider.hh>
#include <Navigation/InDoorOpeningDetector.hh>
#include <Navigation/ScanMotionDetector.hh>
#include <Transformation/Pose3D.hh>
#include <SensorData/LaserScan2d.hh>

// Foward declarations
namespace Cure {
  class NavGraphNode;
  class NavGraphGateway;
};

namespace navsa {

/**
 * This class builds the navigation graph using the robot pose,
 * odometry and laser scans as inputs. The laser scans are used to
 * detect doors.
 *
 * Options:
 * @param -m mapfile, name of graph file to load from / write to
 * @param -c CURE config file to get SENSORPOSE
 * @param --no-file-output supress writing the graph file
 * @param --auto-merge-areas merge areas automatically instead of start clar.dialog
 * @param --min-door-width  Minimum accepted door width [m] (def 0.6m)
 * @param --max-door-width  Maximum accepted door width [m] (def 1.05m)
 * @param --remove-motion remove moving objects before trying to extract doors
 *
 * @author Patric Jensfelt
 * @see
 */
class NavGraphProcess : public cast::ManagedComponent, 
                        public Cure::NavGraphEventListener,
                        public OdometryReceiver,
                        public Scan2dReceiver
{
public:
  NavGraphProcess();
  virtual ~NavGraphProcess();
  
  void runComponent();
  void start();
  
protected:

  void taskAdopted(const std::string &_taskID) {}
  void taskRejected(const std::string &_taskID) {}
  void configure(const std::map<std::string, std::string>& _config);

  void writeTopologicalPosToWorkingMemory(int aid);

  void addFreeNode(long nodeID, 
                   double x, double y, double z, double theta,
                   long areaID, const std::string &areaType, long areaTypeNo,
                   short gateway, 
                   double maxSpeed, double width,
                   const std::string &name,
                   bool tryToPullAreaType = false);

  /**
   * @param camX x-position of the camera in the world
   * @param camY y-position of the camera in the world
   *
   */
  void addObject(double x, double y, double z, 
                 double camX, double camY,
                 long areaID,
                 const std::string &category, 
                 long objectID, double probability);
  
  void addAccessEdge(long startNodeID, long endNodeID, double weight); 


private:
  class NavGraphServer : public NavData::NavGraphInterface{
    virtual double getPathLength(double xS, double yS, double aS, double xG, double yG, double aG, const Ice::Current &_context);
    virtual  int getAreaId(double x, double y, double a, double maxDist , const Ice::Current &_context);
    virtual int getClosestNodeId(double x, double y, double a, double maxDist ,const Ice::Current &_context);
      NavGraphProcess *m_pOwner;
      NavGraphServer(NavGraphProcess *owner) : m_pOwner(owner)
      {}
      friend class NavGraphProcess;
    };
    friend class NavGraphServer;


  void areaIdConflict(Cure::NavGraphNode &currNode, Cure::NavGraphGateway &gw);
  void changedArea(int aid);
  void checkAndAddArea(int aid,
                       const std::string &areaClass = "",
                       long areaClassNo = -1);
  void changedCurrentNode(int fromId, int toId);
  void mergedAreaIds(int aid1, int aid2);
  void changedNodeType(int id);

  /// @return 0 if node was added, 1 if node was already added
  int processNewCureNavGraphNode(bool knownToBeGateway= false, double w=1);
  void checkForNodeChanges();

  void writeGraphToWorkingMemory(bool forceWrite = false);

  void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);

  void newObjObs(const cast::cdl::WorkingMemoryChange &objID);
  void newVisualObject(const cast::cdl::WorkingMemoryChange & wmChange);
  void newDoorHypothesis(const cast::cdl::WorkingMemoryChange &objID);
  
  // The above merely queue events; the methods below do the work
  void processNewObjObs(const cast::cdl::WorkingMemoryAddress &objID);
  void processNewVisualObject(const cast::cdl::WorkingMemoryAddress & wmChange);
  void processNewDoorHypothesis(const cast::cdl::WorkingMemoryAddress &objID);
  void processScan(Cure::LaserScan2d &cureScan);

  // Callback function for new door hypothesis

  void receiveOdometry(const Robotbase::Odometry &castOdom);
  void receiveScan2d(const Laser::Scan2d &castScan);

  // This function returns a unique single long id for the edge. This
  // id is constructed by taking the smallest node id and multiplying
  // it with m_MaxNumFNodes and adding the larger nodeId. This
  // provides the means to easily lookup a certain node in the std::map
  // structure
  long makeEdgeId(int nodeId1, int nodeId2);

  bool areNodesInWM(long id1, long id2);

  void loadGraphFromFile(const std::string &filename);
  void saveGraphToFile(const std::string &filename);

  IceUtil::Mutex m_GraphMutex; //Protects m_Areas, m_cureNavGraph
  IceUtil::Mutex m_TOPPMutex;
  IceUtil::Mutex m_eventQueueMutex;
  IceUtil::Mutex m_scanQueueMutex;

  enum GraphEventType {NEW_OBJ_OBS, NEW_VISUAL_OBJECT, NEW_DOOR_HYPTOHESIS};
  std::deque<std::pair<GraphEventType, cast::cdl::WorkingMemoryAddress> >
    m_graphEventQueue;

  std::deque<Cure::LaserScan2d> m_scanQueue;

  Cure::NavGraph m_cureNavGraph;
  std::string m_cureNavGraphFile;
  std::string m_NavGraphWMid;

  std::vector<std::pair<double, double> > m_doorHypPositions;

  /// This class holds the data for the areas and pointers to where
  /// they can be found in working memory
  class Area {
  public:
    std::string m_WMid;
    NavData::AreaPtr m_data;    
  };

  // List of areas
  std::list<Area> m_Areas;

  std::list<Area> m_AreasFromFile;

  // Pointer to the struct in working memory that holds the
  // topological position
  NavData::TopologicalRobotPosPtr m_TopRobPos;
  std::string m_TopRobPosWMid;
  bool m_WriteFirstTopologicalPose; // true when navgraph read from file

  bool m_UseDoorHypotheses; // Whether to filter out doorways not detected from afar

  Cure::InDoorOpeningDetector m_DoorDetector;
  Cure::SensorPose m_LaserPoseR;
  Cure::SensorPose m_CamPoseR;

  bool m_InDoor;
  double m_MinDoorWidth;
  double m_MaxDoorWidth;

  Cure::Timestamp m_LastTimePoseUsed;
  Cure::Pose3D m_LastDoorCheckPose;
  Cure::Pose3D m_LastDoorPose;

  bool m_WriteFirstGraph;

  std::string m_MapLoadStatusWM;

  // This variable is used to tell what ids to assign the object nodes
  // and also used when creating a unique combined id for edges.  For
  // simplicity you should pick a value which 10^N since that allows
  // you to visually read out the ids directly from the number 
  long m_MaxNumFNodes;
  
  // If this is true then the graph will replace any object of the
  // same category if a new observation is made, without checking if
  // it is close to the object or not.
  bool m_UniqueObjects;

  long m_NextObjectNodeId;

  Cure::Pose3D m_LastOdom;
  Cure::Pose3D m_LastRobotPose;

  Cure::TransformedOdomPoseProvider m_TOPP;

  class FNodeHolder {
  public:
    NavData::FNodePtr m_data;
    std::string m_WMid;
  };
  std::map<long, FNodeHolder> m_FreeNodes;
  std::list<FNodeHolder> m_NewFNodes;
  std::list<FNodeHolder*> m_ChangedNodes;

  class AEdgeHolder {
  public:
    NavData::AEdgePtr m_data;
    std:: string m_WMid;
  };
  std::map<long, AEdgeHolder> m_AccessEdges;
  std::list<AEdgeHolder> m_NewAEdges;

  class ObjDataHolder {
  public:
    NavData::ObjDataPtr m_data;
    std::string m_WMid;
  };
  std::map<long, ObjDataHolder> m_Objects;
  std::list<ObjDataHolder> m_NewObjData;
  std::list<ObjDataHolder*> m_ChangedObjects;

  bool m_DontWriteFiles;

  bool m_RemoveMotionBeams;
  int m_LaserFreqEstNScans;
  double m_LaserFreqEstStartTime;
  Cure::ScanMotionDetector *m_MotionDetector;
};

}; // namespace navsa

#endif // NavGraphProcess_hpp

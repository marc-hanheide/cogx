//
// = FILENAME
//    LocalMapManager.hpp
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

#ifndef LocalMapManager_hpp
#define LocalMapManager_hpp
#include <cast/architecture/ManagedComponent.hpp>
#include <Scan2dReceiver.hpp>
#include <OdometryReceiver.hpp>
#include <NavData.hpp>

#include <SensorData/LaserScan2d.hh>
#include <Navigation/NavGraph.hh>
//#include <Navigation/FrontierExplorer.hh>
#include <Navigation/GridLineRayTracer.hh>
#include <NavX/XDisplayLocalGridMap.hh>
#include <Map/TransformedOdomPoseProvider.hh>
#include <Navigation/LocalMap.hh>
#include <SensorData/SensorPose.hh>
#include <FrontierInterface.hpp>
#include <Transformation/Transformation3D.hh>
#include <PTZ.hpp>
#include <VisionData.hpp>
#include "GridObjectFinder.hpp"

namespace spatial {
  typedef std::vector<std::pair<double, double> > PlaneList;

struct PlaneData {
  //first component is the Z position; the second, the confidence.
  PlaneList planes; 
};

std::ostream &operator<<(std::ostream &os, const PlaneData &data)
{
  os.precision(2);
  double maxHeight = 0.0;
  for (PlaneList::const_iterator it = data.planes.begin();
      it != data.planes.end(); it++) {
    if (it->second > 10.0) {
      if (it->first > maxHeight) 
	maxHeight = it->first;
    }
  }
  os << maxHeight;
  return os;
}

bool operator==(const PlaneData &data, char cmp)
{
  if (cmp == '2') { //Unknown
    return data.planes.empty();
  }

  double maxHeight = 0.0;
  for (PlaneList::const_iterator it = data.planes.begin();
      it != data.planes.end(); it++) {
    if (it->second > 10.0) {
      if (it->first > maxHeight) maxHeight = it->first;
    }
  }

  if (cmp == '0') return maxHeight == 0.0;
  return maxHeight != 0.0;
}

typedef Cure::LocalGridMap<unsigned char> CharMap ;
typedef Cure::GridLineRayTracer<unsigned char> CharGridLineRayTracer;
typedef Cure::LocalGridMap<PlaneData> PlaneMap;
/**
 * This class maintains small grid maps around individual nav nodes. The maps
 * are centered on the nav node, and continually updated as long as the robot
 * is at that node. This makes sure that the robot doesn't know about free space
 * behind walls, in other rooms.
 * Also maintains a tentative local map that is used to initialize the grid
 * map of a newly discovered node; this tentative map is cleared each time a known
 * node is visited.
 * @param -c cure config file to define the robot shape, sensor pose, etc
 * @param --laser-range the range at which the laser is capped when sweeping out
 * free space. NOTE: Takes precedence over the next two options
 * @param --laser-range-for-placeholders Specific range for local maps used
 * with placeholder evaluation
 * @param --laser-range-for-combined-maps Specific range for local maps used
 * with the getCombinedLocalMaps interface
 * @param --laser-range the range at which the laser is capped when sweeping out
 * free space
 * @param --robot-server-host the ice server name for the robot server (default RobotbaseServer)
 * @param --no-tentative-window Do not show the window displaying the tentative map
 * @param --no-local-map-window Do not show the window displaying the current node's map
 * @param --no-planes		Do not record stereo planes
 *
 * @author Kristoffer Sjöö
 * @see
 */
class LocalMapManager : public cast::ManagedComponent,
  		  public OdometryReceiver,
		  public Scan2dReceiver
{
  private:
    class EvaluationServer: public FrontierInterface::HypothesisEvaluator {
      virtual FrontierInterface::HypothesisEvaluation getHypothesisEvaluation(int hypID, const Ice::Current &_context);
      LocalMapManager *m_pOwner;
      EvaluationServer(LocalMapManager *owner) : m_pOwner(owner)
      {}
      friend class LocalMapManager;
    };
    friend class EvaluationServer;
    
    class LocalMapServer: public FrontierInterface::LocalMapInterface {
      virtual FrontierInterface::LocalGridMap getCombinedGridMap(const SpatialData::PlaceIDSeq &places,
	  const Ice::Current &_context);
      LocalMapManager *m_pOwner;
      LocalMapServer(LocalMapManager *owner) : m_pOwner(owner)
      {}
      friend class LocalMapManager;
    };
    friend class LocalMapServer;
public:
  LocalMapManager();
  virtual ~LocalMapManager();

  virtual void runComponent();
  virtual void start();

protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  virtual void taskAdopted(const std::string &_taskID) {};
  virtual void taskRejected(const std::string &_taskID) {};

  double m_MaxLaserRangeForPlaceholders; 
  double m_MaxLaserRangeForCombinedMaps;

  bool m_bNoPlanes;

  IceUtil::Mutex m_Mutex;
  // This grid map represents the current Place
  CharMap* m_lgm1;
  CharGridLineRayTracer* m_Glrt1;
  Cure::XDisplayLocalGridMap<unsigned char>* m_Displaylgm1;

  // This grid map represents a potential new place and
  // is reset each time the robot changes Place.
  CharMap* m_lgm2;
  CharGridLineRayTracer* m_Glrt2;
  Cure::XDisplayLocalGridMap<unsigned char>* m_Displaylgm2;


  //Same as above, for use if we're using different horizons for
  //placeholder property evaluation and combined local map retrieval
  CharMap* m_lgm1_alt;
  CharGridLineRayTracer* m_Glrt1_alt;
  CharMap* m_lgm2_alt;
  CharGridLineRayTracer* m_Glrt2_alt;

  std::map<int, CharMap *> m_nodeGridMaps;
  bool m_isUsingSeparateGridMaps;
  std::map<int, CharMap *> m_nodeGridMapsAlt;

  PlaneMap* m_planeMap;
  CharMap* m_planeObstacleMap;
  Cure::XDisplayLocalGridMap<unsigned char>* m_DisplayPlaneMap;

  std::vector<GridObjectFinder *> m_planeObjectFinders;

  Cure::TransformedOdomPoseProvider m_TOPP;

  bool m_firstScanReceived;

  NavData::RobotPose2dPtr lastRobotPose;

  Cure::Pose3D m_SlamRobotPose;
  Cure::Pose3D m_CurrPose;
  Cure::SensorPose m_LaserPoseR;
  Cure::SensorPose m_CameraPoseR;

  ptz::PTZInterfacePrx m_ptzInterface;

  std::string m_RobotServerHost;
  Robotbase::RobotbaseServerPrx m_RobotServer;
  FrontierInterface::PlaceInterfacePrx m_placeInterface;


private:
  void receiveScan2d(const Laser::Scan2d &castScan);
  void receiveOdometry(const Robotbase::Odometry &castOdom);
  void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
  NavData::FNodePtr getCurrentNavNode();
  FrontierInterface::HypothesisEvaluation getHypothesisEvaluation(int hypID);
  void getCombinedGridMap(FrontierInterface::LocalGridMap &map, 
      const SpatialData::PlaceIDSeq &places);
  Cure::Transformation3D getCameraToWorldTransform();
  void newConvexHull(const cast::cdl::WorkingMemoryChange &objID);
  void PaintPolygon(const VisionData::Vector3Seq &points);
};
}; // namespace spatial


#endif

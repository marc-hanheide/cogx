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

namespace spatial {
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

  IceUtil::Mutex m_Mutex;
  // This grid map represents the current Place
  Cure::LocalGridMap<unsigned char>* m_lgm1;
  Cure::GridLineRayTracer<unsigned char>* m_Glrt1;
  Cure::XDisplayLocalGridMap<unsigned char>* m_Displaylgm1;

  // This grid map represents a potential new place and
  // is reset each time the robot changes Place.
  Cure::LocalGridMap<unsigned char>* m_lgm2;
  Cure::GridLineRayTracer<unsigned char>* m_Glrt2;
  Cure::XDisplayLocalGridMap<unsigned char>* m_Displaylgm2;


  //Same as above, for use if we're using different horizons for
  //placeholder property evaluation and combined local map retrieval
  Cure::LocalGridMap<unsigned char>* m_lgm1_alt;
  Cure::GridLineRayTracer<unsigned char>* m_Glrt1_alt;
  Cure::LocalGridMap<unsigned char>* m_lgm2_alt;
  Cure::GridLineRayTracer<unsigned char>* m_Glrt2_alt;

  std::map<int, Cure::LocalGridMap<unsigned char> *> m_nodeGridMaps;
  bool m_isUsingSeparateGridMaps;
  std::map<int, Cure::LocalGridMap<unsigned char> *> m_nodeGridMapsAlt;

  Cure::TransformedOdomPoseProvider m_TOPP;

  bool m_firstScanReceived;

  Cure::Pose3D m_SlamRobotPose;
  Cure::Pose3D m_CurrPose;
  Cure::SensorPose m_LaserPoseR;

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
};
}; // namespace spatial


#endif

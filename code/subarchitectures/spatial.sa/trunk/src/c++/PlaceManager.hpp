//
// = Filename
//   PlaceManager.hpp
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

#ifndef PlaceManager_hpp
#define PlaceManager_hpp

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialData.hpp>
#include <NavData.hpp>
#include <FrontierInterface.hpp>
#include <string>
#include <list>
#include <map>
#include <utility>

namespace spatial {

typedef int PlaceID;
typedef int NodeID;
typedef int HypID;

class PlaceMapper: public cast::ManagedComponent {

  int m_placeIDCounter;
  int m_hypIDCounter;

  std::map<int, int> m_NodeIDToPlaceIDMap; // For map loading

  IceUtil::Mutex m_mutex;

  struct PlaceMapEntry {
    SpatialData::PlacePtr place;
    NavData::FNodePtr node;
    SpatialData::NodeHypothesisPtr hyp;

    std::string placeWMID;
    //	std::string nodeWMID;
    std::string hypWMID;
  };

  std::vector<PlaceMapEntry> entries;

public:

  const SpatialData::PlacePtr _getPlace(PlaceID id);

  PlaceID _getPlaceIDForNode(NodeID id);
  const NavData::FNodePtr _getNodeForPlace(PlaceID id);

  PlaceID _getPlaceIDForHyp(HypID id);
  const SpatialData::NodeHypothesisPtr _getHypForPlace(PlaceID id);

  std::string _getPlaceWMIDForPlace(PlaceID id);
  //    std::string _getNodeWMIDForPlace(PlaceID id);
  std::string _getHypWMIDForPlace(PlaceID id);

  void _getPlaceholders(std::vector<PlaceID> &ret);
  void _getTruePlaces(std::vector<PlaceID> &ret);

  void _overwriteHypForPlace(PlaceID placeID,
      SpatialData::NodeHypothesisPtr hyp);
  void _updateNodeForPlace(PlaceID placeID, NavData::FNodePtr node);

  void _deletePlace(PlaceID id);
  //    void _deleteHypothesis(PlaceID id);

  void _upgradePlaceholderMappings(PlaceID id, SpatialData::PlacePtr place,
      NavData::FNodePtr node);

  PlaceID _addPlaceWithNode(SpatialData::PlacePtr _place,
      NavData::FNodePtr _node);//, string _nodeWMID);
  PlaceID _addPlaceWithHyp(SpatialData::PlacePtr _place,
      SpatialData::NodeHypothesisPtr _node);

  void _checkConsistency(int line);

protected:
  PlaceMapper();
  void LoadPlaces(const std::string &filename);
  void SavePlaces();
};

/**
 * Component that maintains Place structs on WM, and fills them with
 * the necessary information.
 * 
 * Parameters: --max-frontier-dist x : Culling distance for frontiers around a certain node
 *		--min-frontier-length x : Minimum size of exploration frontiers to consider
 *		--min-node-separation x : Spread of new hypothetical targets around a node
 *		--no-local-maps : disables PlaceholderPlaceProperty maintenance and connection to LocalMapManager
 *
 * @author Patric Jensfelt
 * @see
 */
class PlaceManager: public PlaceMapper {
private:

  class PlaceServer: public FrontierInterface::PlaceInterface {
    virtual SpatialData::PlacePtr getPlaceFromNodeID(int nodeID,
        const Ice::Current &_context);
    virtual SpatialData::PlacePtr getPlaceFromHypID(int hypID,
        const Ice::Current &_context);
    virtual SpatialData::NodeHypothesisPtr getHypFromPlaceID(int placeID,
        const Ice::Current &_context);
    virtual NavData::FNodePtr getNodeFromPlaceID(int placeID,
        const Ice::Current &_context);
    virtual void beginPlaceTransition(int goalPlaceID,
        const Ice::Current &_context);
    virtual SpatialData::PlacePtr getCurrentPlace(const Ice::Current &_context);
    virtual FrontierInterface::PlaceMembership getPlaceMembership(double x,
        double y, const Ice::Current &_context);
    int updatePlaceholderEdge(int placeholderId, const Ice::Current &_context);
    FrontierInterface::AdjacencyLists getAdjacencyLists(
        const Ice::Current &_context);

    PlaceManager *m_pOwner;
    PlaceServer(PlaceManager *owner) :
      m_pOwner(owner) {
    }
    friend class PlaceManager;
  };
  friend class PlaceServer;
public:
  /**
   * Constructor
   */
  PlaceManager();

  /**
   * Destructor
   */
  virtual ~PlaceManager();

  virtual void start();
  virtual void stop();
  virtual void runComponent();
  virtual void configure(const std::map<std::string, std::string>& _config);

protected:
  void
  newEndPlaceTransitionCommand(const cast::cdl::WorkingMemoryChange &objID);
  // Call back functions for nodes 
  void newNavNode(const cast::cdl::WorkingMemoryChange &objID);
  void modifiedNavNode(const cast::cdl::WorkingMemoryChange &objID);
  void deletedNavNode(const cast::cdl::WorkingMemoryChange &objID);

  // Callback function for detected objects
  void newObject(const cast::cdl::WorkingMemoryChange &objID);

  // Callback function for new door hypothesis
  void newDoorHypothesis(const cast::cdl::WorkingMemoryChange &objID);

  // Call back functions for edges 
  void newEdge(const cast::cdl::WorkingMemoryChange &objID);
  void modifiedEdge(const cast::cdl::WorkingMemoryChange &objID);

  // Call back function for overwrites on MapLoadStructs
  void mapLoadStatusOverwritten(const cast::cdl::WorkingMemoryChange &wmc);

  void evaluateUnexploredPaths();
  IceUtil::Mutex m_PlaceholderMutex;

  //    std::vector<std::pair <double,double> > getPlaceholderPositionsFromFrontiers(FrontierInterface::FrontierPtSeq frontiers, int placeId);

  void updateReachablePlaceholderProperties(int placeID);
  //    void updatePlaceholderPositions(FrontierInterface::FrontierPtSeq frontiers);
  SpatialData::PlacePtr getCurrentPlace();
  NavData::FNodePtr getCurrentNavNode();
  void beginPlaceTransition(int goalPlaceID);

private:

  void LoadConnectivityProperties(const std::string &filename);
  void SaveConnectivityProperties();
  bool m_usePeekabot;
  int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry
  peekabot::PeekabotClient m_PeekabotClient;
  peekabot::GroupProxy m_ProxyForbiddenMap;
  std::string m_PbHost;
  int m_PbPort;
  bool m_robotInitialPoseReceived;
  double m_robotInitialX;
  double m_robotInitialY;
  double m_initialMovementThreshold;
  bool m_placesWritten;

  void connectPeekabot();

  //    SpatialData::PlacePtr getPlaceFromNodeID(int nodeID);
  //    SpatialData::PlacePtr getPlaceFromHypID(int hypID);
  //    SpatialData::NodeHypothesisPtr getHypFromPlaceID(int placeID);
  //    NavData::FNodePtr getNodeFromPlaceID(int placeID);
  FrontierInterface::PlaceMembership getPlaceMembership(double x, double y);
  //    void refreshPlaceholders(std::vector<std::pair<double,double> > coords);

  // Callback function for metric movement
  void robotMoved(const cast::cdl::WorkingMemoryChange &objID);
  void processPlaceArrival(bool failed);
  void processNewNode(const NavData::FNodePtr node);

  void upgradePlaceholder(int placeID, NavData::FNodePtr newNode);
  void deletePlaceProperties(int placeID);
  void deletePlaceholderProperties(int placeID);
  void deletePlaceholder(int placeId);
  //    bool createPlaceholder(int curPlaceId, double x, double y);

  int updatePlaceholderEdge(int placeholderId);
  //    bool isPointCloseToExistingPlaceholder(double x, double y, int curPlaceId);

  std::map<int, std::vector<int> > getAdjacencyLists();

  double getGatewayness(double x, double y);
  void setOrUpgradePlaceholderGatewayProperty(int hypothesisID, double value);

  std::string concatenatePlaceIDs(int place1ID, int place2ID);
  void createConnectivityProperty(double cost, int place1ID, int place2ID);
  bool deleteConnectivityProperty(int place1ID, int place2ID);
  // Helper function to create Gateway properties
  void addNewGatewayProperty(int placeID);

  /* Helper function for adding a place to a node */
  int addPlaceForNode(NavData::FNodePtr node);

  // Methods to deal with synchronisation of edge addition
  void checkUnassignedEdges(int newNodeID);
  void processEdge(NavData::AEdgePtr edge);

  // Frontier based exploration path parameters
  double m_minFrontierDist; // Culling distance for frontiers around a given node
  double m_minNodeSeparation; // Min distance that has to exist between the new
  // hypothetical nodes generated at a place

  double m_hypPathLength; // How far to try and move in the direction of the
  // frontier
  bool m_useLocalMaps; // Whether to connect to the LocalMapManager and
  // generate PlaceholderPlaceProperties
  bool m_bNoPlaceholders;

  //    bool m_updatePlaceholderPositions;

  //List of hypotheses that have already been tried and failed from each Node
  std::vector<std::pair<int, SpatialData::NodeHypothesisPtr> >
      m_rejectedHypotheses;
  bool m_isPathFollowing;
  bool m_nickSanity;
  int m_startNodeForCurrentPath; // During transitions, stores where the robot
  // came from last
  int m_goalPlaceForCurrentPath; // During transitions, stores where the robot
  // thought it was going
  int m_currentNodeOnPath; // During path following, stores the current node the robot is on

  std::map<int, std::set<int> > m_connectivities; // Keeps track of the
  // connectivity properties maintained
  std::map<std::string, std::string> m_placeIDsToConnectivityWMID;
  std::list<std::pair<int, int> > m_hypotheticalConnectivities; // Keeps track of hypothetical connectivities
  // which may be deleted on upgrading placeholders

  std::map<int, std::string> m_gatewayProperties;
  IceUtil::Mutex m_PlacePropsMutex;

  struct ForbiddenZone {
    double minX;
    double maxX;
    double minY;
    double maxY;
  };
  std::vector<ForbiddenZone> m_forbiddenZones;

  //Maps from placeID 
  std::map<int, std::string> m_placeholderGatewayProperties;
  std::map<int, std::string> m_freeSpaceProperties; // Keeps track of the
  // freespace placeholder properties maintained
  std::map<int, std::string> m_borderProperties; // Keeps track of the
  // border placeholder properties maintained
  bool m_firstMovementRegistered;
  bool m_foundNodeThisMovement;

  std::set<NavData::AEdgePtr> m_unprocessedEdges;

  FrontierInterface::FrontierReaderPrx frontierReader;
  FrontierInterface::HypothesisEvaluatorPrx hypothesisEvaluator;
  SpatialData::MapInterfacePrx m_mapInterface;
}; // class PlaceManager

}
; // namespace spatial

#endif // PlaceManager_hpp

//
// = FILENAME
//    NavGraph.hh
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef NavGraph_hh
#define NavGraph_hh


#ifndef DEPEND
#include <sstream>
#include <list>
#include <map>
#endif

#include <Utils/RoboLookProxy.h>
#include <Navigation/NavArea.hh>

namespace Cure {

// Forward declaration
class NavGraphNode;
class NavGraphEdge;
class NavGraphGateway;


/**
 * Defines an interface for classes that wants to trap evens such a
 * new nodes, new gateways, etc
 *
 * @author Patric Jensfelt
 *
 * @see NavGraph::addEventListener
 * @see NavGraph::delEventListener
 */
class NavGraphEventListener {
public:
  NavGraphEventListener(const std::string &name);
  virtual ~NavGraphEventListener();
  virtual void areaIdConflict(NavGraphNode &currNode, NavGraphGateway &gw) {}
  virtual void newNode(NavGraphNode &n) {}
  virtual void changedArea(int aid) {}
  virtual void changedCurrentNode(int fromId, int toId) {}
  virtual void newGateway(NavGraphGateway &gw) {}
  virtual void mergedAreaIds(int aid1, int aid2) {}
  virtual void changedNodeType(int id) {}
  std::string getName() const { return m_Name; }
protected:
  std::string m_Name;
};

/**
 * This class defines a free space navigation graph. The idea is that
 * the robt calls the addFreePose fnction as it moves through th
 * eenvironment. Every now and then a new node will be added to the
 * graph. Using these nodes the robot then now how the different parts
 * of the environment are connected.
 *
 * @author Patric Jensfelt
 * @see
 */
class NewNavGraph {
public:

  // List of nodes
  std::list<NavGraphNode*> m_Nodes;

  // List of edges
  std::list<NavGraphEdge> m_Edges;

  // List of pointers to gateway nodes.
  std::list<NavGraphGateway*> m_Gateways;

  // The list of areas
  std::map<int, NavArea> m_Areas;

public:

  /**
   * Constructor
   *
   * @param nodeDist the typical minimum distance between nodes
   */
  NewNavGraph(double nodeDist = 1.0);

  /**
   * Destructor
   */
  ~NewNavGraph();

  /**
   * Clear the navigation graph
   */
  int clear();  

  /**
   * Specify whether the NewNavGraph should automatically try to fix
   * problems with conflicting node types for example. The is to send
   * a request to any NavGraphEventListener that might be listening
   */
  void setAutoFix(bool autofix);

  /**
   * Call this function to possibly update the navigation graph with a
   * new robot position. If it is updated depends on if we are far
   * enough from any other node. We might also change the current node
   * to an existing node.
   * 
   * @return 1 if node was added, 2 if node was modified, 0 if nothing
   * happened, else error code <0
   */
  int maybeAddNewNodeAt(double x, double y, double a);

  /**
   * DEPRECATED AS IT IS NOT OBVIOUS FROM THE NAME THAT A NODE MIGHT
   * NOT BE ADDED
   * 
   * Call this function to possibly update the navigation graph with a
   * new robot position. If it is updated depends on if we are far
   * enough from any other node. We might also change the current node
   * to an existing node.
   * 
   * @return 0 always
   */
  int addFreePose(double x, double y, double a);

  /**
   * Call this function to add a gateway node at the specified pose.
   * 
   * @param x x-coordinate for gateway center [m]
   * @param y y-coordinate for gateway center [m]
   * @param a orientation of the gateway [rad]
   * @param width width of the gateway
   *
   * @return 0 if ok, else error code
   */
  int addGatewayNode(double x, double y, double a, double width);

  /**
   * @return id of current node, -1 if error
   */
  int getCurrentNodeId() const;

  /**
   * Use this function to set the current node id. This is useful when
   * you start the system with an existing graph and want to put the
   * robot at an existing node.
   *
   * @return the id of the current node after the operation
   */
  int setCurrentNodeId(int id);

  /**
   * @return type of current node, -1 if error
   */
  int getCurrentNodeType() const;

  /**
   * @return class of current area, -1 if error
   */
  int getCurrentNodeAreaClass() const;

  /**
   * @return 0 is ok, else error code
   */
  int setCurrentNodeAreaClass(int c);

  /**
   * @return id of current area, -1 if error
   */
  long getCurrentAreaId() const;

  /**
   * Call this function to change a gateway node into being a normal
   * node. You can choose to make the area_id on the two sides the
   * same with an extra flag
   *
   * @param id of gateway to change to a normal node
   * @param uniteAreaIds true if you want area ids on both side to be same
   * @param idToKeep if non-negative use to specify which of the two ids 
   *        to keep (default oldest one)
   */
  int turnGatewayIntoNode(int id, bool uniteAreaIds, long idToKeep = -1);

  
  /**
   * Use this function to plan a path from one node to another
   * @return o if ok, else error code
   */
  int findPath(NavGraphNode *from, NavGraphNode *to,
               std::list<NavGraphNode> &path, double *cost);

  /**
   * Use this function to plan a path from one node to another
   * @return o if ok, else error code
   */
  int findPath(NavGraphNode *from, int toNodeId,
               std::list<NavGraphNode> &path, double *cost);

  /**
   * Use this function to plan a path from one node to another
   * @return o if ok, else error code
   */
  int findPath(int fromNodeId, NavGraphNode *to,
               std::list<NavGraphNode> &path, double *cost);

  /**
   * Use this function to plan a path from one node to another
   * @return o if ok, else error code
   */
  int findPath(int fromNodeId, int toNodeId,
               std::list<NavGraphNode> &path, double *cost);

  /**
   * Use this function to plan a path from a node to a pose defined by
   * x,y,a. Unless the goal is VERY close to an existing node it will
   * generate a temporary node with id -42 and name TEMPGOAL. The cost
   * to move from the last node in the graph to the TEMPGOAL node will
   * be estimated as 0 as we do not know what meteric is used for the
   * cost.
   *
   * @return o if ok, else error code
   */
  int findPath(NavGraphNode *from, double xG, double yG, double aG,
               std::list<NavGraphNode> &path, double *cost);
  
  /**
   * Same as above but will look up the first node to move to based on
   * the start position.
   *
   * @return o if ok, else error code
   */
  int findPath(double xS, double yS, double aS,
               double xG, double yG, double aG,
               std::list<NavGraphNode> &path, double *cost);

  /**
   * Same as above but will look up the first node to move to based on
   * the start position.
   *
   * @return o if ok, else error code
   */
  int findPath(double xS, double yS, double aS, NavGraphNode *to,
               std::list<NavGraphNode> &path, double *cost);

  /**
   * Remove a node from the graph, also making sure that the edges to
   * that node are deleted
   */
  int deleteNode(int id);
  int deleteNode(const std::string &name);
  int deleteNode(NavGraphNode *node);

  /**
   * @return pointer to the closest node to a certain pose, 0 if no
   * node within found within maxDist (maxDist<0 -> no limit)
   */
  NavGraphNode* getClosestNode(double x, double y, double a=0, 
                               double maxDist = 3);
  
  /**
   * Call this function if you want to drop a node at a certain position
   * 
   * @return 0 if ok, else error code
   */
  int markPose(double x, double y, double a, const std::string &tag);

  NavGraphNode* getNode(const std::string &name);
  NavGraphNode* getNode(const long id);
  NavGraphEdge* getEdge(const long id1, const long id2);
  NavGraphGateway* getGateway(const long id);

  /**
   * Use this function to get all gateways in a certain area
   */
  std::list<NavGraphGateway*> getGatewaysInArea(const long aid);


  /**
   * Helper function that adds a node to the list of nodes given the
   * parameters needed to define it. Can be used when reading node
   * data from files or some other media to create a graph.
   *
   * NOTE you need to call connectNodes after calling
   * addEdgeToEdgeList, addDoorToNodeList or addNodeToNodeList since
   * these functions do not make the necessary internal connections
   *
   * @param id id of the ndoe
   * @param areaId the id of the area that this node is in
   * @param x x-coordinate of the node
   * @param y y-coordinate of the node
   * @param a the orientatio of the node
   * @param name the name of the node (should always be "-" unless a
   * special destination or something similar
   * @param maxspeed the max speed the robot is allowed to drive near this node
   */
  int addNodeToNodeList(int id, int areaId,
                        double x, double y, double a,                       
                        const std::string &name = "-",
                        double maxspeed = 2);

  /**
   * Helper function that adds a node of door type to the list of
   * nodes given the parameters needed to define it. Can be used when
   * reading node data from files or some other media to create a
   * graph.
   *
   * NOTE that you should only call this function for a door and not
   * the addNodeToNodeList
   * 
   * NOTE you need to call connectNodes after calling
   * addEdgeToEdgeList, addDoorToNodeList or addNodeToNodeList since
   * these functions do not make the necessary internal connections
   *
   * @param id id of the ndoe
   * @param areaId the id of the area that this node is in
   * @param x x-coordinate of the node
   * @param y y-coordinate of the node
   * @param a the orientatio of the node
   * @param width the width of the door
   * @param name the name of the node (normally "-")
   * @param maxspeed the max speed the robot is allowed to drive near this node
   */
  int addDoorToNodeList(int id, int areaId,
                        double x, double y, double a, double width,
                        const std::string &name = "-",
                        double maxspeed = 1);

  /**
   * Helper function that adds a node to the list of nodes given the
   * parameters needed to define it
   *
   * NOTE you need to call connectNodes after calling
   * addEdgeToEdgeList, addDoorToNodeList or addNodeToNodeList since
   * these functions do not make the necessary internal connections
   *
   * @param id1 id of the first node
   * @param id2 id of the second node
   */
  int addEdgeToEdgeList(int id1, int id2);

  /**
   * Read graph from a file
   *
   * @return 0 if OK, else error code
   */
  int loadGraph(const std::string &filename);

  /**
   * Save graph to file
   *
   * @return 0 if OK, else error code
   */
  int saveGraph(const std::string &filename);

  /**
   * Reads the data for a certain NewNavGraph from an input stream
   *
   * @return 0 if OK, else error code
   */
  int readFromStream(std::istream &is);

  /**
   * Write the graph to a stream
   */
  int writeToStream(std::ostream &os);

  /**
   * Goes through the list of edges and updates the connection info
   * for the nodes
   *
   * @return 0 if ok, else error code
   */
  int connectNodes();

  void addEventListener(NavGraphEventListener *l);
  void delEventListener(NavGraphEventListener *l);

  void displayRL(RoboLookProxy *rlp, int env = RL_ENV_EST, 
                 bool clearStars = false, bool clearLines = false,
                 bool showDoorFrames = false);

  bool setAreaName(int areaId, const std::string &name);
  bool setAreaClass(int areaId, int c);

  std::string getAreaName(int areaId) const;
  int getAreaClass(int areaId);

  /**
   * Call this function to set all gateway states to UNKNOWN
   */
  bool setGatewayStatesToUnknown();

  /**
   * Change the size of the star display in RoboLook. a value -1 means
   * no change to that star type
   */
  void configRLStars(double sizeCurrent = -1,
                     double sizeNormal = -1,
                     double sizeNamedGoal = -1,
                     double sizeGateway = -1,
                     double sizeGatewayBlocked = -1);


  /**
   * Use this function to set the distance that the target can be from
   * any node in the graph for the robot still to consider a path to
   * be found to it
   */
  void setMaxDistTargetFromNode(double d) { m_MaxDist = d; }

  double getMaxDistTargetFromNode() const { return m_MaxDist; }
protected:

  /**
   * Connect two nodes and give the edge between them a certain cost
   */
  int connectNodes(int id1, int id2, double cost);


  /**
   * Same as other addFreePose but here you have the option to force
   * in a pose if you want.
   *
   * @return 1 if node was added, 2 if node was modified, 0 if nothing
   * happened, else error code <0
   */
  int maybeAddNewNodeAt(double x, double y, double a,
                        bool forceInNode, int type);

  /**
   * Replace one node object with another shifting all connections
   * from the old to the new node
   */
  int replaceNode(NavGraphNode *origNode, NavGraphNode *newNode);

  /**
   * Turn an existing node into a gateway node 
   *
   * @return 0 if ok, else error code
   */
  int makeGatewayOutOfNode(NavGraphNode *node, 
                           double x, double y, double a, double width);

  /**
   * Turn a node misclassified as a gateway into a node again
   *
   * @return 0 if ok, else error code
   */
  int makeNodeOutOfGateway(NavGraphGateway *g, double x, double y, double a);

  /**
   * Will change all area ids from aid2 into aid1
   */
  int mergeAreaIds(long aid1, long aid2);


  /**
   * Finds gateway connecting two areas. Will only return the first
   * one it finds.
   */
  std::list<NavGraphGateway*>::iterator findConnectingGateway(long aid1, 
                                                              long aid2);

  /**
   * Will turn gateways connecting nodes with area id aid1 and aid2
   * into normal nodes
   */
  int replaceGatewaysConnecting(long aid1, long aid2);

  /**
   * Function that determines if there is a path to the goal node
   * without passing a gateway and any node that has already been
   * visited (costG!=0). Uses recursion
   */
  bool foundPathToNode(NavGraphNode *goal, NavGraphNode *node);
  
  /**
   * Uses recursion to spread a new area id to all nodes not yet
   * marked as visited (costG!=0) visited.
   */
  void spreadNewAreaId(long areaId, NavGraphNode *node);

  /**
   * Use this function to let push clients know that the area has changed
   */
  void notifyChangedArea(long areaId);

  /**
   * Use this function to let push clients know that the current node
   * has changed
   */
  void notifyChangedCurrentNode(long fromId, long toId);

protected:
  /// The typical minimum distance between nodes
  double m_NodeDist;

  /// The max distance from a node to a target location to find a path
  /// there
  double m_MaxDist;

  /// The current node that the robot is at
  NavGraphNode *m_CurrNode;

  /// Next id to assign to a node
  long m_NextNodeId;

  /// The default max speed to assign to a node
  double m_MaxSpeed;

  // The id of the last area 
  long m_LastAreaId;

  // False as long as a new area id has not yet been assigned. This is
  // used to detect if we come trough a door start creating nodes with
  // new areaids and then all of a sudden jump to a new id without
  // passing a door. This either means that ther are two door or that
  // we created a new node by mistake entering a room, maybe because
  // we entered with a very different angle
  bool m_AssignedNewAreaId;

  bool m_AutoFixGateway;

  double m_RLStarSizeCurrent;
  double m_RLStarSizeNormal;
  double m_RLStarSizeNamedGoal;
  double m_RLStarSizeGateway;
  double m_RLStarSizeGatewayBlocked;

  std::list<NavGraphEventListener*> m_EventListeners;
};

}; // namespace Cure

std::ostream&
operator<< (std::ostream &os, const Cure::NewNavGraph &a);


#endif // NavGraph_hh

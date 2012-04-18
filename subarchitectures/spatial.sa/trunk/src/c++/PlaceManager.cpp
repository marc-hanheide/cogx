//
// = Filename
//   PlaceManager.cpp
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

#include "PlaceManager.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <Navigation/NavGraphNode.hh>
#include <Math/BinaryMatrix.hh>
#include <FrontierInterface.hpp>
#include <SpatialProperties.hpp>
#include <Rendezvous.h>
#include <Ice/Ice.h>
#include <float.h>
#include <limits>

//HACK: Constant value even w/o nearby gateways
#define GATEWAY_FUNCTION(x) (exp(-x/1.0))

#define DO_CONSISTENCY_CHECK

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;
using namespace Ice;
using namespace SpatialData;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PlaceManager();
  }
}

PlaceMapper::PlaceMapper() :
    m_placeIDCounter(0), 
    m_hypIDCounter(0)
{
}

PlaceManager::PlaceManager() : PlaceMapper(),
    m_robotInitialPoseReceived(false),
    m_initialMovementThreshold(0.05),
    m_isPathFollowing(false),
    m_startNodeForCurrentPath(-1),
    m_goalPlaceForCurrentPath(-1),
    m_currentNodeOnPath(-1),
    m_firstMovementRegistered(false)
{
  cout<<"PlaceManager::PlaceManager()"<<endl;
}

PlaceManager::~PlaceManager()
{
}

void
PlaceManager::configure(const std::map<std::string, std::string>& _config)
{
  log("Configure entered");

  m_usePeekabot = false;
  if (_config.find("--usepeekabot") != _config.end()) {
    m_usePeekabot= true;

    connectPeekabot();
  }

  if(_config.find("--no-local-maps") != _config.end()) {
    m_useLocalMaps = false;
  }
  else {
    m_useLocalMaps = true;
  }

  if(_config.find("--min-frontier-dist") != _config.end()) {
    std::istringstream str(_config.find("--min-frontier-dist")->second);
    str >> m_minFrontierDist;
  }
  else {
    m_minFrontierDist = 0.5;
  }

  if(_config.find("--min-node-separation") != _config.end()) {
    std::istringstream str(_config.find("--min-node-separation")->second);
    str >> m_minNodeSeparation;
  }
  else {
    m_minNodeSeparation = 2.0;
  }

  m_bNoPlaceholders = false;
  if(_config.find("--no-placeholders") != _config.end()) {
    m_bNoPlaceholders = true;
  }

  m_RetryDelay = 1000;
  if(_config.find("--retry-interval") != _config.end()){
    std::istringstream str(_config.find("--retry-interval")->second);
    str >> m_RetryDelay;
  }

//  m_updatePlaceholderPositions = true;
//  if(_config.find("--no-update-placeholders") != _config.end()) {
//    m_updatePlaceholderPositions = false;
//  }

  if(_config.find("--exclude-from-exploration") != _config.end()) {
    std::istringstream str(_config.find("--exclude-from-exploration")->second);



    m_PbPort = 5050;
    m_PbHost = "localhost";

    ForbiddenZone newZone;
    newZone.minX = -DBL_MAX;
    newZone.maxX = DBL_MAX;
    newZone.minY = -DBL_MAX;
    newZone.maxY = DBL_MAX;
    while (!str.eof()) {
      string buf;
      str >> buf;

      if (buf == "or") {
    	    println("new forbidden zone: %.02g, %.02g, %.02g, %.02g", newZone.minX, newZone.minY, newZone.maxX, newZone.maxY);

	m_forbiddenZones.push_back(newZone);
	newZone.minX = -DBL_MAX;
	newZone.maxX = DBL_MAX;
	newZone.minY = -DBL_MAX;
	newZone.maxY = DBL_MAX;
      }
      else if (buf == "x") {
	str >> buf;
	if (buf == ">") {
	  str >> newZone.minX;
	}
	else if (buf == "<") {
	  str >> newZone.maxX;
	}
	else {
	  log("Warning: Malformed --exclude-from-exploration string %s", _config.find("--exclude-from-exploration")->second.c_str());
	  break;
	}
      }
      else if (buf == "y") {
	str >> buf;
	if (buf == ">") {
	  str >> newZone.minY;
	}
	else if (buf == "<") {
	  str >> newZone.maxY;
	}
	else {
	  log("Warning: Malformed --exclude-from-exploration string", _config.find("--exclude-from-exploration")->second.c_str());
	  break;
	}
      }
      else {
	log("Warning: Malformed --exclude-from-exploration string", _config.find("--exclude-from-exploration")->second.c_str());
	break;
      }
    }
    println("new forbidden zone: %.02g, %.02g, %.02g, %.02g", newZone.minX, newZone.minY, newZone.maxX, newZone.maxY);

    m_forbiddenZones.push_back(newZone);
  }

  if(_config.find("--hyp-path-length") != _config.end()) {
    std::istringstream str(_config.find("--hyp-path-length")->second);
    str >> m_hypPathLength;
  }
  else {
    m_hypPathLength = 1.5;
  }


  LoadPlaces("Places.txt");

  FrontierInterface::PlaceInterfacePtr servant = new PlaceServer(this);
  registerIceServer<FrontierInterface::PlaceInterface, FrontierInterface::PlaceInterface>(servant);
}

void 
PlaceManager::start()
{
    if(m_usePeekabot){
        while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
            sleepComponent(m_RetryDelay);
            connectPeekabot();
        }
        m_ProxyForbiddenMap.add(m_PeekabotClient, "pm_forbidden",peekabot::REPLACE_ON_CONFLICT);
        m_ProxyForbiddenMap.set_position(0,0,-0.004);

        for (vector<ForbiddenZone>::iterator fbIt = m_forbiddenZones.begin();
            fbIt != m_forbiddenZones.end(); fbIt++) {
            peekabot::PolygonProxy* p = new peekabot::PolygonProxy();
            
	    peekabot::VertexSet vs;
            p->add(m_ProxyForbiddenMap, "zone");
            p->set_color(1,0.1,0.1);
            vs.add( fbIt->minX < -10 ? -10 : fbIt->minX, fbIt->minY < -10 ? -10 : fbIt->minY, 0 );
            vs.add( fbIt->minX < -10 ? -10 : fbIt->minX, fbIt->maxY > 10 ? 10 : fbIt->maxY, 0 );
            vs.add( fbIt->maxX > 10 ? 10 : fbIt->maxX, fbIt->maxY > 10 ? 10 : fbIt->maxY, 0 );
            vs.add( fbIt->maxX > 10 ? 10 : fbIt->maxX, fbIt->minY < -10 ? -10 : fbIt->minY, 0 );
	    p->add_vertices(vs);
        }
	}

  addChangeFilter(createLocalTypeFilter<NavData::EndPlaceTransitionCommand>(cdl::ADD),
		  new MemberFunctionChangeReceiver<PlaceManager>(this,
								  &PlaceManager::newEndPlaceTransitionCommand));

  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::newNavNode));

  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::modifiedNavNode));

  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::DELETE),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::deletedNavNode));
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::robotMoved));
  addChangeFilter(createLocalTypeFilter<NavData::AEdge>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::newEdge));
  addChangeFilter(createLocalTypeFilter<NavData::AEdge>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::modifiedEdge));
  addChangeFilter(createLocalTypeFilter<NavData::ObjData>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::newObject));
  addChangeFilter(createLocalTypeFilter<NavData::ObjData>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::newObject));
  addChangeFilter(createLocalTypeFilter<FrontierInterface::DoorHypothesis>(cdl::ADD),
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::newDoorHypothesis));
  addChangeFilter(createLocalTypeFilter<MapLoadStatus>(cdl::OVERWRITE), 
      new MemberFunctionChangeReceiver<PlaceManager>(this,
	&PlaceManager::mapLoadStatusOverwritten));

  frontierReader = FrontierInterface::FrontierReaderPrx(getIceServer<FrontierInterface::FrontierReader>("spatial.control"));
  m_mapInterface = MapInterfacePrx(getIceServer<MapInterface>("spatial.control"));
  if (m_useLocalMaps) {
    hypothesisEvaluator = FrontierInterface::HypothesisEvaluatorPrx(getIceServer<FrontierInterface::HypothesisEvaluator>("map.manager"));
  }
  else {
    hypothesisEvaluator = 0;
  }

}

void 
PlaceManager::stop()
{
}

void PlaceMapper::SavePlaces(){
  ofstream fout("Places.txt");
  for(vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->node != 0) { // TODO: Save placeholders too?
      int PlaceID = it->place->id;
      int NodeID = it->node->nodeId;
      fout << PlaceID << " " << NodeID << endl;
    }
  }
}

void PlaceMapper::LoadPlaces(const std::string &filename){
  ifstream file(filename.c_str());
  if (!file.good()){
    log("Could not read places file, exiting.");
    return;
  }
  string line,tmp;

  while(!getline(file,line).eof()){
    istringstream istr(line); 
    istr >> tmp;
    int PlaceID = atoi(tmp.c_str());
    istr >> tmp;
    int NodeID = atoi(tmp.c_str());
    m_NodeIDToPlaceIDMap[NodeID]=PlaceID;
  }
}


void 
PlaceManager::runComponent()
{
  log("PlaceManager::runComponent: now at start of runComponent");

  Rendezvous *rv = new Rendezvous(*this);
  rv->addChangeFilter(
      createLocalTypeFilter<NavData::FNode>(cdl::ADD));
  cdl::WorkingMemoryChange change = rv->wait();
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(change.address);

  int count = 0;
  while(isRunning()) {
    count = (count + 1) % 12;
    sleepComponent(100);
    if(count == 0) {
      SavePlaces();
      // Execute this every 5 seconds.  However we don't want to sleep for so
      // long to be able to react to the component being stopped, thus we break
      // it up into sleeps of 100ms.
//      evaluateUnexploredPaths();
    }
  }
}

void 
PlaceManager::newNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  try {
    NavData::FNodePtr oobj =
      getMemoryEntry<NavData::FNode>(objID.address);
    log("newNavNode called (%s)", objID.address.id.c_str());
//    log("Nav Node is: %li, (%f,%f:%f), %i on %s", 
//	oobj->nodeId, oobj->x, oobj->y, oobj->theta, oobj->gateway,
//	objID.address.id.c_str());

    if (m_firstMovementRegistered) {
    	log("");
      
      if (oobj != 0) {
	processPlaceArrival(false);
      }
    }
    else {
      
      // Special case: Robot hasn't yet moved; this must be a loaded map
      // node. Just add it as a Place.
      PlacePtr p = new Place;   
      p->status = TRUEPLACE;
      //p.m_data->id = oobj->getData()->nodeId;
      
      PlaceID newPlaceID = _addPlaceWithNode(p, oobj);

      checkUnassignedEdges(oobj->nodeId);

      if(oobj->gateway == 1) {
	addNewGatewayProperty(newPlaceID);			
      }

    }
  }
  catch (DoesNotExistOnWMException) {
    log("Error! Nav node missing from WM!");
  }
  log("newNavNode exited");
}

void
PlaceManager::cancelMovement(bool failed = false)
{
  log("CancelMovement(%i) called", failed);

  m_goalPlaceForCurrentPath = -1;
  m_isPathFollowing = false;

  // Stop the robot
  vector<CASTData<NavCommand> > commands;
  getMemoryEntriesWithData<NavCommand>(commands, "spatial.sa");
  for (vector<CASTData<NavCommand> >::iterator it =
      commands.begin(); it != commands.end(); it++) {
    try {
      log("locking");
      lockEntry(it->getID(), cdl::LOCKEDOD);
      log("locked");
      //Re-read after lock
      NavCommandPtr ptr = getMemoryEntry<NavCommand>(it->getID());
      if (ptr->cmd == GOTOPLACE &&
          ptr->comp == COMMANDINPROGRESS) {
        ptr->status = failed ? TARGETUNREACHABLE : UNKNOWN;
        log("overwrite 1: %s", it->getID().c_str());
        overwriteWorkingMemory<NavCommand>(it->getID(), ptr);
      }
      log("unlocking");
      unlockEntry(it->getID());
    }
    catch (IceUtil::NullHandleException e) {
      //Just ignore it, it obviously doesn't need to be cancelled
    }
    catch (cast::DoesNotExistOnWMException e) {
    }
    catch (cast::ConsistencyException e) {
      log("Error! ConsistencyException in cancelMovement!");
    }
  }
  log("CancelMovement exited");
}

//FIXME: evaluateUnexploredPaths shouldn't be called in the message thread!
void PlaceManager::newEndPlaceTransitionCommand(const cdl::WorkingMemoryChange &objID) 
{
  log("Received new EndPlaceTransitionCommand (%s)", objID.address.id.c_str());
  try {
    NavData::EndPlaceTransitionCommandPtr obj =
      getMemoryEntry<NavData::EndPlaceTransitionCommand>(objID.address);
    if (m_isPathFollowing) {
      log("  We were still trying to follow a path; must have failed");
      m_isPathFollowing = false;
      // If successful and the transition was unexplored, 
      // check current node.
      // If the node is different from the previous,
      // change the Place struct to non-placeholder,
      // remove the NodeHypothesis struct and change the
      // m_PlaceToNodeIDMap/m_PlaceToHypIDMap members
      processPlaceArrival(obj->failed);
    }
    if (obj->generatePlaceholders){ 
      evaluateUnexploredPaths();
    }
    obj->comp = NavData::SUCCEEDED;
    overwriteWorkingMemory<NavData::EndPlaceTransitionCommand>(objID.address, obj);
  }
  catch (DoesNotExistOnWMException) {
    log("Could not find PlaceholderEnumeratingCommand on WM!");
  }
}


void 
PlaceManager::modifiedNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  log("modifiedNavNode called (%s)", objID.address.id.c_str());
  try {
    NavData::FNodePtr oobj =
      getMemoryEntry<NavData::FNode>(objID.address);

    if (oobj != 0) {

      PlaceID correspondingPlaceID = _getPlaceIDForNode(oobj->nodeId);

      const NavData::FNodePtr prev = _getNodeForPlace(correspondingPlaceID);
      if (prev == 0) {
	// If the node is not in our 
	log("Did not find the node from before, have to assume that we did not start early enough to catch it, will treat it as new");
	newNavNode(objID);

//	processPlaceArrival(false);
	log("modifiedNavNode exited");
	return;
      }

      if (prev->gateway == 0 && oobj->gateway == 1) {
	// Has gained gateway status; add gateway property to WM
	addNewGatewayProperty(correspondingPlaceID);
      }

      /*
	 log("Modified place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
	 overwriteWorkingMemory<Place>(m_Places[i].m_WMid, 
	 m_Places[i].m_data);
       */
      // FIXME: evaluateUnexploredPaths shouldn't be called in the message thread
      if (prev->x != oobj->x || prev->y != oobj->y) {
	// If the node has been moved, frontiers must be reevaluated
	// and, if necessary, moved.
	evaluateUnexploredPaths();
      }

      _updateNodeForPlace(correspondingPlaceID, oobj);

      log("modifiedNavNode exited");
      return;
    }
  }
  catch (DoesNotExistOnWMException e) {
    log("Couldn't find supposedly modified node!");
  }
  log("modifiedNavNode exited");
}

void 
PlaceManager::deletedNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  log("deletedNavNode called (%s)", objID.address.id.c_str());
  //TODO: This will never work!
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);

  if (oobj != 0) {

    PlaceID correspondingPlaceID = 
      _getPlaceIDForNode(oobj->getData()->nodeId);

    _deletePlace(correspondingPlaceID);
  }
  log("deletedNavNode exited");
}

void
PlaceManager::checkUnassignedEdges(int newNodeID) 
{
  for (set<NavData::AEdgePtr>::iterator it = 
      m_unprocessedEdges.begin(); it != m_unprocessedEdges.end(); it++) {
    try {
      if ((*it)->startNodeId == newNodeID ||
	  (*it)->endNodeId == newNodeID) {
	log("Processing deferred edge between nodes %i and %i", (*it)->startNodeId, (*it)->endNodeId);
	processEdge(*it); // this could invalidate 'it', but it's ok since we return
	return;
      }
    }
    catch (IceUtil::NullHandleException e) {
      log("Error! edge in list of unassigned edges was null!");
      m_unprocessedEdges.erase(it);
      it = m_unprocessedEdges.begin();
    }
  }
}

void
PlaceManager::processEdge(NavData::AEdgePtr oobj) 
{
  try {
    PlaceID startPlace = _getPlaceIDForNode(oobj->startNodeId);
    PlaceID endPlace = _getPlaceIDForNode(oobj->endNodeId);
    if (startPlace >= 0 && endPlace >= 0) {
      int newEdgeStartId = startPlace;
      int newEdgeEndId = endPlace;
      double newEdgeCost = oobj->cost;

      createConnectivityProperty(newEdgeCost, newEdgeStartId, newEdgeEndId);
      createConnectivityProperty(newEdgeCost, newEdgeEndId, newEdgeStartId);

      m_unprocessedEdges.erase(oobj);
    }
    else {
      log("Warning: New edge was detected between %i and %i, but connected Places were missing!", startPlace, endPlace);
      m_unprocessedEdges.insert(oobj);
      return;
    }
  }
  catch (IceUtil::NullHandleException e) {
    log("Error! edge objects disappeared from memory!");
  }
}

void 
PlaceManager::newEdge(const cast::cdl::WorkingMemoryChange &objID)
{
  log("newEdge called (%s)", objID.address.id.c_str());
  try {
    NavData::AEdgePtr oobj =
      getMemoryEntry<NavData::AEdge>(objID.address);

    if (oobj != 0) {
      try {
	processEdge(oobj);
      }
      catch (IceUtil::NullHandleException e) {
	log("Error! edge objects disappeared from memory!");
      }
    }
  } catch (DoesNotExistOnWMException e) {
    log("Error! edge object disappeared!");
  }
  log("newEdge exited");
}

void 
PlaceManager::modifiedEdge(const cast::cdl::WorkingMemoryChange &objID)
{
  log("modifiedEdge called (%s)", objID.address.id.c_str());
  // This will probably never be called...
  /*
     shared_ptr<CASTData<NavData::AEdge> > oobj =
     getWorkingMemoryEntry<NavData::AEdge>(objID.address);

     if (oobj != 0) {

  // Look for the place in the internal vector
   */
  log("modifiedEdge exited");
}

void 
PlaceManager::newObject(const cast::cdl::WorkingMemoryChange &objID)
{
	// Commented out as for the AVS IROS paper, the AVS is creating the objectplaceproperties!

/**  log("newObject called (%s)", objID.address.id.c_str());
  try {
    lockEntry(objID.address, cdl::LOCKEDOD);
    vector<NavData::FNodePtr> nodes;
    getMemoryEntries<NavData::FNode>(nodes, 0);

    NavData::ObjDataPtr object;
    try {
      object = getMemoryEntry<NavData::ObjData>(objID.address);
      unlockEntry(objID.address);
    }
    catch (DoesNotExistOnWMException e) {
      log("Error! New ObjData couldn't be read!");
      unlockEntry(objID.address);
      log("newObject exited");
      return;
    }
      
    string category;
    double objX;
    double objY;
    //Find the node closest to the robotPose
    objX = object->x;
    objY = object->y;
    category = object->category;
    double probability = object->probability;
    
    log("newObject is of category %s and exists with probability %f", category.c_str(), probability);


    double minDistance = FLT_MAX;
    NavData::FNodePtr closestNode = 0;

    for (vector<NavData::FNodePtr>::iterator it = nodes.begin();
	it != nodes.end(); it++) {
      try {
	double x = (*it)->x;
	double y = (*it)->y;

	double distance = (x - objX)*(x-objX) + (y-objY)*(y-objY);
	if (distance < minDistance) {
	  closestNode = *it;
	  minDistance = distance;
	}
      }
      catch (IceUtil::NullHandleException e) {
	log("Error! FNode suddenly disappeared!");
      }
    }

    if (closestNode != 0) {
      PlacePtr place = _getPlaceIDForNode(closestNode->nodeId);
      if (place != 0) {
	int placeID = place->id;
	try {
	  SpatialProperties::StringValuePtr objCat = 
	    new SpatialProperties::StringValue;
	  objCat->value = category;

	  SpatialProperties::ValueProbabilityPair pair1 =
	  { objCat, probability };

	  SpatialProperties::ValueProbabilityPairs pairs;
	  pairs.push_back(pair1);

	  SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
	    new SpatialProperties::DiscreteProbabilityDistribution;
	  discDistr->data = pairs;

	  SpatialProperties::ObjectPlacePropertyPtr objProp =
	    new SpatialProperties::ObjectPlaceProperty();
	  objProp->placeId = placeID;
	  objProp->distribution = discDistr;
	  objProp->mapValue = objCat;
	  objProp->mapValueReliable = 1;

	  string newID = newDataID();
	  addToWorkingMemory<SpatialProperties::ObjectPlaceProperty>(newID, objProp);
	}
	catch (DoesNotExistOnWMException e) {
	  log("Error! Failed to create new object property!");
	  unlockEntry(objID.address.id);
	  log("newObject exited");
	  return;
	}
      }
      else {
	log("Could not find Place for object!");
	unlockEntry(objID.address.id);
	log("newObject exited");
	return;
      }
    }
    else {
      log("Could not find Node for object!");
      unlockEntry(objID.address.id);
      log("newObject exited");
      return;
    }
    unlockEntry(objID.address.id);
  }
  catch (DoesNotExistOnWMException e) {
    log ("Object disappeared!");
  }
  log("newObject exited"); */
}

void 
PlaceManager::newDoorHypothesis(const cast::cdl::WorkingMemoryChange &objID)
{
  log("newDoorHypothesis called  (%s)", objID.address.id.c_str());
  try {
    FrontierInterface::DoorHypothesisPtr doorHyp =
      getMemoryEntry<FrontierInterface::DoorHypothesis>(objID.address);

    if (doorHyp != 0) {
      double doorX = doorHyp->x;
      double doorY = doorHyp->y;

      vector<NodeHypothesisPtr> hyps;
      getMemoryEntries<NodeHypothesis>(hyps);
      for (vector<NodeHypothesisPtr>::iterator
	  it = hyps.begin(); it != hyps.end(); it++) {
	NodeHypothesisPtr nodeHyp = *it;
	int hypID = nodeHyp->hypID;

	// Check extant placeholder, add/modify gateway placeholder prop. accordingly
	double dx = doorX - nodeHyp->x;
	double dy = doorY - nodeHyp->y;
	double distSq = dx*dx+dy*dy;
	double gatewayness = GATEWAY_FUNCTION(distSq);
	log("newDoorHypothesis calling setOrUpgradePlaceholderGatewayProperty");
	setOrUpgradePlaceholderGatewayProperty(hypID, gatewayness);
      }
    }
  }
  catch (DoesNotExistOnWMException) {
  }
}

///* Optionally only check for placholders origining from currentPlaceId. */
//bool PlaceManager::isPointCloseToExistingPlaceholder(double x, double y, int curPlaceId=-1) {
//  log("entered isPointCloseToExistingPlaceholderd");
//  double minDistanceSq = FLT_MAX;
//
//  vector<NodeHypothesisPtr> hypotheses;
//  getMemoryEntries<NodeHypothesis>(hypotheses);
//  // Compare distance to all other hypotheses created for this node
//  for (vector<NodeHypothesisPtr>::iterator extantHypIt =
//      hypotheses.begin(); extantHypIt != hypotheses.end(); extantHypIt++) {
//    NodeHypothesisPtr extantHyp = *extantHypIt;
//    try {
//        PlacePtr placeholder = _getPlaceIDForNode(extantHyp->hypID);
//        int pid = placeholder->id;
//      log("Checking against placeholder = %d", pid);
//      if (curPlaceId < 0 || extantHyp->originPlaceID == curPlaceId) {
//        double distanceSq = (extantHyp->x - x)*(extantHyp->x - x) + (extantHyp->y - y)*(extantHyp->y - y);
//        log("2distanceSq = %f x = %f y= %f x1 = %f y1 = %f place = %d", distanceSq,x,y,extantHyp->x,extantHyp->y,extantHyp->originPlaceID);
//        if (distanceSq < minDistanceSq) {
//          minDistanceSq = distanceSq;
//        }
//      } 
//    }
//    catch (IceUtil::NullHandleException e) {
//      log("Error: hypothesis suddenly disappeared!");
//    }
//  }
//
//  if (minDistanceSq < m_minNodeSeparation * m_minNodeSeparation) 
//    return true;
//  else
//    return false;
//}

//NOTE: DEPRECATED
std::map<int, std::vector<int> > PlaceManager::getAdjacencyLists() {
  std::map<int, std::vector<int> > adjacencyLists;
//
//  // Build adjecency lists (edges to placeholders not included)
//  std::vector<NavData::AEdgePtr> edges;
//  getMemoryEntries<NavData::AEdge>(edges);
//  for(std::vector<NavData::AEdgePtr>::iterator it = edges.begin();
//      it != edges.end(); ++it) {
//
//    try {
//      PlacePtr place1 = _getPlaceIDForNode((*it)->startNodeId);
//      PlacePtr place2 = _getPlaceIDForNode((*it)->endNodeId);
//
//      adjacencyLists[place1->id].push_back(place2->id);
//      adjacencyLists[place2->id].push_back(place1->id);
//    } catch(IceUtil::NullHandleException e) {
//      log("Edge disappeared. Ignoring.");
//    }
//  }
//
//  // Add the edges that are connected to placeholders
//  std::vector<PlacePtr> places;
//  getMemoryEntries<Place>(places);
//  for(std::vector<PlacePtr>::iterator placesIt = places.begin();
//      placesIt != places.end(); ++placesIt) {
//
//    try {
//      PlacePtr place = *placesIt;
//      if(place->status == PLACEHOLDER) {
//        NodeHypothesisPtr hyp = getHypFromPlaceID(place->id);
//
//        if(!hyp)
//          continue;
//
//        adjacencyLists[hyp->originPlaceID].push_back(place->id);
//      }
//    } catch(IceUtil::NullHandleException e) {
//      log("Place disappeared. Ignoring.");
//    }
//  }
//
  return adjacencyLists;
}


//bool 
//FrontierPtCompare(const FrontierInterface::FrontierPtPtr &a, const FrontierInterface::FrontierPtPtr &b)
//{
//  return a->mWidth < b->mWidth;
//}


/* Returns the a list of coordinates of positions where placeholders can
   be placed after checking them for a series of criterias.
 
   placeId: the Id of the place to use as the current place */
//std::vector<pair <double,double> >
//PlaceManager::getPlaceholderPositionsFromFrontiers(
//    FrontierInterface::FrontierPtSeq frontiers,
//    int placeId) {
//  debug("Entered getplaceholderpositionsfromfrontiers\n");
//
//  std::vector<pair <double,double> > ret;
//
//  NavData::FNodePtr curNode = getNodeFromPlaceID(placeId);
//  if(!curNode) {
//    log("Couldnt find node from the placeid specified");
//    return ret;
//  }
//  // @demmeln 22.03.2012: curNodeId seems to be unused, so comment out.
//  // int curNodeId = curNode->nodeId 
//  double nodeX = curNode->x;
//  double nodeY = curNode->y;
//
//  // Find out which points are reachable
//  log("Find out which points are reachable");
//  std::vector<std::vector<double> > coords;
//  for(FrontierInterface::FrontierPtSeq::iterator frontierIt =
//      frontiers.begin(); frontierIt != frontiers.end(); frontierIt++) {
//    std::vector<double> coord;
//    coord.push_back((*frontierIt)->x);
//    coord.push_back((*frontierIt)->y);
//    coords.push_back(coord);
//  }
//
//  // Loop over currently observed frontiers
//  log("looping over frontiers\n");
//  for (FrontierInterface::FrontierPtSeq::iterator frontierIt =
//      frontiers.begin(); frontierIt != frontiers.end(); frontierIt++) {
//    FrontierInterface::FrontierPtPtr frontierPt = *frontierIt;
//    double x = frontierPt->x;
//    double y = frontierPt->y;
//    double nodeDistanceSq = (x - nodeX)*(x - nodeX) + (y - nodeY)*(y - nodeY);
//    log("Evaluating frontier at (%f, %f) with square-distance %f and length %f", x, y, nodeDistanceSq, frontierPt->mWidth);
//
//    double newX = x;// + m_hypPathLength * (x - nodeX)/sqrt(nodeDistanceSq);
//    double newY = y;// + m_hypPathLength * (y - nodeY)/sqrt(nodeDistanceSq);
//
//    // Consider only frontiers with an open path to them
//    if ((*frontierIt)->mState != FrontierInterface::FRONTIERSTATUSOPEN)
//      continue;
//
//    bool excluded = false;
//    for (vector<ForbiddenZone>::iterator fbIt = m_forbiddenZones.begin();
//        fbIt != m_forbiddenZones.end(); fbIt++) {
//          	  log("checking forbidden zone: %.02g, %.02g, %.02g, %.02g,", fbIt->minX, fbIt->minY, fbIt->maxX, fbIt->maxY);
//          	  log("checking against: %.02g, %.02g", newX, newY);
//      if (newX <= fbIt->maxX && newX >= fbIt->minX &&
//            newY <= fbIt->maxY && newY >= fbIt->minY) {
//        log("Placeholder in forbidden zone excluded");
//        excluded = true;
//        break;
//      }
//    }
//
//    if(excluded)
//      continue;
//
//    // Consider only frontiers at a great enough distance to the current Nav node
//    if (nodeDistanceSq < m_minFrontierDist*m_minFrontierDist) {
//      continue;
//    }
//    
//    double minDistanceSq = FLT_MAX;
//
///*
//    if (m_rejectedHypotheses.find(curNodeId) != m_rejectedHypotheses.end()) {
//      for (vector<NodeHypothesisPtr>::iterator rejectedHypIt =
//          m_rejectedHypotheses[curNodeId].begin(); rejectedHypIt != m_rejectedHypotheses[curNodeId].end(); rejectedHypIt++) {
//        double distanceSq = ((*rejectedHypIt)->x - newX)*((*rejectedHypIt)->x - newX) + ((*rejectedHypIt)->y - newY)*((*rejectedHypIt)->y - newY);
//        log ("distanceSq = %f", distanceSq);
//        if (distanceSq < minDistanceSq) {
//          minDistanceSq = distanceSq;
//        }
//      }
//    }
//*/
//    if (minDistanceSq < m_minNodeSeparation * m_minNodeSeparation) {
//      continue;
//    }
//
//    // This frontier passed all tests. Add the coordinates to the return list.
//    ret.push_back(make_pair(newX,newY));
//  }
//  return ret;
//}

void PlaceManager::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again?) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);
    
    m_PeekabotClient.connect(m_PbHost, m_PbPort);

    log("Connection to Peekabot established");

    
  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}



///* Creates a nodehypothesis with a placeholder at (x,y) connected to placeId */
//bool PlaceManager::createPlaceholder(int curPlaceId, double x, double y)
//{
//  NavData::FNodePtr curNode = m_PlaceIDToNodeMap[curPlaceId];
//  if (curNode == 0) {
//    log ("Could not determine current nav node! Can not create placeholder.");
//    return false;
//  }
//
//  int curNodeId = curNode->nodeId;
//  PlacePtr curPlace = _getPlaceIDForNode(curNodeId);
//
//  if (curPlace == 0) {
//    log("Could not find current place. Can not create placeholder.");
//    return false;
//  }
//
//  int currentPlaceID = curPlace->id;
//
//  NodeHypothesisPtr newHyp = 
//    new NodeHypothesis;
//  newHyp->x = x;
//  newHyp->y = y;
//  newHyp->hypID = m_hypIDCounter;
//  newHyp->originPlaceID = currentPlaceID;
//
//  log("Adding new hypothesis at (%f, %f) with ID %i", newHyp->x,
//      newHyp->y, newHyp->hypID);
//
//  string newID = newDataID();
//  m_HypIDToWMIDMap[newHyp->hypID]=newID;
//
//  // Create the Place struct corresponding to the hypothesis
//  PlaceHolder p;
//  p.m_data = new Place;   
//  //p.m_data->id = oobj->getData()->nodeId;
//
//  int newPlaceID = m_placeIDCounter;
//  m_placeIDCounter++;
//  p.m_data->id = newPlaceID;
//  m_PlaceIDToHypMap[newPlaceID] = newHyp;
//  m_hypIDCounter++;
//
//  // Add connectivity property (one-way)
//  createConnectivityProperty(m_hypPathLength, currentPlaceID, newPlaceID);
//  m_hypotheticalConnectivities.push_back(pair<int, int>(currentPlaceID, newPlaceID));
//
//  p.m_data->status = PLACEHOLDER;
//  p.m_WMid = newDataID();
//  log("Adding placeholder %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
//  addToWorkingMemory<Place>(p.m_WMid, p.m_data);
//  addToWorkingMemory<NodeHypothesis>(newID, newHyp);
//
//  m_Places[newPlaceID]=p;
//
//  return true;
//}


/* Updates the edge of the placeholder so that it is linked to the node to
   which it has the shortest path.
   Returns the place id the placeholder is linked to or -1 on error*/
int PlaceManager::updatePlaceholderEdge(int placeholderId) {

  const NodeHypothesisPtr hyp = _getHypForPlace(placeholderId);
  if(!hyp) {
    return -1;
  }

  int originPlaceId = hyp->originPlaceID;

  log("Finding closest node to placeholder %i.", placeholderId);
  int closestNodeId = m_mapInterface->findClosestNode(hyp->x, hyp->y);
  if(closestNodeId < 0) {
    log("Error in finding closest node. Returning.");
    return -1;
  }

  PlaceID closestPlace = _getPlaceIDForHyp(closestNodeId);
//  PlacePtr closestPlace = _getPlaceIDForNode(closestNodeId);
  if(closestPlace < 0) {
    log("No place attached to node. Try again later.");
    return -1;
  }

  if(closestPlace == hyp->originPlaceID) {
    log("Closest place was the same as before. Returning.");
    return closestPlace;
  }

  log("Deleting connectivity property.");
  deleteConnectivityProperty(originPlaceId, placeholderId);

  hyp->originPlaceID = closestPlace;

  _overwriteHypForPlace(placeholderId, hyp);

  log("Found a new connection!");

  return closestPlace;
}



/* Upgrades placeholderproperties of placeholders reachable from placeID */
void PlaceManager::updateReachablePlaceholderProperties(int placeID) {

  m_PlacePropsMutex.lock();

  vector<NodeHypothesisPtr> hypotheses;
  getMemoryEntries<NodeHypothesis>(hypotheses);

  for (vector<NodeHypothesisPtr>::iterator hypIt =
      hypotheses.begin(); hypIt != hypotheses.end(); hypIt++) {
    try {
      if ((*hypIt)->originPlaceID == placeID) {
        NodeHypothesisPtr hyp = *hypIt;
        int hypID = hyp->hypID;


        PlaceID placeholderID = _getPlaceIDForHyp(hypID);

        if (placeholderID >= 0) {

          //NOTE: May block, but shouldn't since 
          //LocalMapManager::getHypothesisEvaluation only waits for the first scan
          FrontierInterface::HypothesisEvaluation eval = 
            hypothesisEvaluator->getHypothesisEvaluation(hypID);
          log("Hypothesis %i evaluates to %f %f.", hypID, eval.freeSpaceValue,
              eval.unexploredBorderValue);

          //Create/update the placeholder properties

          {
            //Free space property
            SpatialProperties::FloatValuePtr freespacevalue = 
              new SpatialProperties::FloatValue;
            freespacevalue->value = eval.freeSpaceValue;
            SpatialProperties::ValueProbabilityPair pair =
            { freespacevalue, 1.0 };
            SpatialProperties::ValueProbabilityPairs pairs;
            pairs.push_back(pair);
            SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
              new SpatialProperties::DiscreteProbabilityDistribution;
            discDistr->data = pairs;

            map<int, string>::iterator
              foundFSIt = m_freeSpaceProperties.find(placeholderID);
            if (foundFSIt != m_freeSpaceProperties.end()) {
              try {
                log("lock 6");
                lockEntry(foundFSIt->second, cdl::LOCKEDOD);
                SpatialProperties::AssociatedSpacePlaceholderPropertyPtr
                  freeProp = getMemoryEntry
                  <SpatialProperties::AssociatedSpacePlaceholderProperty>
                  (foundFSIt->second);

                if (freeProp != 0) {
                  // Property exists; change it
                  freeProp->distribution = discDistr;
                  freeProp->placeId = placeholderID;
                  freeProp->mapValue = freespacevalue;
                  freeProp->mapValueReliable = 1;

                  log("overwrite 2: %s", foundFSIt->second.c_str());
                  bool done = false;
                  while (!done) {
                    try {
                      overwriteWorkingMemory
                        <SpatialProperties::AssociatedSpacePlaceholderProperty>(foundFSIt->second,freeProp);
                      done=true;
                    }
                    catch(PermissionException e) {
                      log("Error! permissionException! Trying again...");
                    }
                  }
                }
                unlockEntry(foundFSIt->second);
                log("unlock 6");
              }
              catch(DoesNotExistOnWMException e) {
                log("Property missing!");
              }
            }
            else {
              SpatialProperties::AssociatedSpacePlaceholderPropertyPtr freeProp =
                new SpatialProperties::AssociatedSpacePlaceholderProperty;
              freeProp->distribution = discDistr;
              freeProp->placeId = placeholderID;
              freeProp->mapValue = freespacevalue;
              freeProp->mapValueReliable = 1;

              string newID = newDataID();
              addToWorkingMemory<SpatialProperties::AssociatedSpacePlaceholderProperty>
                (newID, freeProp);
              m_freeSpaceProperties[placeholderID] = newID;
            }
          }



          /* Gatewayness Property */

          double gatewayness = getGatewayness(hyp->x, hyp->y);

          {
            m_PlacePropsMutex.unlock();
            setOrUpgradePlaceholderGatewayProperty(hypID, gatewayness);
            m_PlacePropsMutex.lock();
          }

          /* Frontier length property */
          {
            SpatialProperties::FloatValuePtr bordervalue = 
              new SpatialProperties::FloatValue;
            bordervalue->value = eval.unexploredBorderValue;
            SpatialProperties::ValueProbabilityPair pair =
            { bordervalue, 1.0 };
            SpatialProperties::ValueProbabilityPairs pairs;
            pairs.push_back(pair);
            SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
              new SpatialProperties::DiscreteProbabilityDistribution;
            discDistr->data = pairs;

            map<int, string>::iterator
              foundUnexpIt = m_borderProperties.find(placeholderID);
            if (foundUnexpIt != m_borderProperties.end()) {
              try {
                log("lock 7");
                lockEntry(foundUnexpIt->second, cdl::LOCKEDOD);
                SpatialProperties::AssociatedBorderPlaceholderPropertyPtr
                  borderProp = getMemoryEntry
                  <SpatialProperties::AssociatedBorderPlaceholderProperty>
                  (foundUnexpIt->second);

                if (borderProp != 0) {
                  // Property exists; change it
                  borderProp->distribution = discDistr;
                  borderProp->placeId = placeholderID;
                  borderProp->mapValue = bordervalue;
                  borderProp->mapValueReliable = 1;
                  log("overwrite 3: %s", foundUnexpIt->second.c_str());
                  bool done = false;
                  while (!done) {
                    try {
                      overwriteWorkingMemory
                        <SpatialProperties::AssociatedBorderPlaceholderProperty>(foundUnexpIt->second,borderProp);
                      done=true;
                    }
                    catch(PermissionException e) {
                      log("Error! permissionException! Trying again...");
                    }
                  }
                }

                unlockEntry(foundUnexpIt->second);
                log("unlock 7");
              }
              catch(DoesNotExistOnWMException e) {
                log("Property missing!");
              }

            }
            else {
              SpatialProperties::AssociatedBorderPlaceholderPropertyPtr borderProp =
                new SpatialProperties::AssociatedBorderPlaceholderProperty;
              borderProp->distribution = discDistr;
              borderProp->placeId = placeholderID;
              borderProp->mapValue = bordervalue;
              borderProp->mapValueReliable = 1;

              string newID = newDataID();
              addToWorkingMemory<SpatialProperties::AssociatedBorderPlaceholderProperty>
                (newID, borderProp);
              m_borderProperties[placeholderID] = newID;
            }
          }
        }
      }
    }
    catch (IceUtil::NullHandleException e) {
      log("Error: hypothesis suddenly disappeared!");
    }
  }

  m_PlacePropsMutex.unlock();
}

///* Updates the position of a placeholder to match the position of a frontier
//   close to it. If the frontier has moved slightly, the placeholder will move
//   with it. */
//void PlaceManager::updatePlaceholderPositions(FrontierInterface::FrontierPtSeq frontiers) {
//  vector<NodeHypothesisPtr> hypotheses;
//  getMemoryEntries<NodeHypothesis>(hypotheses);
//  for (FrontierInterface::FrontierPtSeq::iterator frontierIt = frontiers.begin(); frontierIt != frontiers.end(); frontierIt++) {
//    FrontierInterface::FrontierPtPtr frontierPt = *frontierIt;
//    double frontierX = frontierPt->x;
//    double frontierY = frontierPt->y;
//    double minDistanceSq = FLT_MAX;
//    int minDistID = -1;
//
//    for (vector<NodeHypothesisPtr>::iterator extantHypIt =
//        hypotheses.begin(); extantHypIt != hypotheses.end(); extantHypIt++) {
//      NodeHypothesisPtr extantHyp = *extantHypIt;
//      try {
//        // if (extantHyp->originPlaceID == currentPlaceID) {
//        double distanceSq = (extantHyp->x - frontierX)*(extantHyp->x - frontierX) + (extantHyp->y - frontierY)*(extantHyp->y - frontierY);
//        log("2distanceSq = %f", distanceSq);
//        if (distanceSq < minDistanceSq) {
//          minDistanceSq = distanceSq;
//          minDistID = extantHyp->hypID;
//        }
//        //}
//      }
//      catch (IceUtil::NullHandleException e) {
//        log("Error: hypothesis suddenly disappeared!");
//      }
//    }
//
//    if (minDistanceSq < m_minNodeSeparation * m_minNodeSeparation && minDistID != -1 && minDistanceSq > 0) {
//      // Modify the extant hypothesis that best matched the
//      // new position indicated by the frontier
//      try {
//        lockEntry(m_HypIDToWMIDMap[minDistID], cdl::LOCKEDOD);
//        NodeHypothesisPtr updatedHyp = 
//          getMemoryEntry<NodeHypothesis>(m_HypIDToWMIDMap[minDistID]);
//
//        // Remove the hypothesis from our local list so we don't move
//        // it back again in the next iteration...
//        for (vector<NodeHypothesisPtr>::iterator iter =
//            hypotheses.begin(); iter != hypotheses.end(); iter++) {
//          if ((*iter)->hypID == minDistID) {
//            hypotheses.erase(iter);
//            break;
//          }
//        }
//        log("Updating hypothesis at (%f, %f) with ID %i to (%f, %f)", updatedHyp->x, updatedHyp->y, updatedHyp->hypID, frontierX, frontierY);
//
//        updatedHyp->x = frontierX;
//        updatedHyp->y = frontierY;
//
//        overwriteWorkingMemory<NodeHypothesis>(m_HypIDToWMIDMap[minDistID], updatedHyp);
//        unlockEntry(m_HypIDToWMIDMap[minDistID]);
//      }
//      catch (const std::exception& e) {
//        log("Could not update hypothesis! Caught exception at %s. Message: %s", __HERE__, e.what());
//      }
//    }
//  }
//}

void PlaceManager::evaluateUnexploredPaths()
{
  log("Entering evaluateUnexplorePaths");
  IceUtil::Mutex::Lock locks(m_PlaceholderMutex);
  NavData::FNodePtr curNode = getCurrentNavNode();

  if(!m_bNoPlaceholders) {
    PlaceID curPlace = _getPlaceIDForNode(curNode->nodeId);
    
    if(curPlace < 0) {
      error("Could not find current place on %i.", __LINE__);
      return;
    }

    NodeHypothesisSeq hypotheses = m_mapInterface->refreshNodeHypothesis();
//PROCESS NEW NODE HYP, ADD NEW, DELETE, OVERWRITE     
//1. LOOP NODE HYPOTHESIS
    vector<PlacePtr> places;
    getMemoryEntries<Place>(places);

    // Loop over existing placeholders. For each, check if the corresponding
    // hypothesis still exists in the fresh set.
    // If it does, see that it hasn't moved into a forbidden zone,
    // and if it is still valid, check if it now changed its closest place
    // and if so, update connectivity to reflect this
    for (vector<PlacePtr>::iterator it = places.begin(); it != places.end(); ++it) {
      if ((*it)->status == PLACEHOLDER) {
        try {
          PlacePtr place = *it;
          const NodeHypothesisPtr nodeHyp = _getHypForPlace(place->id);
	  
	  if (nodeHyp == 0) {
	    getLogger()->warn("Couldn't find NodeHypothesis!");
	    continue;
	  }

          bool exists=false;
          
          for (vector<NodeHypothesisPtr>::iterator extantHypIt =
              hypotheses.begin(); extantHypIt != hypotheses.end(); extantHypIt++) {
            NodeHypothesisPtr extantHyp = *extantHypIt;
            if (extantHyp->hypID==nodeHyp->hypID){
              exists=true;
//              if ((*nodeHyp).x!=extantHyp->x || (*nodeHyp).y!=extantHyp->y){
                for (vector<ForbiddenZone>::iterator fbIt = m_forbiddenZones.begin();
		    fbIt != m_forbiddenZones.end(); fbIt++) {
                      	  log("checking forbidden zone: %.02g, %.02g, %.02g, %.02g,", fbIt->minX, fbIt->minY, fbIt->maxX, fbIt->maxY);
                      	  log("checking against: %.02g, %.02g", extantHyp->x, extantHyp->y);
                  if ( extantHyp->x <= fbIt->maxX &&  extantHyp->x >= fbIt->minX &&
                        extantHyp->y <= fbIt->maxY && extantHyp->y >= fbIt->minY) {
                    log("Placeholder in forbidden zone excluded");
                    exists = false;
                    break;
                  }
                }
                if (exists){
                  log("move overlapped placeholder");
                  nodeHyp->x=extantHyp->x;
                  nodeHyp->y=extantHyp->y;
                  if (extantHyp->originPlaceID == -1) extantHyp->originPlaceID = _getPlaceIDForNode(extantHyp->originNodeID);
                  if (nodeHyp->originPlaceID != extantHyp->originPlaceID){
                    log("Found closest node. Connecting with the current");

		    log("Deleting connectivity between %i and %i", nodeHyp->originPlaceID, place->id);
                    deleteConnectivityProperty(nodeHyp->originPlaceID, place->id);
		    for (list<std::pair<int, int> >::iterator
			      hit = m_hypotheticalConnectivities.begin();
			      hit != m_hypotheticalConnectivities.end(); hit++) {
		      if (hit->first == nodeHyp->originPlaceID &&
			        hit->second == place->id) {
			      m_hypotheticalConnectivities.erase(hit);
			      break;
		      }
		    }

                    nodeHyp->originPlaceID = extantHyp->originPlaceID;

		    log("Adding connectivity between %i and %i", extantHyp->originPlaceID, place->id);
                    createConnectivityProperty(m_hypPathLength, 
			extantHyp->originPlaceID, place->id);
                    m_hypotheticalConnectivities.push_back(pair<int, int>(place->id, extantHyp->originPlaceID));

                  }
  //TODO CHANGE origplace and connectivity
		  _overwriteHypForPlace(place->id, nodeHyp);
                }
              //}
              break;
            }
          }
//2. DELETE NON EXISTING PLACEHOLDERS
          if (!exists){
            log("delete overlapped placeholder");
            deletePlaceholder(place->id);
          }
        }
        catch (IceUtil::NullHandleException e) {
          log("Place suddenly disappeared!\n");
        }
      }
    }
//3. CREATE NEW PLACES FROM NEW NODEHYPOTHESES
    for (vector<NodeHypothesisPtr>::iterator extantHypIt =
        hypotheses.begin(); extantHypIt != hypotheses.end(); extantHypIt++) {
      NodeHypothesisPtr newHyp = *extantHypIt;
      if (newHyp->hypID==-1){
        log("create new placeholder");
//        newHyp->hypID = m_hypIDCounter;
        if (newHyp->originPlaceID == -1) newHyp->originPlaceID = _getPlaceIDForNode(newHyp->originNodeID);
//          log("Couldn't find closest node. Connecting with the current");
 
//       newHyp->originPlaceID = _getPlace(curPlace)->id;

        bool excluded = false;
        for (vector<ForbiddenZone>::iterator fbIt = m_forbiddenZones.begin();
            fbIt != m_forbiddenZones.end(); fbIt++) {
              	  log("checking forbidden zone: %.02g, %.02g, %.02g, %.02g,", fbIt->minX, fbIt->minY, fbIt->maxX, fbIt->maxY);
              	  log("checking against: %.02g, %.02g", newHyp->x, newHyp->y);
          if ( newHyp->x <= fbIt->maxX &&  newHyp->x >= fbIt->minX &&
                newHyp->y <= fbIt->maxY && newHyp->y >= fbIt->minY) {
            log("Placeholder in forbidden zone excluded");
            excluded = true;
            break;
          }
        }
        if (!excluded){
          // Create the Place struct corresponding to the hypothesis
          PlacePtr p;
          p = new Place;   
          p->status = PLACEHOLDER;

      	  PlaceID newPlaceID = _addPlaceWithHyp(p, newHyp);
          log("Added new hypothesis at (%f, %f) with ID %i", newHyp->x,
              newHyp->y, newHyp->hypID);

          // Add connectivity property (one-way)
          createConnectivityProperty(m_hypPathLength, newHyp->originPlaceID, newPlaceID);
          m_hypotheticalConnectivities.push_back(pair<int, int>(newHyp->originPlaceID, newPlaceID));
        }
      }
    }

    // Update properties for the placeholders reachable from the current place
    log("Updating reachable placeholder properties.");
    updateReachablePlaceholderProperties(curPlace);
  }
  int curPlaceID = _getPlaceIDForNode(curNode->nodeId);
  if (curPlaceID>=0){
    set<int> &curPlaceConnectivities = m_connectivities[curPlaceID];
    vector<PlaceMapEntry> gr;
    for (int k = 0; k < entries.size(); k++) {
      if (entries[k].node != 0)
        gr.push_back(entries[k]);
    }

    bool swapped = true;
    int j = 0;
    PlaceMapEntry tmp;
    while (swapped) {
      swapped = false;
      j++;
      for (int k = 0; k < gr.size() - j; k++) {
        double dist1 = (gr[k].node->x - curNode->x)*(gr[k].node->x - curNode->x) + (gr[k].node->y - curNode->y) * (gr[k].node->y - curNode->y);
        double dist2 = (gr[k + 1].node->x - curNode->x)*(gr[k + 1].node->x - curNode->x) + (gr[k + 1].node->y - curNode->y) * (gr[k + 1].node->y - curNode->y);
        if (dist2 < dist1) {
              tmp = gr[k];
              gr[k] = gr[k + 1];
              gr[k + 1] = tmp;
              swapped = true;
        }
      }
    }

    vector<FrontierInterface::DoorHypothesisPtr> doorHyps;
    getMemoryEntries<FrontierInterface::DoorHypothesis>(doorHyps);

    for(vector<PlaceMapEntry>::iterator it = gr.begin(); it != gr.end(); it++) {
      if ((it->node != 0) && (it->node->nodeId != curNode->nodeId) && (curPlaceConnectivities.find(it->place->id) == curPlaceConnectivities.end())){ 
        // Check if was checked already            

        // Check if are connected via node
        double max_dist = 2.1;    
        bool link_gateway = false;
        for(set<int>::iterator it1 = curPlaceConnectivities.begin(); it1 != curPlaceConnectivities.end(); it1++) {
          set<int> placeConnectivities = m_connectivities[(*it1)];
          set<int>::iterator it2 = placeConnectivities.find(it->place->id);
          if (it2 != placeConnectivities.end()){
            max_dist = 1.5;
            NavData::FNodePtr link = _getNodeForPlace(*it1);
            log("alex link %d %d - %d", curPlaceID, it->place->id, (*it1));
                        
            if (link->gateway == 1){
              link_gateway = true;
              break;
            }              
          } 
        }
        // Connected via doorway - skip
        if (link_gateway) continue;
        double dist2 = (it->node->x - curNode->x)*(it->node->x - curNode->x) + (it->node->y - curNode->y) * (it->node->y - curNode->y);
                    
        if (dist2 < max_dist * max_dist){
          // Check path
          double path_dist = m_mapInterface->getPathLength(it->node->x,it->node->y,curNode->x,curNode->y);
          log("alex path_dist %d %d - %f %f %f %f", curPlaceID, it->place->id, max_dist,sqrt(dist2), path_dist, path_dist/sqrt(dist2));
          if (path_dist/sqrt(dist2) > 0 && path_dist/sqrt(dist2) < 22){
            // Check doorHyps
            bool intersects_door = false;
            if ((it->node->gateway == 0) && (curNode->gateway == 0)){
              for (vector<FrontierInterface::DoorHypothesisPtr>::iterator itDoor = 
                  doorHyps.begin(); itDoor != doorHyps.end(); itDoor++) {
                double x1 = it->node->x;
                double y1 = it->node->y;
                double x2 = curNode->x;
                double y2 = curNode->y;

                double dx = (*itDoor)->x;
                double dy = (*itDoor)->y;
                double theta = (*itDoor)->theta;
                double width = (*itDoor)->width;

                double l = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));         
                double nx = (x2-x1)/l;                
                double ny = (y2-y1)/l;                
                double n2x = cos(theta);
                double n2y = sin(theta);
                if (((n2x*ny-n2y*nx)!=0) && ((n2y*nx-n2x*ny)!=0)){
                  double s = (ny*x1-nx*y1 - dx*ny + dy*nx)/(n2x*ny-n2y*nx);
                  double t = (dx*n2y-dy*n2x - x1*n2y + y1*n2x)/(n2y*nx-n2x*ny);
                  
                  if (t > 0 && t < l && (s > -width/2) && (s < width/2)){
                    NodeHypothesisPtr newHyp = new SpatialData::NodeHypothesis();
                    if (t>l/2) t=l/2;
                    newHyp->x=x1 + nx*t;
                    newHyp->y=y1 + ny*t;
                    newHyp->hypID=-1;
                    newHyp->originPlaceID=curPlaceID;
                    newHyp->originNodeID=curNode->nodeId;
                    newHyp->doorway=true;
                    PlacePtr p;
                    p = new Place;   
                    p->status = PLACEHOLDER;

                	  PlaceID newPlaceID = _addPlaceWithHyp(p, newHyp);
                    log("Added new hypothesis at (%f, %f) with ID %i", newHyp->x,
                        newHyp->y, newHyp->hypID);

                    // Add connectivity property (one-way)
                    createConnectivityProperty(m_hypPathLength, curPlaceID, newPlaceID);
                    m_hypotheticalConnectivities.push_back(pair<int, int>(newHyp->originPlaceID, newPlaceID));

                    intersects_door = true;
                    break;
                  }  
                }
              }
            }
            if (!intersects_door){
              createConnectivityProperty(sqrt(dist2), curPlaceID, it->place->id);
              createConnectivityProperty(sqrt(dist2), it->place->id, curPlaceID);
            }
          }
        }
      }
    }
  }
  log("Exiting evaluateUnexplorePaths");
}

void
PlaceManager::setOrUpgradePlaceholderGatewayProperty(int hypothesisID, double value)
{
  log("setOrUpgradePlaceholderGatewayProperty(%i,%f) called", hypothesisID, value);
  m_PlacePropsMutex.lock();

  SpatialProperties::BinaryValuePtr gatewaynessValue =
    new SpatialProperties::BinaryValue;
  gatewaynessValue->value = true;
  SpatialProperties::ValueProbabilityPair pair =
  { gatewaynessValue, value};
  SpatialProperties::ValueProbabilityPairs pairs;
  pairs.push_back(pair);

  SpatialProperties::BinaryValuePtr noGatewaynessValue =
    new SpatialProperties::BinaryValue;
  noGatewaynessValue->value = false;
  SpatialProperties::ValueProbabilityPair pair2 =
  { noGatewaynessValue, (1 - value)};
  pairs.push_back(pair2);


  SpatialProperties::BinaryValuePtr gatewaynessMapValue =
    new SpatialProperties::BinaryValue;

  gatewaynessMapValue->value = (value > 0.5);

  SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
    new SpatialProperties::DiscreteProbabilityDistribution;
  discDistr->data = pairs;

  PlaceID placeholderID = _getPlaceIDForHyp(hypothesisID);
  if (placeholderID < 0) {
    error("Error! Hypothesis missing on %i", __LINE__);
    return;
  }

  map<int, string>::iterator
    foundFSIt = m_placeholderGatewayProperties.find(placeholderID);
  if (foundFSIt != m_placeholderGatewayProperties.end()) {
    try {
      lockEntry(foundFSIt->second, cdl::LOCKEDOD);
      SpatialProperties::GatewayPlaceholderPropertyPtr
	gwProp = getMemoryEntry
	<SpatialProperties::GatewayPlaceholderProperty>
	(foundFSIt->second);

      if (gwProp != 0) {
	// Property exists; change it if value is higher
	SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
	  SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(gwProp->distribution);
	double currentProb = discDistr->data[0].probability;
	log("Comparing with existing probability: %f", currentProb);
	if (currentProb < value) {
	  gwProp->distribution = discDistr;
	  gwProp->placeId = placeholderID;
	  gwProp->mapValue = gatewaynessMapValue;
	  gwProp->mapValueReliable = 1;

	  log("overwrite: %s", foundFSIt->second.c_str());
	  bool done = false;
	  while (!done) {
	    try {
	      overwriteWorkingMemory
		<SpatialProperties::GatewayPlaceholderProperty>(foundFSIt->second,gwProp);
	      done=true;
	    }
	    catch(PermissionException e) {
	      log("Error! permissionException! Trying again...");
	    }
	  }
	}
      }
      unlockEntry(foundFSIt->second);
    }
    catch(DoesNotExistOnWMException e) {
      log("Property missing!");
    }
  }
  else {
    SpatialProperties::GatewayPlaceholderPropertyPtr gwProp =
      new SpatialProperties::GatewayPlaceholderProperty;
    gwProp->distribution = discDistr;
    gwProp->placeId = placeholderID;
    gwProp->mapValue = gatewaynessMapValue;
    gwProp->mapValueReliable = 1;

    string newID = newDataID();
    log("Adding new GWPlaceholderProperty");
    addToWorkingMemory<SpatialProperties::GatewayPlaceholderProperty>
      (newID, gwProp);
    m_placeholderGatewayProperties[placeholderID] = newID;
  }

  m_PlacePropsMutex.unlock();
}

double
PlaceManager::getGatewayness(double x, double y) 
{
  vector<FrontierInterface::DoorHypothesisPtr> doorHyps;
  getMemoryEntries<FrontierInterface::DoorHypothesis>(doorHyps);

  double minDistSq = DBL_MAX;
  for (vector<FrontierInterface::DoorHypothesisPtr>::iterator it = 
      doorHyps.begin(); it != doorHyps.end(); it++) {
    double dx = x - (*it)->x;
    double dy = y - (*it)->y;
    double distSq = dx*dx+dy*dy;
    if (distSq < minDistSq) {
      minDistSq = distSq;
    }
  }
  return GATEWAY_FUNCTION(minDistSq);
}

PlacePtr PlaceManager::getCurrentPlace() {
  NavData::FNodePtr curNode = getCurrentNavNode();
  if(curNode != 0){
    PlacePtr curPlace = _getPlace(_getPlaceIDForNode(curNode->nodeId));
    return curPlace;
  } else {
    return 0;
  }
}

NavData::FNodePtr
PlaceManager::getCurrentNavNode()
{
  // log("getCurrentNavNode called");
  vector<NavData::FNodePtr> nodes;
  getMemoryEntries<NavData::FNode>(nodes, 0);

  vector<NavData::RobotPose2dPtr> robotPoses;
  getMemoryEntries<NavData::RobotPose2d>(robotPoses, 0);

  if (robotPoses.size() == 0) {
    log("getCurrentNavNode exited - Could not find RobotPose!");
    return 0;
  }

  //Find the node closest to the robotPose
  double robotX = robotPoses[0]->x;
  double robotY = robotPoses[0]->y;
  double minDistance = FLT_MAX;
  NavData::FNodePtr ret = 0;

  for (vector<NavData::FNodePtr>::iterator it = nodes.begin();
      it != nodes.end(); it++) {
    try {
      double x = (*it)->x;
      double y = (*it)->y;

      double distance = (x - robotX)*(x-robotX) + (y-robotY)*(y-robotY);
      if (distance < minDistance) {
	ret = *it;
	minDistance = distance;
      }
    }
    catch (IceUtil::NullHandleException e) {
      println("Error! FNode suddenly disappeared!");
    }
  }
 // log("getCurrentNavNode exited");
  if (ret == 0) log("Warning: Found no current Nav Node");
  return ret;
}

FrontierInterface::PlaceMembership
PlaceManager::getPlaceMembership(double inX, double inY)
{
  log("getPlaceMembership called (%i, %i)", inX, inY);
  vector<NavData::FNodePtr> nodes;
  getMemoryEntries<NavData::FNode>(nodes, 0);

  double minDistance = FLT_MAX;
  int closestNodeID = -1;

  for (vector<NavData::FNodePtr>::iterator it = nodes.begin();
      it != nodes.end(); it++) {
    try {
      double x = (*it)->x;
      double y = (*it)->y;

      double distance = (x - inX)*(x-inX) + (y-inY)*(y-inY);
      if (distance < minDistance) {
	closestNodeID = (*it)->nodeId;
	minDistance = distance;
      }
    }
    catch (IceUtil::NullHandleException e) {
      log("Error! FNode suddenly disappeared!");
    }
  }

  PlaceID placeID; 

  if (closestNodeID == -1) {
    error("Error! Found no place membership on %i, assigning to place 0!", __LINE__);
    placeID = 0;
  }
  else {
    placeID = _getPlaceIDForNode(closestNodeID);
  }

  FrontierInterface::PlaceMembership ret;
  ret.placeID = placeID;
  ret.confidence = 1.0;

  log("getPlaceMembership exited");
  return ret;
}

//NodeHypothesisPtr
//PlaceManager::getHypFromPlaceID(int placeID)
//{
//  log("getHypFromPlaceID called");
//  map<int, NodeHypothesisPtr>::iterator it =
//    m_PlaceIDToHypMap.find(placeID);
//  if (it == m_PlaceIDToHypMap.end()) {
//    log("getHypFromPlaceID exited");
//    return 0;
//  }
//  else {
//    log("getHypFromPlaceID exited");
//    return it->second;
//  }
//}

//NavData::FNodePtr
//PlaceManager::getNodeFromPlaceID(int placeID)
//{
//  debug("getNodeFromPlaceID called");
//  map<int, NavData::FNodePtr>::iterator it =
//    m_PlaceIDToNodeMap.find(placeID);
//  if (it == m_PlaceIDToNodeMap.end()) {
//    debug("getNodeFromPlaceID exited");
//    return 0;
//  }
//  else {
//    debug("getNodeFromPlaceID exited");
//    return it->second;
//  }
//}

//PlacePtr 
//PlaceManager::getPlaceFromNodeID(int nodeID)
//{
//  log("getPlaceFromNodeID called");
//  for(map<int, NavData::FNodePtr>::iterator it =
//      m_PlaceIDToNodeMap.begin();
//      it != m_PlaceIDToNodeMap.end(); it++) {
//    if (it->second->nodeId == nodeID) {
//      map<int, PlaceHolder>::iterator it2 = m_Places.find(it->first);
//      if (it2 != m_Places.end()) {
//	PlacePtr ret = getMemoryEntry<Place>(it2->second.m_WMid);
//	log("getPlaceFromNodeID exited");
//	return ret;
//      }
//    }
//  }
//  log("getPlaceFromNodeID exited");
//  return 0;
//}

//PlacePtr 
//PlaceManager::getPlaceFromHypID(int hypID)
//{
//  log("getPlaceFromHypID called");
//  for(map<int, NodeHypothesisPtr>::iterator it =
//      m_PlaceIDToHypMap.begin();
//      it != m_PlaceIDToHypMap.end(); it++) {
//    if (it->second->hypID == hypID) { //Found a Place ID for the hypothesis
//      map<int, PlaceHolder>::iterator it2 = m_Places.find(it->first);
//      if (it2 != m_Places.end()) {
//	PlacePtr ret = getMemoryEntry<Place>(it2->second.m_WMid);
//	log("getPlaceFromHypID exited");
//	return ret;
//      }
//    }
//  }
//  log("getPlaceFromHypID exited");
//  return 0;
//}

void 
PlaceManager::beginPlaceTransition(int goalPlaceID)
{
  log("beginPlaceTransition called; goal place ID=%i", goalPlaceID);
  // Check whether the transition is an explored or unexplored edge
  // Store where it was we came from
  
  const NodeHypothesisPtr goalHypothesis = _getHypForPlace(goalPlaceID);

  if (goalHypothesis == 0) {
    // Goal is not unexplored
    NavData::FNodePtr goalNode = _getNodeForPlace(goalPlaceID);

    if (goalNode == 0) {
      // Goal is unknown
      log("Could not find supposed goal Place!");
      log("beginPlaceTransition exited");
      return;
    }
  }

  NavData::FNodePtr curNode = getCurrentNavNode();
  if (curNode != 0) {
    m_startNodeForCurrentPath = curNode->nodeId;
    m_currentNodeOnPath = curNode->nodeId;
  }
  else {
    log("Error! Could not find current Nav node!");
  }

  m_goalPlaceForCurrentPath = goalPlaceID;
  m_isPathFollowing = true;
  log("beginPlaceTransition exited");
}

void 
PlaceManager::processPlaceArrival(bool failed) 
{
  try {
    log("processPlaceArrival called (failed=%i)", failed);
    log("m_goalPlaceForCurrentPath was %i", m_goalPlaceForCurrentPath);

    int wasHeadingForPlace = m_goalPlaceForCurrentPath;
    int wasComingFromNode = m_startNodeForCurrentPath;

    const NodeHypothesisPtr goalHyp =
      _getHypForPlace(wasHeadingForPlace);

    bool wasExploring = (goalHyp != 0);

    NavData::FNodePtr curNode = getCurrentNavNode();
    if (curNode != 0) {
      int curNodeId = curNode->nodeId;
      log("current node id: %i", curNodeId);

      PlaceID curPlaceID = _getPlaceIDForNode(curNodeId);
      bool placeExisted = (curPlaceID >= 0);

      double curNodeX = curNode->x;
      double curNodeY = curNode->y;
      int curNodeGateway = curNode->gateway;

      int arrivalCase = -1;

      if (wasExploring) {
        bool closeToGoal = false;
        vector<NavData::RobotPose2dPtr> robotPoses;
        getMemoryEntries<NavData::RobotPose2d>(robotPoses, 0);
        if (robotPoses.size() != 0) {
          double distSq = (robotPoses[0]->x-goalHyp->x)*(robotPoses[0]->x-goalHyp->x) + (robotPoses[0]->y-goalHyp->y)*(robotPoses[0]->y-goalHyp->y);
          closeToGoal = distSq < 0.25*m_minNodeSeparation*m_minNodeSeparation;
        }
        //The transition was an exploration action
        if (!placeExisted && closeToGoal && (curNodeGateway == 0)) { //No Place exists for current node -> it must be new.
          arrivalCase = 1;
          //CASE 1: We were exploring a path, and a new node was discovered.
          //Stop moving, upgrade the placeholder we were heading for and connect it
          //to this new node, and delete the NodeHypotheses
          log("  CASE 1: New node close to goal discovered while exploring");

	  PlacePtr goalPlace = _getPlace(wasHeadingForPlace);

          if (goalPlace != 0) {
            //Upgrade Place "wasHeadingForPlace"; delete hypothesis goalHyp; 
            //make the Place refer to node curNode
            upgradePlaceholder(wasHeadingForPlace, curNode);

            if (curNodeGateway == 1) {
              addNewGatewayProperty(wasHeadingForPlace);
            }
          }
          else {
            log("Missing place %i on %i! Cancelling movement!", wasHeadingForPlace, __LINE__);
            cancelMovement();
            log("processPlaceArrival exited");
            return;
          }
        }
        else if (!placeExisted && closeToGoal && (curNodeGateway == 1)) {
          arrivalCase = 7;
          //CASE 7: 
          log("  CASE 7: Door");
          
          /* If we are not close enough we came to a new node with no place, so we add one and DON'T cancel the movement */
          addPlaceForNode(curNode);
          
        }

        else if (!placeExisted && !closeToGoal) {
          //No Place exists for current node -> it must be new BUT too far away from our goal to be upgraded in CASE 1.
          arrivalCase = 2;
          //CASE 2: We were exploring a path, and a new far from the goal node was discovered.
          //Don't stop our movement, just add a new node and continue our path.
          log("  CASE 2: New node far from goal discovered while exploring");
          
          /* If we are not close enough we came to a new node with no place, so we add one and DON'T cancel the movement */
          addPlaceForNode(curNode);
        }

        else if (!failed && curNodeId != wasComingFromNode && closeToGoal) {
          arrivalCase = 3;
//          int currentPlaceID = curPlaceID;
          //CASE 3: We were exploring, but ended up in a known Place which was not
          //the one we started from.
          //Remove the NodeHypothesis and its Placeholder, and
          //send the Place merge message
          log("  CASE 3: Exploration action failed - place already known. Deleting Place %i", wasHeadingForPlace);

          m_isPathFollowing = false; 
/*
          deletePlaceProperties(wasHeadingForPlace);

          m_rejectedHypotheses[wasComingFromNode].push_back(goalHyp);
      	  deletePlaceholder(wasHeadingForPlace);

          //Prepare and send merge notification
          PlaceMergeNotificationPtr newNotify = new
            PlaceMergeNotification;
          newNotify->mergedPlaces.push_back(wasHeadingForPlace);
          newNotify->mergedPlaces.push_back(currentPlaceID);
          newNotify->resultingPlace = currentPlaceID;
          log("Sending merge notification between places %i and %i", 
              currentPlaceID, wasHeadingForPlace);

          addToWorkingMemory<PlaceMergeNotification>(newDataID(), newNotify);
*/          //TODO:delete notifications sometime
        }
        
        else if (!failed && curNodeId != wasComingFromNode && !closeToGoal) {
          arrivalCase = 4;
          //CASE 4: We were exploring, but ended up in a known Place which was
          //not the one we started from.
          //Since we are not close to the goal we do NOT merge the goal and
          //the known place (ie. allow travelling over already visited
          //places).
          log("   CASE 4: travelling over known place %d distant from goal %d, ignoring.", curPlaceID, m_goalPlaceForCurrentPath);
        }

        else if (failed) {//curPlace != 0 && (failed || curNodeId == wasComingFromNode))
          arrivalCase = 5;
          //CASE 5: We were exploring but one way or another, we failed or 
          //ended up were we'd started.
          //Just delete the NodeHypothesis and its Placeholder.
          log("  CASE 5: Exploration action failed; couldn't reach goal. Deleting place %i",
              wasHeadingForPlace);

          m_isPathFollowing = false; 

          //int currentPlaceID = curPlace->id;

          deletePlaceProperties(wasHeadingForPlace);
          m_rejectedHypotheses[wasComingFromNode].push_back(goalHyp);
	  deletePlaceholder(wasHeadingForPlace);
        }

      }
      else { // (wasExploring); i.e. the goal place was not hypothetical
        PlacePtr curPlace = _getPlace(curPlaceID);
        bool placeExisted = (curPlace != 0);

        if (!placeExisted) { 
          arrivalCase = 6;
          //CASE 6: We were *not* exploring, but a new node was discovered.
          //We may have been going between known Places, or following a person
          //or pushed around in Stage.
          //Create a new Place for this node. If the node matches a hypothesis
          //belonging to the Place we just came from, upgrade that node as in
          //Case 1, above.
          log("  CASE 6: Node (%d) found while not exploring", curNodeId);

          //Check the previous Place for NodeHypotheses matching this one
          bool foundHypothesis = 0;

          log("processPlaceArrival:1");
          vector<NodeHypothesisPtr> hyps;
          getMemoryEntries<NodeHypothesis>(hyps);
          log("processPlaceArrival:2");

          if (wasComingFromNode >= 0) {
	    PlaceID prevPlaceID = _getPlaceIDForNode(wasComingFromNode);
            if (prevPlaceID >= 0) {
	      vector<PlaceID> placeholders;
	      _getPlaceholders(placeholders);
	      NavData::FNodePtr prevNode = _getNodeForPlace(prevPlaceID);

	      // Find Placeholders that have the previous node as their origin
	      // and close enough to the new one to be upgraded
	      for(vector<PlaceID>::iterator
		  phIt = placeholders.begin(); phIt != placeholders.end(); phIt++) {
		PlaceID placeholderID = *phIt;
		const NodeHypothesisPtr hyp = _getHypForPlace(placeholderID);
		if (hyp != 0) {
		  if (prevPlaceID == hyp->originPlaceID) {
		    double distSq = (curNodeX-hyp->x)*(curNodeX-hyp->x) + 
		      (curNodeY-hyp->y)*(curNodeY-hyp->y);
		    log("  checking hypothesis %i with distance %f (%f,%f)-(%f,%f)",
			hyp->hypID, distSq, curNodeX, curNodeY, hyp->x, hyp->y);
		    if (distSq < 0.25*m_minNodeSeparation*m_minNodeSeparation) {
		      //Close enough 
		      //FIXME: should really check just bearing, not distance (because
		      //of m_hypPathLength
		      PlacePtr placeholder = _getPlace(placeholderID);
		      if (placeholder == 0) {
			log("Could not find placeholder to upgrade");
			m_goalPlaceForCurrentPath = -1;
			m_isPathFollowing = false;
			log("processPlaceArrival exited");
			return;
		      }

		      foundHypothesis = true;
		      upgradePlaceholder(placeholderID, curNode);

		      //Write the Gateway property if present
		      if (curNodeGateway == 1) {
			addNewGatewayProperty(placeholderID);
		      }
		    }
		  }
		}
	      }
	    }
	  }

          if (!foundHypothesis) {
            //Found no prior hypothesis to upgrade. Create new Place instead.

	    PlacePtr p = new Place;   
            p->status = TRUEPLACE;
	    PlaceID newPlaceID = _addPlaceWithNode(p, curNode);

            checkUnassignedEdges(curNode->nodeId);

            //Write the Gateway property if present
            if (curNodeGateway == 1) {
              addNewGatewayProperty(newPlaceID);
            }
            
          }
        }
        else {
          arrivalCase = 8;
          // We weren't exploring, and the place was known before - don't
          // do anything.
          // (Could check whether we ended up in the expected Place, but
          // that's for the future)
        }

        m_isPathFollowing = false; 
      }
    }
  }
  catch(CASTException &e) {
    cout<<e.what()<<endl;
    cout<<e.message<<endl;
    abort();
  }
  log("processPlaceArrival exited");
}


NodeHypothesisPtr 
PlaceManager::PlaceServer::getHypFromPlaceID(int placeID,
    const Ice::Current &_context) {
//  m_pOwner->lockComponent();
  NodeHypothesisPtr ptr = m_pOwner->_getHypForPlace(placeID);

  //Copy for thread safety
  if (ptr != 0)
    ptr = new NodeHypothesis(*ptr);

//  m_pOwner->unlockComponent();
  return ptr;
}

NavData::FNodePtr
PlaceManager::PlaceServer::getNodeFromPlaceID(int placeID,
    const Ice::Current &_context) {
//  m_pOwner->lockComponent();
  NavData::FNodePtr ptr = m_pOwner->_getNodeForPlace(placeID);

  //Copy for thread safety
  if (ptr != 0)
    ptr = new NavData::FNode(*ptr);

//  m_pOwner->unlockComponent();
  return ptr;
}

PlacePtr 
PlaceManager::PlaceServer::getPlaceFromNodeID(int nodeID,
    const Ice::Current &_context)
{
//  m_pOwner->lockComponent();
  PlacePtr ptr = m_pOwner->_getPlace(m_pOwner->_getPlaceIDForNode(nodeID));

  //Copy for thread safety
  if (ptr != 0)
    ptr = new Place(*ptr);

//  m_pOwner->unlockComponent();
  return ptr;
}

PlacePtr PlaceManager::PlaceServer::getPlaceFromHypID(int hypID,
    const Ice::Current &_context)
{
//  m_pOwner->lockComponent();
  PlacePtr ptr = m_pOwner->_getPlace(m_pOwner->_getPlaceIDForHyp(hypID));

  //Copy for thread safety
  if (ptr != 0)
    ptr = new Place(*ptr);

//  m_pOwner->unlockComponent();
  return ptr;
}

void 
PlaceManager::PlaceServer::beginPlaceTransition(int goalPlaceID, const Ice::Current &_context)
{
//  m_pOwner->lockComponent();
  m_pOwner->beginPlaceTransition(goalPlaceID);
//  m_pOwner->unlockComponent();
}


///* Remove placeholders that are not close to any coordinate in coords, but only
//   if it's close to the robot. Ignore placeholders that are far away.*/
//void
//PlaceManager::refreshPlaceholders(std::vector<std::pair<double,double> > coords) {
//  log("refreshPlaceholders() called");
//
//  vector<PlacePtr> places;
//  getMemoryEntries<Place>(places);
//
//  NavData::FNodePtr curNode = getCurrentNavNode();
//  if(curNode == 0)
//    return;
//  
//  for (vector<PlacePtr>::iterator it = places.begin();
//      it != places.end(); ++it) {
//    if ((*it)->status == PLACEHOLDER) {
//      try {
//        PlacePtr place = *it;
//        NodeHypothesisPtr nodeHyp = getHypFromPlaceID(place->id);
//
//        bool placeholderStillValid = false;
//        for(vector<pair <double, double> >::iterator coordIt = coords.begin();
//            coordIt != coords.end(); ++coordIt) {
//          double distanceSq =
//            (coordIt->first - nodeHyp->x)*(coordIt->first - nodeHyp->x) +
//            (coordIt->second - nodeHyp->y)*(coordIt->second - nodeHyp->y);
//
//          if(distanceSq < 0.25 * m_minNodeSeparation * m_minNodeSeparation) {
//            placeholderStillValid = true;
//            break;
//          }
//        }
//        if(!placeholderStillValid && m_goalPlaceForCurrentPath != -1) {
//          /* Check if we are deleting the current goal hypothesis */
//          if (place->id != m_goalPlaceForCurrentPath) {
//            /* If it was not the goal just delete it */
//            log("deleting placeholder %d", place->id);
//            deletePlaceholder(place->id);
//          }
//          else {
//            /* If we are trying to delete the goal hypothesis only delete it
//             * if it is blocked, if the goal is in free space (ie no frontier
//             * there) we should still (try to) go to it */
//            NodeHypothesisPtr goalHyp = getHypFromPlaceID(place->id); 
//            if (goalHyp && !m_mapInterface->isCircleObstacleFree(goalHyp->x, goalHyp->y, -1)) {
//              log("deleting goal placeholder %d", place->id);
//              deletePlaceholder(place->id);
//            }
//          }
//        }
//      } catch (IceUtil::NullHandleException e) {
//        log("Place suddenly disappeared!\n");
//      }
//    }    
//
//  }
//  places.clear(); /* May contain dangling pointers */
//
//  log("refreshPlaceholders() exited");
//}

void PlaceManager::deletePlaceholder(int placeID) {
  // TODO: Delete connectivities

  log("deletePlaceholder entered (ID=%i)", placeID);
  PlacePtr place = _getPlace(placeID);
  NodeHypothesisPtr nodeHyp = _getHypForPlace(placeID);

  if (placeID == 0) {
    error("Error! Unknown Place on %i", __LINE__);
    return;
  }
  if (nodeHyp == 0) {
    error("Error! Unknown NodeHypothesis on %i", __LINE__);
    return;
  }

  log("deleting placeholder properties");
  deletePlaceholderProperties(placeID);

  _deletePlace(placeID);

  if (placeID == m_goalPlaceForCurrentPath){
    cancelMovement(true);
  }
  log("deletePlaceholder exited");
}

void 
PlaceManager::robotMoved(const cast::cdl::WorkingMemoryChange &objID)
{
  //log("robotMoved called");

  //If the robot is not currently executing a path transition
  //(but people-following, being joypadded or dragged along in Stage),
  //check if the robot has changed its closest node.
  //If so, reset the m_startNodeForCurrentPath
  NavData::FNodePtr curNode = getCurrentNavNode();
  if (curNode != 0) {
    if (!m_isPathFollowing) {
      m_startNodeForCurrentPath = curNode->nodeId;
    }
    else {
      // If we were following a path and changed nodes, we've arrived at a Place
      if (curNode->nodeId != m_startNodeForCurrentPath) {
        // Make sure we haven't processed this place already
        if (curNode->nodeId != m_currentNodeOnPath) {
          m_currentNodeOnPath = curNode->nodeId;
          processPlaceArrival(false);
        }
      }
    }
  }

  // Before the robot has moved, all nodes added to the WM are places.
  // Previously m_firstMovementRegistered was set to true on the first receipt of a robot pose
  // now we check that the robot has actually moved.
  if (!m_firstMovementRegistered){
    if (!m_robotInitialPoseReceived) {
      // store initial pose
      vector<NavData::RobotPose2dPtr> robotPoses;
      getMemoryEntries<NavData::RobotPose2d>(robotPoses, 0);

      if (robotPoses.size() != 0) {
        m_robotInitialX = robotPoses[0]->x;
        m_robotInitialY = robotPoses[0]->y;
        log("Initial robot position (%f, %f)", m_robotInitialX, m_robotInitialY);
        m_robotInitialPoseReceived = true;
      }
      else {
        // no pose available!
      }
      
    }
    else {
      // check if the robot has moved
      // get current pose
      vector<NavData::RobotPose2dPtr> robotPoses;
      getMemoryEntries<NavData::RobotPose2d>(robotPoses, 0);
      if (robotPoses.size() != 0) {
        double robotX = robotPoses[0]->x;
        double robotY = robotPoses[0]->y;
        double distance_squared = (m_robotInitialX - robotX)*(m_robotInitialX - robotX) + 
          (m_robotInitialY - robotY)*(m_robotInitialY - robotY);
        log("Squared distance from robot's initial position is %f", distance_squared);
        if (distance_squared > m_initialMovementThreshold) {
          log("Robot has moved more than the threshold distance");
	        m_firstMovementRegistered = true;
//	        processPlaceArrival(false);
          evaluateUnexploredPaths();
        }
      }
      else {
        // no pose available!
      }
    }
  }
  //log("robotMoved exited");
}

void 
PlaceManager::deletePlaceProperties(int placeID)
{
  log("deletePlaceProperties called (placeID = %i)", placeID);
  deletePlaceholderProperties(placeID);
  {
    m_PlacePropsMutex.lock();
    //Delete gateway property
    map<int, string>::iterator it = m_gatewayProperties.find(placeID);
    if (it != m_gatewayProperties.end()) {
      try {
	deleteFromWorkingMemory(it->second);
      }
      catch (Exception e) {
	log("Gateway property could not be deleted; already missing!");
      }
      m_gatewayProperties.erase(it);
    }
    m_PlacePropsMutex.unlock();
  }

  {
    //TODO: Delete connectivity properties
  }

  log("deletePlaceProperties exited");
}

void 
PlaceManager::deletePlaceholderProperties(int placeID)
{
  log("deletePlaceholderProperties called (placeID = %i)", placeID);
  m_PlacePropsMutex.lock();
  {  
    //Delete free space property
    map<int, string>::iterator it = m_freeSpaceProperties.find(placeID);
    if (it != m_freeSpaceProperties.end()) {
      try {
	deleteFromWorkingMemory (it->second);
      }
      catch (...) {
	log("Free space property could not be deleted; already missing!");
      }
      m_freeSpaceProperties.erase(it);
    }
  }
  {
    //Delete border property
    map<int, string>::iterator it = m_borderProperties.find(placeID);
    if (it != m_borderProperties.end()) {
      try {
	deleteFromWorkingMemory (it->second);
      }
      catch (...) {
	log("Border property could not be deleted; already missing!");
      }
      m_borderProperties.erase(it);
    }
  }
  {
    //Delete gateway property
    map<int, string>::iterator it = m_placeholderGatewayProperties.find(placeID);
    if (it != m_placeholderGatewayProperties.end()) {
      try {
	deleteFromWorkingMemory (it->second);
      }
      catch (...) {
	log("Gateway placeholder property could not be deleted; already missing!");
      }
      m_placeholderGatewayProperties.erase(it);
    }
  }
  m_PlacePropsMutex.unlock();
  log("deletePlaceholderProperties exited");
}

void
PlaceManager::addNewGatewayProperty(int placeID)
{
  log("addNewGatewayProperty(%i)", placeID);
  SpatialProperties::BinaryValuePtr trueValue = 
    new SpatialProperties::BinaryValue;
  SpatialProperties::BinaryValuePtr falseValue = 
    new SpatialProperties::BinaryValue;
  trueValue->value = true;
  falseValue->value = false;

  SpatialProperties::ValueProbabilityPair pair1 =
  { trueValue, 0.9 };
  SpatialProperties::ValueProbabilityPair pair2 =
  { falseValue, 0.1 };

  SpatialProperties::ValueProbabilityPairs pairs;
  pairs.push_back(pair1);
  pairs.push_back(pair2);

  SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
    new SpatialProperties::DiscreteProbabilityDistribution;
  discDistr->data = pairs;

  SpatialProperties::GatewayPlacePropertyPtr gwProp =
    new SpatialProperties::GatewayPlaceProperty;
  gwProp->placeId = placeID;
  gwProp->mapValue = trueValue;
  gwProp->mapValueReliable = 1;

  string newID = newDataID();
  addToWorkingMemory<SpatialProperties::GatewayPlaceProperty>(newID, gwProp);
  m_gatewayProperties[gwProp->placeId] = newID;
  log("addNewGatewayProperty exited");
}

PlacePtr
PlaceManager::PlaceServer::getCurrentPlace(const Ice::Current &_context) {
//  m_pOwner->lockComponent();
  PlacePtr currPlace = m_pOwner->getCurrentPlace();
//  m_pOwner->unlockComponent();

  return currPlace;
}

FrontierInterface::PlaceMembership
PlaceManager::PlaceServer::getPlaceMembership(double x, double y,
    const Ice::Current &_context) {
//  m_pOwner->lockComponent();
  FrontierInterface::PlaceMembership membership = m_pOwner->getPlaceMembership(x, y);
//  m_pOwner->unlockComponent();
  return membership;
}

int PlaceManager::PlaceServer::updatePlaceholderEdge(int placeholderId, const Ice::Current &_context) {
  int ret;

//  m_pOwner->lockComponent();
  ret = m_pOwner->updatePlaceholderEdge(placeholderId);
// m_pOwner->unlockComponent();

  return ret;
}

FrontierInterface::AdjacencyLists PlaceManager::PlaceServer::getAdjacencyLists(const Ice::Current &_context) {
  return m_pOwner->getAdjacencyLists();
}

void
PlaceManager::upgradePlaceholder(int placeID, NavData::FNodePtr newNode)
{
  log("  Upgrading Place %d from Placeholder status; associating with node %d", placeID, newNode->nodeId);
  string goalPlaceWMID = _getPlaceWMIDForPlace(placeID);

  try {
    log("lock 1");
    lockEntry(goalPlaceWMID, cdl::LOCKEDOD);
    deletePlaceholderProperties(placeID);

    // Re-read after lock
    PlacePtr place = getMemoryEntry<Place>(goalPlaceWMID);
    place->status = TRUEPLACE;
    log("overwrite 4: %s", goalPlaceWMID.c_str());
    _upgradePlaceholderMappings(placeID, place, newNode);

    checkUnassignedEdges(newNode->nodeId);

    unlockEntry(goalPlaceWMID);
    log("unlock 1");
  }
  catch (DoesNotExistOnWMException e) {
    log("The Place has disappeared! Re-adding it!");

    PlacePtr place = new Place;
    place->status = TRUEPLACE;

    _addPlaceWithNode(place, newNode);
  }

  // Need to make sure that no hypothetical connectivities remain, only
  // real ones (from the node map)
  {
    for (list<pair<int, int> >::iterator it = m_hypotheticalConnectivities.begin();
	    it != m_hypotheticalConnectivities.end(); it++) {
      if (it->second == placeID) {
	      // Hypothetical edge leading to new Place; 
	      // Check if there's a connectivity property in the
	      // opposite direction
	      // If so, that edge has already been processed
	      // and we can keep it
	      // Otherwise, the hypothesis should be discarded
	      string placeIDstr = concatenatePlaceIDs(it->second, it->first);
	      if (m_placeIDsToConnectivityWMID.count(placeIDstr) == 0) {
	        deleteConnectivityProperty(it->first, it->second);
	      }
      	it = m_hypotheticalConnectivities.erase(it);
      }
    }
  }

  // Delete placeholder properties for the Place in question
  deletePlaceholderProperties(placeID);
 
//  // 1: Free space properties
//  if (m_freeSpaceProperties.find(placeID) != m_freeSpaceProperties.end()) {
//    try {
//      lockEntry(m_freeSpaceProperties[placeID], cdl::LOCKEDOD);
//      deleteFromWorkingMemory(m_freeSpaceProperties[placeID]);
//    }
//    catch (DoesNotExistOnWMException) {
//      log("Error! gateway property was already missing!");
//    }
//    m_freeSpaceProperties.erase(placeID);
//  }
//
//  // 2: Border properties
//  if (m_borderProperties.find(placeID) != m_borderProperties.end()) {
//    try {
//      lockEntry(m_borderProperties[placeID], cdl::LOCKEDOD);
//      deleteFromWorkingMemory(m_borderProperties[placeID]);
//    }
//    catch (DoesNotExistOnWMException) {
//      log("Error! gateway property was already missing!");
//    }
//    m_borderProperties.erase(placeID);
//  }
}

void
PlaceManager::createConnectivityProperty(double cost, int place1ID, int place2ID)
{
  NavData::FNodePtr node1 = _getNodeForPlace(place1ID);
  NavData::FNodePtr node2 = _getNodeForPlace(place2ID);

  if (node1 != 0 && node2 != 0) {
    m_mapInterface->addConnection(node1->nodeId,node2->nodeId);
  }

  set<int> &place1Connectivities = m_connectivities[place1ID];
  if (place1Connectivities.find(place2ID) == 
      place1Connectivities.end()) {
    SpatialProperties::FloatValuePtr costValue1 = 
      new SpatialProperties::FloatValue;
    SpatialProperties::FloatValuePtr costValue2 = 
      new SpatialProperties::FloatValue;
    costValue1->value = cost;
    costValue2->value = numeric_limits<double>::infinity();

    SpatialProperties::ValueProbabilityPair pair1 =
    { costValue1, 0.9 };
    SpatialProperties::ValueProbabilityPair pair2 =
    { costValue2, 0.1 };

    SpatialProperties::ValueProbabilityPairs pairs;
    pairs.push_back(pair1);
    pairs.push_back(pair2);

    SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
      new SpatialProperties::DiscreteProbabilityDistribution;
    discDistr->data = pairs;

    SpatialProperties::ConnectivityPathPropertyPtr connectivityProp1 =
      new SpatialProperties::ConnectivityPathProperty;
    connectivityProp1->place1Id = place1ID;
    connectivityProp1->place2Id = place2ID;
    connectivityProp1->distribution = discDistr;
    connectivityProp1->mapValue = costValue1;
    connectivityProp1->mapValueReliable = 1;

    string newID = newDataID();
    log("Creating connectivity property between %i and %i", place1ID, place2ID);
    addToWorkingMemory<SpatialProperties::ConnectivityPathProperty>(newID, connectivityProp1);
    set<int> &place1Connectivities = m_connectivities[place1ID];
    place1Connectivities.insert(place2ID); 

    string placeIDstr = concatenatePlaceIDs(place1ID, place2ID);
    m_placeIDsToConnectivityWMID[placeIDstr] = newID;
  }
}

std::string PlaceManager::concatenatePlaceIDs(int place1ID, int place2ID) {
  std::stringstream concatenated;
  concatenated << place1ID << '_' << place2ID;
  return concatenated.str();
}

bool PlaceManager::deleteConnectivityProperty(int place1ID, int place2ID) {
  NavData::FNodePtr node1 = _getNodeForPlace(place1ID);
  NavData::FNodePtr node2 = _getNodeForPlace(place2ID);

  if (node1 != 0 && node2 != 0) {
    m_mapInterface->removeConnection(node1->nodeId,node2->nodeId);
  }

  set<int> &place1Connectivities = m_connectivities[place1ID];
  if(place1Connectivities.find(place2ID) != place1Connectivities.end()) {
    string placeIDstr = concatenatePlaceIDs(place1ID,place2ID);
    map<string,string>::iterator it = m_placeIDsToConnectivityWMID.find(placeIDstr);
    if(it != m_placeIDsToConnectivityWMID.end()) {
      try {  
          deleteFromWorkingMemory(it->second);
          place1Connectivities.erase(place2ID);
          m_placeIDsToConnectivityWMID.erase(placeIDstr);
      } catch (...) {
        error("Error! Can't delete from working memory!");
      }
      return true;
    }
  }
  return false;
}

int PlaceManager::addPlaceForNode(NavData::FNodePtr node) {
	PlacePtr p = new Place;   
	p->status = TRUEPLACE;

	PlaceID newPlaceID = _addPlaceWithNode(p, node);

	checkUnassignedEdges(node->nodeId);

	//Write the Gateway property if present
	if (node->gateway == 1) {
		addNewGatewayProperty(newPlaceID);
	}

	return newPlaceID;
}

void 
PlaceManager::mapLoadStatusOverwritten(const cdl::WorkingMemoryChange &wmc)
{
  log("mapLoadStatusOverwritten (%s)", wmc.address.id.c_str());
  try {
    lockEntry(wmc.address, cdl::LOCKEDOD);
    MapLoadStatusPtr statusStruct = getMemoryEntry<MapLoadStatus>(wmc.address);
    if (statusStruct->nodesWritten && !statusStruct->placesWritten) {
      // If we receive the signal that all nodes have been written, 
      // then we will already have received all those nodes
      // and all the Places corresponding to them will be written too
      statusStruct->placesWritten = true;
      overwriteWorkingMemory<MapLoadStatus>(wmc.address.id, statusStruct);
    }
    unlockEntry(wmc.address);
  }
  catch (DoesNotExistOnWMException) {
    getLogger()->warn("MapLoadStatus struct disappeared from WM! Map loading procedure may get stuck.");
  }
}

void
PlaceMapper::_checkConsistency(int line)
{
#ifdef DO_CONSISTENCY_CHECK
  IceUtil::Mutex::Lock lock(m_mutex);
  for (vector<PlaceMapEntry>::const_iterator it = entries.begin(); it != entries.end(); it++) {
    const PlaceMapEntry &entry = *it;

    if (entry.place == 0) {
      error("Consistency check on %i failed: place = 0!", line);
    }
    else {
      if (entry.placeWMID == "") {
	error("Consistency check on %i failed: placeWMID = \"\"!", line);
      }
      else {
	PlacePtr placeEntry = getMemoryEntry<Place>(entry.placeWMID);
	if (entry.place->id != placeEntry->id ||
	    entry.place->status != placeEntry->status) {
	  error("Consistency check on %i failed: local and WM places not equal!", line);
	}
      }
    }
    if (entry.node == 0) {
      if (entry.hyp == 0) {
	error("Consistency check on %i failed: node = hyp = 0!", line);
      }
      else {
	if (entry.hypWMID == "") {
	  error("Consistency check on %i failed: hypWMID = \"\"!", line);
	}
	else {
	  NodeHypothesisPtr hypEntry = getMemoryEntry<NodeHypothesis>(entry.hypWMID);
	  if (entry.hyp->x != hypEntry->x ||
	      entry.hyp->y != hypEntry->y ||
	      entry.hyp->hypID != hypEntry->hypID ||
	      entry.hyp->originPlaceID != hypEntry->originPlaceID) {
	    error("Consistency check on %i failed: local and WM hyps not equal!", line);
	    error("(%f, %f); (%f, %f); (%i, %i); (%i, %i)",
entry.hyp->x , hypEntry->x ,
	      entry.hyp->y , hypEntry->y ,
	      entry.hyp->hypID , hypEntry->hypID ,
	      entry.hyp->originPlaceID , hypEntry->originPlaceID);
	  }
	}
      }
    }
    else {
      if (entry.hyp != 0) {
	error("Consistency check on %i failed: node and hyp both != 0!", line);
      }
    }
  }
#endif
}


const PlacePtr
PlaceMapper::_getPlace(PlaceID id)
{
  log("Entering PlaceMapper::_getPlace(%i)", id);
  IceUtil::Mutex::Lock lock(m_mutex);

  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->place->id == id) 
      return new Place(*it->place);
  }

  return 0;
}

PlaceID
PlaceMapper::_getPlaceIDForNode(NodeID id)
{
  log("Entering PlaceMapper::_getPlaceIDForNode(%i)", id);
  IceUtil::Mutex::Lock lock(m_mutex);

  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->node != 0 && it->node->nodeId == id) 
      return it->place->id;
  }

  return -1;
}

const NavData::FNodePtr
PlaceMapper::_getNodeForPlace(PlaceID id)
{
  log("Entering PlaceMapper::_getNodeForPlace(PlaceID id=%i)", id);
  IceUtil::Mutex::Lock lock(m_mutex);

  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->place == 0) {
      error("Error! it->place = 0 on %i", __LINE__);
      return 0;
    }
    if (it->place->id == id)  {
      if (it->node == 0) 
	return 0;
      return new NavData::FNode(*it->node);
    }
  }

  return 0;
}

PlaceID
PlaceMapper::_getPlaceIDForHyp(HypID id)
{
  log("Entering PlaceMapper::_getPlaceIDForHyp(%i)", id);
  IceUtil::Mutex::Lock lock(m_mutex);

  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->hyp != 0 && it->hyp->hypID == id) 
      return it->place->id;
  }

  return -1;
}

const NodeHypothesisPtr
PlaceMapper::_getHypForPlace(PlaceID id)
{
  log("Entering PlaceMapper::_getHypIDForPlace(%i)", id);
  IceUtil::Mutex::Lock lock(m_mutex);

  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->place->id == id) {
      if (it->hyp == 0) 
	return 0;
      return new NodeHypothesis(*it->hyp);
    }
  }

  return 0;
}

string
PlaceMapper::_getPlaceWMIDForPlace(PlaceID id)
{
  log("Entering PlaceMapper::_getPlaceWMIDForPlace(%i)", id);
  IceUtil::Mutex::Lock lock(m_mutex);

  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->place->id == id) 
      return it->placeWMID;
  }

  return "";
}

//string
//PlaceMapper::_getNodeWMIDForPlace(PlaceID id)
//{
//  IceUtil::Mutex::Lock lock(m_mutex);
//
//  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
//    if (it->placeID == id) 
//      return it->nodeWMID;
//  }
//
//  return "";
//}

string
PlaceMapper::_getHypWMIDForPlace(PlaceID id)
{
  log("Entering PlaceMapper::_getHypWMIDForPlace(%i)", id);
  IceUtil::Mutex::Lock lock(m_mutex);

  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->place->id == id) 
      return it->hypWMID;
  }

  return "";
}

// Note: once you get this list, it's not guaranteed to remain correct!
void 
PlaceMapper::_getPlaceholders(std::vector<PlaceID> &ret)
{
  log("Entering PlaceMapper::_getPlaceholders(std::vector<PlaceID> &ret)");
  IceUtil::Mutex::Lock lock(m_mutex);
  
  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->place->status == PLACEHOLDER) {
      ret.push_back(it->place->id);
    }
  }
}

// Note: once you get this list, it's not guaranteed to remain correct!
void 
PlaceMapper::_getTruePlaces(std::vector<PlaceID> &ret)
{
  log("Entering PlaceMapper::_getTruePlaces(std::vector<PlaceID> &ret)");
  IceUtil::Mutex::Lock lock(m_mutex);
  
  for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
    if (it->place->status == TRUEPLACE) {
      ret.push_back(it->place->id);
    }
  }
}

void
PlaceMapper::_overwriteHypForPlace(PlaceID placeID, NodeHypothesisPtr hyp)
{
  log("Entering PlaceMapper::_overwriteHypForPlace(placeID=%i, hypID=%i)", placeID, hyp==0?-1:hyp->hypID);
  _checkConsistency(__LINE__);
  {
    IceUtil::Mutex::Lock lock(m_mutex);
    try {
      string hypWMID = "";

      for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
	if (it->place->id == placeID) {
	  hypWMID = it->hypWMID;
	  it->hyp = new NodeHypothesis(*hyp);
	  break;
	}
      }

      if (hypWMID == "") {
	error("Error! No hypothesis on %i", __LINE__);
	return;
      }

      lockEntry(hypWMID, cdl::LOCKEDOD);
      getMemoryEntry<NodeHypothesis>(hypWMID);
      overwriteWorkingMemory<NodeHypothesis>(hypWMID, hyp);
      unlockEntry(hypWMID);

    } catch (DoesNotExistOnWMException) {
      log("Error! Could not update hypothesis on WM - entry missing on %i!", __LINE__);
    }
  }
  _checkConsistency(__LINE__);
}

void
PlaceMapper::_updateNodeForPlace(PlaceID placeID, NavData::FNodePtr node)
{
  log("Entering PlaceMapper::_updateNodeForPlace(placeID=%i, nodeID=%i)", placeID, node==0?-1:node->nodeId);
  _checkConsistency(__LINE__);
  {
    IceUtil::Mutex::Lock lock(m_mutex);
      for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
	if (it->place->id == placeID) {
	  it->node = new NavData::FNode(*node);
	  return;
	}
      }
  }
  error("Error! entry not found on line %i", __LINE__);
}

void
PlaceMapper::_deletePlace(PlaceID id)
{
  log("Entering PlaceMapper::_deletePlace(%i)", id);
  string deletePlaceWMID = "";
  string deleteHypWMID = "";
  {
    _checkConsistency(__LINE__);
    IceUtil::Mutex::Lock lock(m_mutex);

    for(vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
      if (it->place->id == id) {
	if (it->hyp != 0) {
	  if (it->hypWMID == "") {
	    error("Error! hypWMID == \"\" at %i", __LINE__);
	  }
	  else {
	    deleteHypWMID = it->hypWMID;
	  }
	}
	else if (it->hypWMID != "") error("Error! hypWMID != \"\" at %i", __LINE__);

	deletePlaceWMID = it->placeWMID;
	entries.erase(it);
	break;
      }
    }
  }

  if (deletePlaceWMID == "") {
    error("Error! placeWMID not found on %i", __LINE__);
  }
  else {
    try {
      deleteFromWorkingMemory(deletePlaceWMID);
    }
    catch (DoesNotExistOnWMException)
    {
      log("Error! Could not find Place %s for deletion on %i!", deletePlaceWMID.c_str(), __LINE__);
    }

    if (deleteHypWMID != "")  {
      try {
	deleteFromWorkingMemory(deleteHypWMID);
      }
      catch (DoesNotExistOnWMException)
      {
	log("Error! Could not find NodeHypothesis %s for deletion on %i!", deleteHypWMID.c_str(), __LINE__);
      }
    }
  }
  _checkConsistency(__LINE__);
}

//void
//PlaceMapper::deleteHypothesis(PlaceID id)
//{
//  string hypWMID = "";
//  {
//  IceUtil::Mutex::Lock lock(m_mutex);
//
//  for(vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
//    if (it->place->id == id) {
//      if (it->hypWMID == "") {
//	getLogger()->warn("Warning! hypWMID == \"\" at %i", __LINE__);
//      }
//      else {
//	deleteFromWorkingMemory(it->hypWMID);
//      }
//
//      deleteFromWorkingMemory(it->placeWMID);
//      entries.erase(it);
//      
//      return;
//    }
//  }
//}

void
PlaceMapper::_upgradePlaceholderMappings(PlaceID placeID, 
    PlacePtr place, NavData::FNodePtr node)
{
  log("Entering PlaceMapper::_upgradePlaceholderMappings(placeID=%i)", placeID);
  string placeWMID = "";
  string hypWMID = "";

  {
    _checkConsistency(__LINE__);
    IceUtil::Mutex::Lock lock(m_mutex);

    for (vector<PlaceMapEntry>::iterator it = entries.begin(); it != entries.end(); it++) {
      if (it->place->id == placeID) {
	it->place = place;

	placeWMID = it->placeWMID;

	if (it->hypWMID != "") {
	  hypWMID = it->hypWMID;
	}
	it->hypWMID = "";

	it->hyp = 0;

	it->node = node;

	break;
      }
    }
  }

  if (placeWMID == "") {
    error("Error! Missing place on %i", __LINE__);
    return;
  }

  try {
    lockEntry(placeWMID, cdl::LOCKEDOD);
    getMemoryEntry<Place>(placeWMID);
    overwriteWorkingMemory(placeWMID, place);
    unlockEntry(placeWMID);
  }
  catch (DoesNotExistOnWMException)
  {
    log("Error! Could not find Place %s for overwrite on %i!", placeWMID.c_str(), __LINE__);
  }

  if (hypWMID == "")  {
    getLogger()->warn("Warning: hypothesis missing");
    return;
  }

  try {
    deleteFromWorkingMemory(hypWMID);
  }
  catch (DoesNotExistOnWMException)
  {
    log("Error! Could not find Hypothesis %s for overwrite on %i!", hypWMID.c_str(), __LINE__);
  }

  _checkConsistency(__LINE__);
}

PlaceID
PlaceMapper::_addPlaceWithNode(PlacePtr _place,
    NavData::FNodePtr _node)//, std::string _nodeWMID)
{
  log("Entering PlaceMapper::_addPlaceWithNode()");
  string newWMID;
  int newPlaceID;
  {
    _checkConsistency(__LINE__);
    IceUtil::Mutex::Lock lock(m_mutex);

    map<int, int>::iterator it = m_NodeIDToPlaceIDMap.find(_node->nodeId);
    if (it != m_NodeIDToPlaceIDMap.end()) {
      newPlaceID = it->second;
      if (newPlaceID > m_placeIDCounter) m_placeIDCounter = newPlaceID;
    }      
    else {
      newPlaceID = m_placeIDCounter;
    }

    m_placeIDCounter++;
    _place->id = newPlaceID;
    newWMID = newDataID();

    PlaceMapEntry newEntry;
    newEntry.place = _place;
    newEntry.node = _node;
    newEntry.hyp = 0;

    newEntry.placeWMID = newWMID;
//    newEntry.nodeWMID = _nodeWMID;
    newEntry.hypWMID = "";

    entries.push_back(newEntry);

    //TODO: maybe no need to lock this - will only be a problem if 
    //another thread is overwriting this
    log("Adding place %ld, based on FNode %ld, at %s", _place->id,
	_node->nodeId, newWMID.c_str());//, _nodeWMID);
    addToWorkingMemory<Place>(newWMID, _place);
  }
  _checkConsistency(__LINE__);
  return newPlaceID;
}

PlaceID 
PlaceMapper::_addPlaceWithHyp(PlacePtr _place, NodeHypothesisPtr _hyp)
{
  log("Entering PlaceMapper::_addPlaceWithHyp()");
  int newPlaceID;
  {
    _checkConsistency(__LINE__);
    IceUtil::Mutex::Lock lock(m_mutex);

    _hyp->hypID = m_hypIDCounter;
    m_hypIDCounter++;

    string newHypWMID = newDataID();
    addToWorkingMemory<NodeHypothesis>(newHypWMID, _hyp);

    newPlaceID = m_placeIDCounter;
    m_placeIDCounter++;
    _place->id = newPlaceID;
    string newWMID = newDataID();

    PlaceMapEntry newEntry;
    newEntry.place = _place;
    newEntry.node = 0;
    newEntry.hyp = _hyp;

    newEntry.placeWMID = newWMID;
//    newEntry.nodeWMID = "";
    newEntry.hypWMID = newHypWMID;

    entries.push_back(newEntry);

    //TODO: maybe no need to lock this - will only be a problem if 
    //another thread is overwriting this
    log("Adding place %ld, based on hypothesis %i, at %s",
	_place->id, _hyp->hypID, newWMID.c_str());
    addToWorkingMemory<Place>(newWMID, _place);
  }
  _checkConsistency(__LINE__);
  return newPlaceID;
}


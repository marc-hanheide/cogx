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
#include <FrontierInterface.hpp>
#include <Ice/Ice.h>
#include <float.h>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;
using namespace Ice;

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

PlaceManager::PlaceManager() : m_placeIDCounter(0), m_hypIDCounter(0), 
  m_startNodeForCurrentPath(-1), m_goalPlaceForCurrentPath(-1)
{
}

PlaceManager::~PlaceManager()
{
}

void
PlaceManager::configure(const std::map<std::string, std::string>& _config)
{
  log("Configure entered");

  if(_config.find("--max-frontier-dist") != _config.end()) {
    std::istringstream str(_config.find("--max-frontier-dist")->second);
    str >> m_maxFrontierDist;
  }
  else {
    m_maxFrontierDist = 2.5;
  }

  if(_config.find("--min-frontier-dist") != _config.end()) {
    std::istringstream str(_config.find("--min-frontier-dist")->second);
    str >> m_minFrontierDist;
  }
  else {
    m_minFrontierDist = 0.5;
  }

  if(_config.find("--min-frontier-length") != _config.end()) {
    std::istringstream str(_config.find("--min-frontier-length")->second);
    str >> m_minFrontierLength;
  }
  else {
    m_minFrontierLength = 0.0;
  }

  if(_config.find("--min-node-separation") != _config.end()) {
    std::istringstream str(_config.find("--min-node-separation")->second);
    str >> m_minNodeSeparation;
  }
  else {
    m_minNodeSeparation = 2.0;
  }

  if(_config.find("--hyp-path-length") != _config.end()) {
    std::istringstream str(_config.find("--hyp-path-length")->second);
    str >> m_hypPathLength;
  }
  else {
    m_hypPathLength = 1.5;
  }
}

void 
PlaceManager::start()
{
  FrontierInterface::PlaceInterfacePtr servant = new PlaceServer(this);
  registerIceServer<FrontierInterface::PlaceInterface, FrontierInterface::PlaceInterface>(servant);
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
}

void 
PlaceManager::stop()
{
}

void 
PlaceManager::runComponent()
{
}

void 
PlaceManager::newNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  if (oobj != 0) {
    
    processPlaceArrival(false, oobj->getData());

  }

}

void 
PlaceManager::modifiedNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  if (oobj != 0) {

    // Look for the place in the internal vector
    //for (unsigned int i = 0; i < m_Places.size(); i++) {
      //if (m_Places[i].m_data->id == oobj->getData()->nodeId) {
    for (map<int, NavData::FNodePtr>::iterator it =
	m_PlaceIDToNodeMap.begin();
	it != m_PlaceIDToNodeMap.end(); it++){
      if (it->second->nodeId == oobj->getData()->nodeId) {

        // Here we need to change mpore stuff, right now there is
        // nothing really that can be changed since all we have is the
        // id

        /*
        log("Modified place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
        overwriteWorkingMemory<SpatialData::Place>(m_Places[i].m_WMid, 
                                                   m_Places[i].m_data);
        */
        return;
      }
    }
    
    // If the node is not in our 
    log("Did not find the node from before, have to assume that we did not start early enough to catch it, will treat it as new");
    newNavNode(objID);

  }
}

void 
PlaceManager::deletedNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  if (oobj != 0) {

    // Look for the place in the internal vector
    //for (unsigned int i = 0; i < m_Places.size(); i++) {
      //if (m_Places[i].m_data->id == oobj->getData()->nodeId) {
    for (map<int, NavData::FNodePtr>::iterator it =
	m_PlaceIDToNodeMap.begin();
	it != m_PlaceIDToNodeMap.end(); it++){
      if (it->second->nodeId == oobj->getData()->nodeId) {
	unsigned int placeID = it->first;

	map<int, PlaceHolder>::iterator it2 = m_Places.find(placeID);
	if(it2 != m_Places.end()) {
	  deleteFromWorkingMemory(m_Places[placeID].m_WMid);
	  return;
	}
      }
    }
    
    log("WARNING: Did not find the node to delete!!!");
  }
}

bool 
FrontierPtCompare(const FrontierInterface::FrontierPtPtr &a, const FrontierInterface::FrontierPtPtr &b)
{
  return (*a).mWidth < (*b).mWidth;
}

void
PlaceManager::evaluateUnexploredPaths()
{
  // To be called whenever it is deemed that the robot needs to
  // check its surroundings for potential paths.

  // Typically, this procedure should be carried out after a move
  // action terminates, or at least whenever a new Place has been
  // created at the current location.

  // The intended function is: if we detect any
  // probable Paths that might be expected to yield new Places,
  // Placeholders should be created for these.

  log("PlaceManager::evaluateUnexploredPaths called");


  // Year 1 implementation: base exploration edges on Cure exploration
  // frontiers
  FrontierInterface::FrontierReaderPrx agg(getIceServer<FrontierInterface::FrontierReader>("spatial.control"));

  debug("Interface created");

  FrontierInterface::FrontierPtSeq points = agg->getFrontiers();

  log("Retrieved %i frontiers", points.size());

  NavData::FNodePtr curNode = getCurrentNavNode();
  if (curNode == 0) {
    log ("Could not determine current nav node!");
    return;
  }
  double nodeX = curNode->x;
  double nodeY = curNode->y;
  SpatialData::PlacePtr curPlace = getPlaceFromNodeID(curNode->nodeId);

  sort(points.begin(), points.end(), FrontierPtCompare);

  vector<FrontierInterface::NodeHypothesisPtr> hypotheses;
  getMemoryEntries<FrontierInterface::NodeHypothesis>(hypotheses);

  for (FrontierInterface::FrontierPtSeq::iterator it =
      points.begin(); it != points.end(); it++) {
    double x = (*it)->x;
    double y = (*it)->y;
    double nodeDistanceSq = (x - nodeX)*(x - nodeX) + (y - nodeY)*(y - nodeY);
    log("Evaluating frontier at (%f, %f) with square-distance %f and length %f", x, y, nodeDistanceSq, (*it)->mWidth);

    // Consider only frontiers within a certain maximum distance of the current
    // Nav node
    if (nodeDistanceSq < m_maxFrontierDist*m_maxFrontierDist) {
      // Consider only frontiers at a great enough distance to the current Nav node
      if (nodeDistanceSq > m_minFrontierDist*m_minFrontierDist) {
	// Consider only frontiers of sufficient size
	if ((*it)->mWidth > m_minFrontierLength) {
	  // Compare distance to all other hypotheses created for this node
	  double minDistanceSq = FLT_MAX;
	  for (vector<FrontierInterface::NodeHypothesisPtr>::iterator it2 =
	      hypotheses.begin(); it2 != hypotheses.end(); it2++) {
	    if ((*it2)->originPlaceID == curPlace->id) {
	      double distanceSq = ((*it2)->x - x)*((*it2)->x - x) + ((*it2)->y - y)*((*it2)->y - y);
	      if (distanceSq < minDistanceSq) minDistanceSq = distanceSq;
	    }
	  }
	  if (minDistanceSq > m_minNodeSeparation * m_minNodeSeparation) {
	    // Create new hypothetical node in the direction of the frontier
	    double newX = nodeX + m_hypPathLength * (x - nodeX)/sqrt(nodeDistanceSq);
	    double newY = nodeY + m_hypPathLength * (y - nodeY)/sqrt(nodeDistanceSq);
	    FrontierInterface::NodeHypothesisPtr newHyp = 
	      new FrontierInterface::NodeHypothesis;
	    newHyp->x = newX;
	    newHyp->y = newY;
	    newHyp->hypID = m_hypIDCounter;
	    newHyp->originPlaceID = curPlace->id;

	    log("Adding new hypothesis at (%f, %f) with ID %i", newHyp->x,
		newHyp->y, newHyp->hypID);

	    string newID = newDataID();
	    m_HypIDToWMIDMap[newHyp->hypID]=newID;
	    addToWorkingMemory<FrontierInterface::NodeHypothesis>(newID, newHyp);
	    hypotheses.push_back(newHyp);

	    // Create the Place struct corresponding to the hypothesis
	    PlaceHolder p;
	    p.m_data = new SpatialData::Place;   
	    //p.m_data->id = oobj->getData()->nodeId;

	    p.m_data->id = m_placeIDCounter;
	    m_PlaceIDToHypMap[m_placeIDCounter] = newHyp;
	    m_hypIDCounter++;

	    p.m_data->status = SpatialData::PLACEHOLDER;
	    p.m_WMid = newDataID();
	    log("Adding placeholder %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
	    addToWorkingMemory<SpatialData::Place>(p.m_WMid, p.m_data);

	    m_Places[m_placeIDCounter]=p;
	    m_placeIDCounter++;
	  }
	}
      }
    }
  }
}

NavData::FNodePtr
PlaceManager::getCurrentNavNode()
{
  vector<NavData::FNodePtr> nodes;
  getMemoryEntries<NavData::FNode>(nodes, 0);

  vector<NavData::RobotPose2dPtr> robotPoses;
  getMemoryEntries<NavData::RobotPose2d>(robotPoses, 0);

  if (robotPoses.size() == 0) {
    log("Could not find RobotPose!");
    return 0;
  }
  
  //Find the node closest to the robotPose
  double robotX = robotPoses[0]->x;
  double robotY = robotPoses[0]->y;
  double minDistance = FLT_MAX;
  NavData::FNodePtr ret = 0;

  for (vector<NavData::FNodePtr>::iterator it = nodes.begin();
      it != nodes.end(); it++) {
    double x = (*it)->x;
    double y = (*it)->y;

    double distance = (x - robotX)*(x-robotX) + (y-robotY)*(y-robotY);
    if (distance < minDistance) {
      ret = *it;
      minDistance = distance;
    }
  }
  return ret;
}

FrontierInterface::NodeHypothesisPtr PlaceManager::getHypFromPlaceID(int placeID)
{
  map<int, FrontierInterface::NodeHypothesisPtr>::iterator it =
    m_PlaceIDToHypMap.find(placeID);
  if (it == m_PlaceIDToHypMap.end()) {
    return 0;
  }
  else {
    return it->second;
  }
}

NavData::FNodePtr PlaceManager::getNodeFromPlaceID(int placeID)
{
  map<int, NavData::FNodePtr>::iterator it =
    m_PlaceIDToNodeMap.find(placeID);
  if (it == m_PlaceIDToNodeMap.end()) {
    return 0;
  }
  else {
    return it->second;
  }
}

SpatialData::PlacePtr PlaceManager::getPlaceFromNodeID(int nodeID)
{
  for(map<int, NavData::FNodePtr>::iterator it =
      m_PlaceIDToNodeMap.begin();
      it != m_PlaceIDToNodeMap.end(); it++) {
    if (it->second->nodeId == nodeID) {
      map<int, PlaceHolder>::iterator it2 = m_Places.find(it->first);
      if (it2 != m_Places.end()) {
	return getMemoryEntry<SpatialData::Place>(it2->second.m_WMid);
      }
    }
  }
  return 0;
}

SpatialData::PlacePtr PlaceManager::getPlaceFromHypID(int hypID)
{
  for(map<int, FrontierInterface::NodeHypothesisPtr>::iterator it =
      m_PlaceIDToHypMap.begin();
      it != m_PlaceIDToHypMap.end(); it++) {
    if (it->second->hypID == hypID) {
      map<int, PlaceHolder>::iterator it2 = m_Places.find(it->first);
      if (it2 != m_Places.end()) {
	return getMemoryEntry<SpatialData::Place>(it2->second.m_WMid);
      }
    }
  }
  return 0;
}

void PlaceManager::beginPlaceTransition(int goalPlaceID)
{
  log("beginPlaceTransition called; goal place ID=%i", goalPlaceID);
  // Check whether the transition is an explored or unexplored edge
  // Store where it was we came from
  map<int, FrontierInterface::NodeHypothesisPtr>::iterator it =
    m_PlaceIDToHypMap.find(goalPlaceID);
  if (it == m_PlaceIDToHypMap.end()) {
    // Goal is not unexplored
    map<int, NavData::FNodePtr>::iterator it2 =
      m_PlaceIDToNodeMap.find(goalPlaceID);
    if (it2 == m_PlaceIDToNodeMap.end()) {
      // Goal is unknown
      log("Could not find supposed goal Place!");
      return;
    }
  }
  NavData::FNodePtr curNode = getCurrentNavNode();
  m_startNodeForCurrentPath = curNode->nodeId;
  m_goalPlaceForCurrentPath = goalPlaceID;
  m_isPathFollowing = true;
}

//TODO: placeholder upgrade on new node

void PlaceManager::endPlaceTransition(int failed)
{
  log("endPlaceTransition called");
  m_isPathFollowing = false;
  // If successful and the transition was unexplored, 
  // check current node.
  // If the node is different from the previous,
  // change the Place struct to non-placeholder,
  // remove the NodeHypothesis struct and change the
  // m_PlaceToNodeIDMap/m_PlaceToHypIDMap members
  processPlaceArrival(failed);
}

void PlaceManager::processPlaceArrival(bool failed, NavData::FNodePtr newNavNode) 
{
  log("processPlaceArrival called");
  debug("m_goalPlaceForCurrentPath was %i", m_goalPlaceForCurrentPath);

  map<int, FrontierInterface::NodeHypothesisPtr>::iterator it =
    m_PlaceIDToHypMap.find(m_goalPlaceForCurrentPath);

  NavData::FNodePtr curNode = getCurrentNavNode();
  int curNodeId = curNode->nodeId;
  log("current node id: %i", curNodeId);

  if (it != m_PlaceIDToHypMap.end()) {
    //The transition was an exploration action
    FrontierInterface::NodeHypothesisPtr goalHyp = it->second;

    SpatialData::PlacePtr curPlace = getPlaceFromNodeID(curNodeId);


    if (curPlace == 0) { //No Place exists for current node -> it must be new

      //CASE 1: We were exploring a path, and a new node was discovered.
      //Stop moving, upgrade the placeholder we were heading for and connect it
      //to this new node, and delete the NodeHypotheses
      //TODO: Stop moving
      log("  CASE 1: New node discovered while exploring");
      m_isPathFollowing = false;
      map<int, PlaceHolder>::iterator it2 = m_Places.find(m_goalPlaceForCurrentPath);
      if (it2 != m_Places.end()) {
	log("  Upgrading Place %i from Placeholder status", m_goalPlaceForCurrentPath);
	it2->second.m_data->status = SpatialData::TRUEPLACE;
	string goalPlaceWMID = it2->second.m_WMid;
	overwriteWorkingMemory(goalPlaceWMID, it2->second.m_data);
	m_PlaceIDToNodeMap[m_goalPlaceForCurrentPath] = curNode;

	deleteFromWorkingMemory(m_HypIDToWMIDMap[goalHyp->hypID]); //Delete NodeHypothesis
	m_HypIDToWMIDMap.erase(goalHyp->hypID);
	m_PlaceIDToHypMap.erase(it);
      }
      else {
	log("Missing Placeholder placeholder!");
	m_goalPlaceForCurrentPath = -1;
	return;
      }
    }

    else if (!failed && curNodeId != m_startNodeForCurrentPath) {
      //CASE 2: We were exploring, but ended up in a known Place which was not
      //the one we started from.
      //Remove the NodeHypothesis and its Placeholder, and
      //send the Place merge message

      log("  CASE 2: Exploration action failed - place already known. Deleting Place %i",
	  m_goalPlaceForCurrentPath);

      deleteFromWorkingMemory(m_HypIDToWMIDMap[goalHyp->hypID]); //Delete NodeHypothesis
      m_HypIDToWMIDMap.erase(goalHyp->hypID); //Delete entry in m_HypIDToWMIDMap
      m_PlaceIDToHypMap.erase(it); //Delete entry in m_PlaceIDToHypMap

      //Delete Place struct and entry in m_Places
      map<int, PlaceHolder>::iterator it2 = m_Places.find(m_goalPlaceForCurrentPath);
      if (it2 != m_Places.end()) {
	deleteFromWorkingMemory(it2->second.m_WMid);
	m_Places.erase(it2);
      }
      else {
	log("Could not find Place to delete!");
      }

      //Prepare and send merge notification
      SpatialData::PlaceMergeNotificationPtr newNotify = new
	SpatialData::PlaceMergeNotification;
      newNotify->mergedPlaces.push_back(m_goalPlaceForCurrentPath);
      int currentPlaceId = getPlaceFromNodeID(curNodeId)->id;
      newNotify->mergedPlaces.push_back(currentPlaceId);
      newNotify->resultingPlace = currentPlaceId;
      log("Sending merge notification between places %i and %i", 
	  currentPlaceId, m_goalPlaceForCurrentPath);

      addToWorkingMemory<SpatialData::PlaceMergeNotification>(newDataID(), newNotify);
      //TODO:delete notifications sometime
    }

    else {//curPlace != 0 && (failed || curNodeId == m_startNodeForCurrentPath))
      //CASE 3: We were exploring but one way or another, we ended up
      //were we'd started.
      //Just delete the NodeHypothesis and its Placeholder.
      log("  CASE 3: Exploration action failed; couldn't reach goal. Deleting place %i",
	  m_goalPlaceForCurrentPath);

      deleteFromWorkingMemory(m_HypIDToWMIDMap[goalHyp->hypID]); //Delete NodeHypothesis
      m_HypIDToWMIDMap.erase(goalHyp->hypID); //Delete entry in m_HypIDToWMIDMap
      m_PlaceIDToHypMap.erase(it); //Delete entry in m_PlaceIDToHypMap

      //Delete Place struct and entry in m_Places
      map<int, PlaceHolder>::iterator it2 = m_Places.find(m_goalPlaceForCurrentPath);
      if (it2 != m_Places.end()) {
	deleteFromWorkingMemory(it2->second.m_WMid);
	m_Places.erase(it2);
      }
      else {
	log("Could not find Place to delete!");
      }
    }

  }
  else { // it == m_PlaceIDToHypMap.end()), i.e. the goal place was not hypothetical
    SpatialData::PlacePtr curPlace = getPlaceFromNodeID(curNodeId);

    if (curPlace == 0) { 
      //CASE 4: We were *not* exploring, but a new node was discovered.
      //We may have been going between known Places, or following a person
      //or pushed around in Stage.
      //Create a new Place for this node. If the node matches a hypothesis
      //belonging to the Place we just came from, upgrade that node as in
      //Case 1, above.
      log("  CASE 4: Node (%d) found while not exploring", curNodeId);

      //Check the previous Place for NodeHypotheses matching this one
      bool foundHypothesis = 0;

      vector<FrontierInterface::NodeHypothesisPtr> hyps;
      getMemoryEntries<FrontierInterface::NodeHypothesis>(hyps);

      if (m_startNodeForCurrentPath >= 0) {
	SpatialData::PlacePtr prevPlace = getPlaceFromNodeID(m_startNodeForCurrentPath);
	NavData::FNodePtr prevNode = getNodeFromPlaceID(prevPlace->id);
	for(vector<FrontierInterface::NodeHypothesisPtr>::iterator it2 =
	    hyps.begin(); it2 != hyps.end(); it2++) {
	  FrontierInterface::NodeHypothesisPtr hyp = *it2;
	  if (prevPlace->id == hyp->originPlaceID) {
	    double distSq = (curNode->x-hyp->x)*(curNode->x-hyp->x) + 
	      (curNode->y-hyp->y)*(curNode->y-hyp->y);
	    log("  checking hypothesis %i with distance %f (%f,%f)-(%f,%f)",
		hyp->hypID, distSq, curNode->x, curNode->y, hyp->x, hyp->y);
	    if (distSq < 0.25*m_minNodeSeparation*m_minNodeSeparation) {
	      //Close enough 
	      //FIXME: should really check just bearing, not distance (because
	      //of m_hypPathLength
	      SpatialData::PlacePtr placeholder = getPlaceFromHypID(hyp->hypID);
	      if (placeholder == 0) {
		log("Could not find placeholder to upgrade");
		m_goalPlaceForCurrentPath = -1;
		return;
	      }

	      log("Upgrading Placeholder with id %d, associating with node %d",
		  (int)placeholder->id, curNodeId);

	      map<int, PlaceHolder>::iterator it3 = m_Places.find(placeholder->id);
	      if (it3 != m_Places.end()) {
		it3->second.m_data->status = SpatialData::TRUEPLACE;
		string placeholderWMID = it3->second.m_WMid;
		overwriteWorkingMemory(placeholderWMID, it3->second.m_data);
		m_PlaceIDToNodeMap[placeholder->id] = curNode;

		deleteFromWorkingMemory(m_HypIDToWMIDMap[hyp->hypID]); //Delete NodeHypothesis
		m_HypIDToWMIDMap.erase(hyp->hypID);
		m_PlaceIDToHypMap.erase(placeholder->id);
	      }
	      else {
		log("Could not find Placeholder placeholder!");
		m_goalPlaceForCurrentPath = -1;
		return;
	      }
	      foundHypothesis = true;
	      break;
	    }
	  }
	}
      }

      if (!foundHypothesis) {
	//Found no prior hypothesis to upgrade. Create new Place instead.

	PlaceHolder p;
	p.m_data = new SpatialData::Place;   
	//p.m_data->id = oobj->getData()->nodeId;

	p.m_data->id = m_placeIDCounter;
	m_PlaceIDToNodeMap[m_placeIDCounter] = newNavNode;

	p.m_data->status = SpatialData::TRUEPLACE;
	p.m_WMid = newDataID();
	log("Adding place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
	addToWorkingMemory<SpatialData::Place>(p.m_WMid, p.m_data);

	m_Places[m_placeIDCounter] = p;
	m_placeIDCounter++;
      }
    }
    else {
      // We weren't exploring, and the place was known before - don't
      // do anything.
      // (Could check whether we ended up in the expected Place, but
      // that's for the future)
    }

  }

  evaluateUnexploredPaths();
  m_goalPlaceForCurrentPath = -1;
}


FrontierInterface::NodeHypothesisPtr 
PlaceManager::PlaceServer::getHypFromPlaceID(int placeID,
    const Ice::Current &_context) {
  return m_pOwner->getHypFromPlaceID(placeID);
}

NavData::FNodePtr
PlaceManager::PlaceServer::getNodeFromPlaceID(int placeID,
    const Ice::Current &_context) {
  return m_pOwner->getNodeFromPlaceID(placeID);
}

SpatialData::PlacePtr 
PlaceManager::PlaceServer::getPlaceFromNodeID(int nodeID,
    const Ice::Current &_context)
{
  return m_pOwner->getPlaceFromNodeID(nodeID);
}

SpatialData::PlacePtr PlaceManager::PlaceServer::getPlaceFromHypID(int hypID,
    const Ice::Current &_context)
{
  return m_pOwner->getPlaceFromHypID(hypID);
}

void 
PlaceManager::PlaceServer::beginPlaceTransition(int goalPlaceID, const Ice::Current &_context)
{
  m_pOwner->beginPlaceTransition(goalPlaceID);
}

void 
PlaceManager::PlaceServer::endPlaceTransition(int failed, const Ice::Current &_context)
{
  m_pOwner->endPlaceTransition(failed);
}

void 
PlaceManager::robotMoved(const cast::cdl::WorkingMemoryChange &objID)
{
  //If the robot is not currently executing a path transition
  //(but people-following, being joypadded or dragged along in Stage),
  //check if the robot has changed its closest node.
  //If so, reset the m_startNodeForCurrentPath
  if (!m_isPathFollowing) {
    m_startNodeForCurrentPath = getCurrentNavNode()->nodeId;
  }
}
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
#include <SpatialProperties.hpp>
#include <Rendezvous.h>
#include <Ice/Ice.h>
#include <float.h>
#include <limits>

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

PlaceManager::PlaceManager() : m_placeIDCounter(0), 
  m_hypIDCounter(0), 
  m_isPathFollowing(false),
  m_startNodeForCurrentPath(-1),
  m_goalPlaceForCurrentPath(-1),
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
  debug("Configure entered");

  if(_config.find("--no-local-maps") != _config.end()) {
    m_useLocalMaps = false;
  }
  else {
    m_useLocalMaps = true;
  }

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

  m_bNoPlaceholders = false;
  if(_config.find("--no-placeholders") != _config.end()) {
    m_bNoPlaceholders = true;
  }

  if(_config.find("--hyp-path-length") != _config.end()) {
    std::istringstream str(_config.find("--hyp-path-length")->second);
    str >> m_hypPathLength;
  }
  else {
    m_hypPathLength = 1.5;
  }

  FrontierInterface::PlaceInterfacePtr servant = new PlaceServer(this);
  registerIceServer<FrontierInterface::PlaceInterface, FrontierInterface::PlaceInterface>(servant);
}

void 
PlaceManager::start()
{
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

  frontierReader = FrontierInterface::FrontierReaderPrx(getIceServer<FrontierInterface::FrontierReader>("spatial.control"));
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

void 
PlaceManager::runComponent()
{
  debug("PlaceManager::runComponent: now at start of runComponent");

  Rendezvous *rv = new Rendezvous(*this);
  rv->addChangeFilter(
      createLocalTypeFilter<NavData::FNode>(cdl::ADD));
  cdl::WorkingMemoryChange change = rv->wait();
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(change.address);


}

void 
PlaceManager::newNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  try {
    NavData::FNodePtr oobj =
      getMemoryEntry<NavData::FNode>(objID.address);
    if (m_firstMovementRegistered) {
      debug("newNavNode called");
      
      if (oobj != 0) {
	processPlaceArrival(false);
      }
    }
    else {
      
      // Special case: Robot hasn't yet moved; this must be a loaded map
      // node. Just add it as a Place.
      PlaceHolder p;
      p.m_data = new SpatialData::Place;   
      //p.m_data->id = oobj->getData()->nodeId;
      
      
      int newPlaceID = m_placeIDCounter;
      m_placeIDCounter++;
      p.m_data->id = newPlaceID;
      m_PlaceIDToNodeMap[newPlaceID] = oobj;

      p.m_data->status = SpatialData::TRUEPLACE;
      p.m_WMid = newDataID();
      log("Adding place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
      addToWorkingMemory<SpatialData::Place>(p.m_WMid, p.m_data);

      m_Places[newPlaceID] = p;

      if(oobj->gateway == 1) {
	addNewGatewayProperty(p.m_data->id);			
      }

    }
  }
  catch (DoesNotExistOnWMException) {
    log("Error! Nav node missing from WM!");
  }
    debug("newNavNode exited");
}

void
PlaceManager::cancelMovement()
{
  debug("CancelMovement called");

  m_goalPlaceForCurrentPath = -1;
  m_isPathFollowing = false;

  // Stop the robot
  vector<CASTData<SpatialData::NavCommand> > commands;
  getMemoryEntriesWithData<SpatialData::NavCommand>(commands, "spatial.sa");
  for (vector<CASTData<SpatialData::NavCommand> >::iterator it =
      commands.begin(); it != commands.end(); it++) {
    try {
      debug("locking");
      lockEntry(it->getID(), cdl::LOCKEDODR);
      debug("locked");
      SpatialData::NavCommandPtr ptr = it->getData();
      if (ptr->cmd == SpatialData::GOTOPLACE &&
	  ptr->comp == SpatialData::COMMANDINPROGRESS) {
	ptr->comp = SpatialData::COMMANDSUCCEEDED;
	debug("overwrite 1: %s", it->getID().c_str());
	overwriteWorkingMemory<SpatialData::NavCommand>(it->getID(), ptr);
      }
      debug("unlocking");
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
  debug("CancelMovement exited");
}

void 
PlaceManager::modifiedNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  debug("modifiedNavNode called");
  try {
    lockEntry(objID.address.id, cdl::LOCKEDODR);
    NavData::FNodePtr oobj =
      getMemoryEntry<NavData::FNode>(objID.address);
    unlockEntry(objID.address.id);

    if (oobj != 0) {

      for (map<int, NavData::FNodePtr>::iterator it =
	  m_PlaceIDToNodeMap.begin();
	  it != m_PlaceIDToNodeMap.end(); it++){
	NavData::FNodePtr node = it->second;
	debug("nodeID = %i", node->nodeId);
	if (node->nodeId == oobj->nodeId) {
	  if (oobj->gateway == 1) {
	    // Has gained gateway status; add gateway property to WM
	    SpatialData::PlacePtr place = getPlaceFromNodeID(oobj->nodeId);
	    if (place != 0) {
	      if (m_gatewayProperties.find(place->id) == m_gatewayProperties.end()) {
		addNewGatewayProperty(place->id);
	      }
	    }
	    else {
	      log("Error! FNode became gateway, but could not find Place to correspond!");
	    }
	  }

	  it->second = oobj;

	  /*
	     log("Modified place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
	     overwriteWorkingMemory<SpatialData::Place>(m_Places[i].m_WMid, 
	     m_Places[i].m_data);
	   */
	  if (node->x != oobj->x || node->y != oobj->y) {
	    // If the node has been moved, frontiers must be reevaluated
	    // and, if necessary, moved.
	    evaluateUnexploredPaths();
	  }

	  unlockEntry(objID.address.id);
	  debug("modifiedNavNode exited");
	  return;
	}
      }

      // If the node is not in our 
      log("Did not find the node from before, have to assume that we did not start early enough to catch it, will treat it as new");
      newNavNode(objID);

      processPlaceArrival(false);

    }
    unlockEntry(objID.address.id);
  }
  catch (DoesNotExistOnWMException e) {
    log("Couldn't find supposedly modified node!");
  }
  debug("modifiedNavNode exited");
}

void 
PlaceManager::deletedNavNode(const cast::cdl::WorkingMemoryChange &objID)
{
  debug("deletedNavNode called");
  //TODO: This will never work!
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);

  if (oobj != 0) {

    // Look for the place in the internal vector
    for (map<int, NavData::FNodePtr>::iterator it =
	m_PlaceIDToNodeMap.begin();
	it != m_PlaceIDToNodeMap.end(); it++){
      if (it->second->nodeId == oobj->getData()->nodeId) {
	unsigned int placeID = it->first;

	map<int, PlaceHolder>::iterator it2 = m_Places.find(placeID);
	if(it2 != m_Places.end()) {
	  deleteFromWorkingMemory(m_Places[placeID].m_WMid);
	  debug("deletedNavNode exited");
	  return;
	}
	m_PlaceIDToNodeMap.erase(it);
	debug("deletedNavNode exited");
	return;
      }
    }

    log("WARNING: Did not find the node to delete!!!");
  }
  debug("deletedNavNode exited");
}

void 
PlaceManager::newEdge(const cast::cdl::WorkingMemoryChange &objID)
{
  debug("newEdge called");
  try {
    lockEntry(objID.address.id, cdl::LOCKEDODR);
    NavData::AEdgePtr oobj =
      getMemoryEntry<NavData::AEdge>(objID.address);
    unlockEntry(objID.address);

    if (oobj != 0) {
      try {
	int newEdgeStartId = getPlaceFromNodeID(oobj->startNodeId)->id;
	int newEdgeEndId = getPlaceFromNodeID(oobj->endNodeId)->id;
	double newEdgeCost = oobj->cost;

	createConnectivityProperty(newEdgeCost, newEdgeStartId, newEdgeEndId);
	createConnectivityProperty(newEdgeCost, newEdgeEndId, newEdgeStartId);
      }
      catch (IceUtil::NullHandleException e) {
	log("Error! edge objects disappeared from memory!");
      }
    }
    unlockEntry(objID.address.id);
  } catch (DoesNotExistOnWMException e) {
    log("Error! edge object disappeared!");
  }
  debug("newEdge exited");
}

void 
PlaceManager::modifiedEdge(const cast::cdl::WorkingMemoryChange &objID)
{
  debug("modifiedEdge called");
  // This will probably never be called...
  /*
     shared_ptr<CASTData<NavData::AEdge> > oobj =
     getWorkingMemoryEntry<NavData::AEdge>(objID.address);

     if (oobj != 0) {

  // Look for the place in the internal vector
   */
  debug("modifiedEdge exited");
}

void 
PlaceManager::newObject(const cast::cdl::WorkingMemoryChange &objID)
{
  debug("newObject called");
  try {
    lockEntry(objID.address, cdl::LOCKEDODR);
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
      debug("newObject exited");
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
      SpatialData::PlacePtr place = getPlaceFromNodeID(closestNode->nodeId);
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
	  debug("newObject exited");
	  return;
	}
      }
      else {
	log("Could not find Place for object!");
	unlockEntry(objID.address.id);
	debug("newObject exited");
	return;
      }
    }
    else {
      log("Could not find Node for object!");
      unlockEntry(objID.address.id);
      debug("newObject exited");
      return;
    }
    unlockEntry(objID.address.id);
  }
  catch (DoesNotExistOnWMException e) {
    log ("Object disappeared!");
  }
  debug("newObject exited");
}

bool 
FrontierPtCompare(const FrontierInterface::FrontierPtPtr &a, const FrontierInterface::FrontierPtPtr &b)
{
  return (*a).mWidth < (*b).mWidth;
}

void
PlaceManager::evaluateUnexploredPaths()
{
  if (!m_bNoPlaceholders) {

  FrontierInterface::FrontierPtSeq points;
  try {
    // To be called whenever it is deemed that the robot needs to
    // check its surroundings for potential paths.

    // Typically, this procedure should be carried out after a move
    // action terminates, or at least whenever a new Place has been
    // created at the current location.

    // The intended function is: if we detect any
    // probable Paths that might be expected to yield new Places,
    // Placeholders should be created for these.

    debug("PlaceManager::evaluateUnexploredPaths called");

    try {
      // Year 1 implementation: base exploration edges on Cure exploration
      // frontiers
      points = frontierReader->getFrontiers();

      log("Retrieved %i frontiers", points.size());
    }
    catch (Exception e) {
      log("Error! getFrontiers failed!");
      debug("PlaceManager::evaluateUnexploredPaths exited");
      return;
    }
  }
  catch (Exception e) {
    log("Error! getFrontiers failed!");
    return;
  }

  NavData::FNodePtr curNode = getCurrentNavNode();
  if (curNode == 0) {
    log ("Could not determine current nav node!");
    debug("PlaceManager::evaluateUnexploredPaths exited");
    return;
  }
  int curNodeId = curNode->nodeId;
  double nodeX = curNode->x;
  double nodeY = curNode->y;
  SpatialData::PlacePtr curPlace = getPlaceFromNodeID(curNodeId);
  if (curPlace != 0) {
    int currentPlaceID = curPlace->id;

    sort(points.begin(), points.end(), FrontierPtCompare);

    vector<FrontierInterface::NodeHypothesisPtr> hypotheses;
    debug("evaluateUnexploredPaths:1");
    getMemoryEntries<FrontierInterface::NodeHypothesis>(hypotheses);
    debug("evaluateUnexploredPaths:2");

    // Find which hypotheses exist that originate in the current Place
    vector<FrontierInterface::NodeHypothesisPtr> relevantHyps;
    for (vector<FrontierInterface::NodeHypothesisPtr>::iterator hypIt =
	hypotheses.begin(); hypIt != hypotheses.end(); hypIt++) {
      try {
	if ((*hypIt)->originPlaceID == currentPlaceID) {
	  relevantHyps.push_back(*hypIt);
	}
      }
      catch (IceUtil::NullHandleException e) {
	log("Error: hypothesis suddenly disappeared!");
      }
    }

    // Loop over currently observed frontiers
    for (FrontierInterface::FrontierPtSeq::iterator frontierIt =
	points.begin(); frontierIt != points.end(); frontierIt++) {
      FrontierInterface::FrontierPtPtr frontierPt = *frontierIt;
      double x = frontierPt->x;
      double y = frontierPt->y;
      double nodeDistanceSq = (x - nodeX)*(x - nodeX) + (y - nodeY)*(y - nodeY);
      log("Evaluating frontier at (%f, %f) with square-distance %f and length %f", x, y, nodeDistanceSq, frontierPt->mWidth);

      double newX = nodeX + m_hypPathLength * (x - nodeX)/sqrt(nodeDistanceSq);
      double newY = nodeY + m_hypPathLength * (y - nodeY)/sqrt(nodeDistanceSq);

      // Consider only frontiers with an open path to them
      if (frontierPt->mState == FrontierInterface::FRONTIERSTATUSOPEN) {
	// Consider only frontiers within a certain maximum distance of the current
	// Nav node
	if (nodeDistanceSq < m_maxFrontierDist*m_maxFrontierDist) {
	  // Consider only frontiers at a great enough distance to the current Nav node
	  if (nodeDistanceSq > m_minFrontierDist*m_minFrontierDist) {
	    // Consider only frontiers of sufficient size
	    if (frontierPt->mWidth > m_minFrontierLength) {
	      double minDistanceSq = FLT_MAX;
	      int minDistID = -1;

	      // Check already-rejected hypotheses for this node
	      if (m_rejectedHypotheses.find(curNodeId) != m_rejectedHypotheses.end()) {
		for (vector<FrontierInterface::NodeHypothesisPtr>::iterator rejectedHypIt =
		    m_rejectedHypotheses[curNodeId].begin(); rejectedHypIt != m_rejectedHypotheses[curNodeId].end(); rejectedHypIt++) {
		  double distanceSq = ((*rejectedHypIt)->x - newX)*((*rejectedHypIt)->x - newX) + ((*rejectedHypIt)->y - newY)*((*rejectedHypIt)->y - newY);
		  log ("distanceSq = %f", distanceSq);
		  if (distanceSq < minDistanceSq) {
		    minDistanceSq = distanceSq;
		  }
		}
	      }
	      // Compare distance to all other hypotheses created for this node
	      for (vector<FrontierInterface::NodeHypothesisPtr>::iterator extantHypIt =
		  relevantHyps.begin(); extantHypIt != relevantHyps.end(); extantHypIt++) {
		FrontierInterface::NodeHypothesisPtr extantHyp = *extantHypIt;
		try {
		  if (extantHyp->originPlaceID == currentPlaceID) {
		    double distanceSq = (extantHyp->x - newX)*(extantHyp->x - newX) + (extantHyp->y - newY)*(extantHyp->y - newY);
		    debug("2distanceSq = %f", distanceSq);
		    if (distanceSq < minDistanceSq) {
		      minDistanceSq = distanceSq;
		      minDistID = extantHyp->hypID;
		    }
		  }
		}
		catch (IceUtil::NullHandleException e) {
		  log("Error: hypothesis suddenly disappeared!");
		}
	      }

	      if (minDistanceSq > m_minNodeSeparation * m_minNodeSeparation) {
		// Create new hypothetical node in the direction of the frontier
		FrontierInterface::NodeHypothesisPtr newHyp = 
		  new FrontierInterface::NodeHypothesis;
		newHyp->x = newX;
		newHyp->y = newY;
		newHyp->hypID = m_hypIDCounter;
		newHyp->originPlaceID = currentPlaceID;

		log("Adding new hypothesis at (%f, %f) with ID %i", newHyp->x,
		    newHyp->y, newHyp->hypID);

		string newID = newDataID();
		m_HypIDToWMIDMap[newHyp->hypID]=newID;
		hypotheses.push_back(newHyp);
		relevantHyps.push_back(newHyp);

		// Create the Place struct corresponding to the hypothesis
		PlaceHolder p;
		p.m_data = new SpatialData::Place;   
		//p.m_data->id = oobj->getData()->nodeId;

		int newPlaceID = m_placeIDCounter;
		m_placeIDCounter++;
		p.m_data->id = newPlaceID;
		m_PlaceIDToHypMap[newPlaceID] = newHyp;
		m_hypIDCounter++;

		// Add connectivity property (one-way)
		createConnectivityProperty(m_hypPathLength, currentPlaceID, newPlaceID);
		p.m_data->status = SpatialData::PLACEHOLDER;
		p.m_WMid = newDataID();
		log("Adding placeholder %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
		addToWorkingMemory<SpatialData::Place>(p.m_WMid, p.m_data);
		addToWorkingMemory<FrontierInterface::NodeHypothesis>(newID, newHyp);

		m_Places[newPlaceID]=p;
	      }
	      else if (minDistID != -1) {
		// Modify the extant hypothesis that best matched the
		// new position indicated by the frontier
		try {
		  FrontierInterface::NodeHypothesisPtr updatedHyp = 
		    getMemoryEntry<FrontierInterface::NodeHypothesis>(m_HypIDToWMIDMap[minDistID]);

		  updatedHyp->x = newX;
		  updatedHyp->y = newY;

		  log("Updating hypothesis at (%f, %f) with ID %i", updatedHyp->x,
		      updatedHyp->y, updatedHyp->hypID);

		  overwriteWorkingMemory<FrontierInterface::NodeHypothesis>(m_HypIDToWMIDMap[minDistID], updatedHyp);
		}
		catch (DoesNotExistOnWMException) {
		  log("Error! Could not update hypothesis on WM - entry missing!");
		}
	      }
	    }
	  }
	}
      }
      else {
	log("Frontier not reachable, skipping");
      }

      if (m_useLocalMaps && hypothesisEvaluator != 0) {
	// Now, update/create Placeholder properties for all Placeholders
	// reachable from this Place.
	for (vector<FrontierInterface::NodeHypothesisPtr>::iterator hypothesisIt =
	    relevantHyps.begin(); hypothesisIt != relevantHyps.end(); hypothesisIt++) {
	  try {
	    int hypID = (*hypothesisIt)->hypID;

	    SpatialData::PlacePtr placeholder = getPlaceFromHypID(hypID);

	    if (placeholder != 0) {

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
		  foundFSIt = m_freeSpaceProperties.find(hypID);
		if (foundFSIt != m_freeSpaceProperties.end()) {
		  try {
		    debug("lock 6");
		    lockEntry(foundFSIt->second, cdl::LOCKEDODR);
		    debug("evaluateUnexploredPaths:3");
		    SpatialProperties::AssociatedSpacePlaceholderPropertyPtr
		      freeProp = getMemoryEntry
		      <SpatialProperties::AssociatedSpacePlaceholderProperty>
		      (foundFSIt->second);
		    debug("evaluateUnexploredPaths:4");

		    // Property exists; change it
		    freeProp->distribution = discDistr;
		    freeProp->placeId = placeholder->id;
		    freeProp->mapValue = freespacevalue;
		    freeProp->mapValueReliable = 1;
		    debug("overwrite 2: %s", foundFSIt->second.c_str());
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
		    unlockEntry(foundFSIt->second);
		    debug("unlock 6");
		  }
		  catch(DoesNotExistOnWMException e) {
		    log("Property missing!");
		  }
		}
		else {
		  SpatialProperties::AssociatedSpacePlaceholderPropertyPtr freeProp =
		    new SpatialProperties::AssociatedSpacePlaceholderProperty;
		  freeProp->distribution = discDistr;
		  freeProp->placeId = placeholder->id;
		  freeProp->mapValue = freespacevalue;
		  freeProp->mapValueReliable = 1;

		  string newID = newDataID();
		  addToWorkingMemory<SpatialProperties::AssociatedSpacePlaceholderProperty>
		    (newID, freeProp);
		  m_freeSpaceProperties[hypID] = newID;
		}
	      }

	      {
		//Frontier length property
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
		  foundUnexpIt = m_borderProperties.find(hypID);
		if (foundUnexpIt != m_borderProperties.end()) {
		  try {
		    debug("lock 7");
		    lockEntry(foundUnexpIt->second, cdl::LOCKEDODR);
		    debug("evaluateUnexploredPaths:5");
		    SpatialProperties::AssociatedBorderPlaceholderPropertyPtr
		      borderProp = getMemoryEntry
		      <SpatialProperties::AssociatedBorderPlaceholderProperty>
		      (foundUnexpIt->second);
		    debug("evaluateUnexploredPaths:6");

		    // Property exists; change it
		    borderProp->distribution = discDistr;
		    borderProp->placeId = placeholder->id;
		    borderProp->mapValue = bordervalue;
		    borderProp->mapValueReliable = 1;
		    debug("overwrite 3: %s", foundUnexpIt->second.c_str());
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

		    unlockEntry(foundUnexpIt->second);
		    debug("unlock 7");
		  }
		  catch(DoesNotExistOnWMException e) {
		    log("Property missing!");
		  }

		}
		else {
		  SpatialProperties::AssociatedBorderPlaceholderPropertyPtr borderProp =
		    new SpatialProperties::AssociatedBorderPlaceholderProperty;
		  borderProp->distribution = discDistr;
		  borderProp->placeId = placeholder->id;
		  borderProp->mapValue = bordervalue;
		  borderProp->mapValueReliable = 1;

		  string newID = newDataID();
		  addToWorkingMemory<SpatialProperties::AssociatedBorderPlaceholderProperty>
		    (newID, borderProp);
		  m_borderProperties[hypID] = newID;
		}
	      }
	    }
	  }
	  catch (IceUtil::NullHandleException) {
	    log("Error! Couldn't evaluate hypothesis; it disappeared!");
	  }
	}
      }
    }
  }
  debug("PlaceManager::evaluateUnexploredPaths exited");
  //  catch(CASTException &e) {
  //    println(e.what());
  //    println(e.message);
  //    abort();
  //
  //  }
  }
}

NavData::FNodePtr
PlaceManager::getCurrentNavNode()
{
  debug("getCurrentNavNode called");
  vector<NavData::FNodePtr> nodes;
  debug("1");
  getMemoryEntries<NavData::FNode>(nodes, 0);

  vector<NavData::RobotPose2dPtr> robotPoses;
  debug("2");
  getMemoryEntries<NavData::RobotPose2d>(robotPoses, 0);
  debug("3");

  if (robotPoses.size() == 0) {
    log("Could not find RobotPose!");
    debug("getCurrentNavNode exited");
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
      log("Error! FNode suddenly disappeared!");
    }
  }
  debug("getCurrentNavNode exited");
  return ret;
}

FrontierInterface::PlaceMembership
PlaceManager::getPlaceMembership(double inX, double inY)
{
  debug("getPlaceMembership called");
  vector<NavData::FNodePtr> nodes;
  getMemoryEntries<NavData::FNode>(nodes, 0);

  double minDistance = FLT_MAX;
  int closestNodeID = 0;

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

  SpatialData::PlacePtr place = getPlaceFromNodeID(closestNodeID);

  FrontierInterface::PlaceMembership ret;
  ret.placeID = place->id;
  ret.confidence = 1.0;

  debug("getPlaceMembership exited");
  return ret;
}

FrontierInterface::NodeHypothesisPtr
PlaceManager::getHypFromPlaceID(int placeID)
{
  debug("getHypFromPlaceID called");
  map<int, FrontierInterface::NodeHypothesisPtr>::iterator it =
    m_PlaceIDToHypMap.find(placeID);
  if (it == m_PlaceIDToHypMap.end()) {
    debug("getHypFromPlaceID exited");
    return 0;
  }
  else {
    debug("getHypFromPlaceID exited");
    return it->second;
  }
}

NavData::FNodePtr
PlaceManager::getNodeFromPlaceID(int placeID)
{
  debug("getNodeFromPlaceID called");
  map<int, NavData::FNodePtr>::iterator it =
    m_PlaceIDToNodeMap.find(placeID);
  if (it == m_PlaceIDToNodeMap.end()) {
    debug("getNodeFromPlaceID exited");
    return 0;
  }
  else {
    debug("getNodeFromPlaceID exited");
    return it->second;
  }
}

SpatialData::PlacePtr 
PlaceManager::getPlaceFromNodeID(int nodeID)
{
  debug("getPlaceFromNodeID called");
  for(map<int, NavData::FNodePtr>::iterator it =
      m_PlaceIDToNodeMap.begin();
      it != m_PlaceIDToNodeMap.end(); it++) {
    if (it->second->nodeId == nodeID) {
      map<int, PlaceHolder>::iterator it2 = m_Places.find(it->first);
      if (it2 != m_Places.end()) {
	SpatialData::PlacePtr ret = getMemoryEntry<SpatialData::Place>(it2->second.m_WMid);
	debug("getPlaceFromNodeID exited");
	return ret;
      }
    }
  }
  debug("getPlaceFromNodeID exited");
  return 0;
}

SpatialData::PlacePtr 
PlaceManager::getPlaceFromHypID(int hypID)
{
  debug("getPlaceFromHypID called");
  for(map<int, FrontierInterface::NodeHypothesisPtr>::iterator it =
      m_PlaceIDToHypMap.begin();
      it != m_PlaceIDToHypMap.end(); it++) {
    if (it->second->hypID == hypID) { //Found a Place ID for the hypothesis
      map<int, PlaceHolder>::iterator it2 = m_Places.find(it->first);
      if (it2 != m_Places.end()) {
	SpatialData::PlacePtr ret = getMemoryEntry<SpatialData::Place>(it2->second.m_WMid);
	debug("getPlaceFromHypID exited");
	return ret;
      }
    }
  }
  debug("getPlaceFromHypID exited");
  return 0;
}

void 
PlaceManager::beginPlaceTransition(int goalPlaceID)
{
  debug("beginPlaceTransition called; goal place ID=%i", goalPlaceID);
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
      debug("beginPlaceTransition exited");
      return;
    }
  }
  NavData::FNodePtr curNode = getCurrentNavNode();
  if (curNode != 0) {
    m_startNodeForCurrentPath = curNode->nodeId;
  }
  else {
    log("Error! Could not find current Nav node!");
  }

  m_goalPlaceForCurrentPath = goalPlaceID;
  m_isPathFollowing = true;
  debug("beginPlaceTransition exited");
}

void 
PlaceManager::endPlaceTransition(int failed)
{
  
  debug("endPlaceTransition called");
  if (m_isPathFollowing) {
    log("  We were still trying to follow a path; must have failed");
    m_isPathFollowing = false;
    // If successful and the transition was unexplored, 
    // check current node.
    // If the node is different from the previous,
    // change the Place struct to non-placeholder,
    // remove the NodeHypothesis struct and change the
    // m_PlaceToNodeIDMap/m_PlaceToHypIDMap members
    processPlaceArrival(failed);
  }
  debug("endPlaceTransition exited");
}

void 
PlaceManager::processPlaceArrival(bool failed) 
{
  try {
    debug("processPlaceArrival called");
    debug("m_goalPlaceForCurrentPath was %i", m_goalPlaceForCurrentPath);

    int wasHeadingForPlace = m_goalPlaceForCurrentPath;
    int wasComingFromNode = m_startNodeForCurrentPath;
    map<int, FrontierInterface::NodeHypothesisPtr>::iterator it =
      m_PlaceIDToHypMap.find(wasHeadingForPlace);

    bool wasExploring = it != (m_PlaceIDToHypMap.end());


    NavData::FNodePtr curNode = getCurrentNavNode();
    if (curNode != 0) {
      int curNodeId = curNode->nodeId;
      log("current node id: %i", curNodeId);

      SpatialData::PlacePtr curPlace = getPlaceFromNodeID(curNodeId);
      bool placeExisted = (curPlace != 0);

      double curNodeX = curNode->x;
      double curNodeY = curNode->y;
      int curNodeGateway = curNode->gateway;

      int arrivalCase = -1;

      if (wasExploring) {
	FrontierInterface::NodeHypothesisPtr goalHyp = it->second;
	//The transition was an exploration action
	if (!placeExisted) { //No Place exists for current node -> it must be new
	  arrivalCase = 1;
	  //CASE 1: We were exploring a path, and a new node was discovered.
	  //Stop moving, upgrade the placeholder we were heading for and connect it
	  //to this new node, and delete the NodeHypotheses
	  log("  CASE 1: New node discovered while exploring");
	  map<int, PlaceHolder>::iterator it2 = m_Places.find(wasHeadingForPlace);

	  if (it2 != m_Places.end()) {
	    //Upgrade Place "wasHeadingForPlace"; delete hypothesis goalHyp; 
	    //make the Place refer to node curNode
	    upgradePlaceholder(wasHeadingForPlace, it2->second, curNode, goalHyp->hypID);

	    if (curNodeGateway == 1) {
	      addNewGatewayProperty(wasHeadingForPlace);
	    }
	  }
	  else {
	    log("Missing Placeholder placeholder!");
	    m_goalPlaceForCurrentPath = -1;
	    m_isPathFollowing = false;
	    debug("processPlaceArrival exited");
	    return;
	  }
	}

	else if (!failed && curNodeId != wasComingFromNode) {
	  arrivalCase = 2;
	  int currentPlaceID = curPlace->id;
	  //CASE 2: We were exploring, but ended up in a known Place which was not
	  //the one we started from.
	  //Remove the NodeHypothesis and its Placeholder, and
	  //send the Place merge message


	  log("  CASE 2: Exploration action failed - place already known. Deleting Place %i",
	      wasHeadingForPlace);

	  deletePlaceProperties(wasHeadingForPlace);

	  m_rejectedHypotheses[wasComingFromNode].push_back(goalHyp);
	  deleteFromWorkingMemory(m_HypIDToWMIDMap[goalHyp->hypID]); //Delete NodeHypothesis
	  m_HypIDToWMIDMap.erase(goalHyp->hypID); //Delete entry in m_HypIDToWMIDMap
	  m_PlaceIDToHypMap.erase(it); //Delete entry in m_PlaceIDToHypMap

	  //Delete Place struct and entry in m_Places
	  map<int, PlaceHolder>::iterator it2 = m_Places.find(wasHeadingForPlace);
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
	  newNotify->mergedPlaces.push_back(wasHeadingForPlace);
	  newNotify->mergedPlaces.push_back(currentPlaceID);
	  newNotify->resultingPlace = currentPlaceID;
	  log("Sending merge notification between places %i and %i", 
	      currentPlaceID, wasHeadingForPlace);

	  addToWorkingMemory<SpatialData::PlaceMergeNotification>(newDataID(), newNotify);
	  //TODO:delete notifications sometime

	}

	else {//curPlace != 0 && (failed || curNodeId == wasComingFromNode))
	  arrivalCase = 3;
	  //CASE 3: We were exploring but one way or another, we ended up
	  //were we'd started.
	  //Just delete the NodeHypothesis and its Placeholder.
	  log("  CASE 3: Exploration action failed; couldn't reach goal. Deleting place %i",
	      wasHeadingForPlace);

	  //int currentPlaceID = curPlace->id;

	  deletePlaceProperties(wasHeadingForPlace);
	  m_rejectedHypotheses[wasComingFromNode].push_back(goalHyp);
	  deleteFromWorkingMemory(m_HypIDToWMIDMap[goalHyp->hypID]); //Delete NodeHypothesis
	  m_HypIDToWMIDMap.erase(goalHyp->hypID); //Delete entry in m_HypIDToWMIDMap
	  m_PlaceIDToHypMap.erase(it); //Delete entry in m_PlaceIDToHypMap

	  //Delete Place struct and entry in m_Places
	  map<int, PlaceHolder>::iterator it2 = m_Places.find(wasHeadingForPlace);
	  if (it2 != m_Places.end()) {
	    deleteFromWorkingMemory(it2->second.m_WMid);
	    m_Places.erase(it2);
	  }
	  else {
	    log("Could not find Place to delete!");
	  }
	}

      }
      else { // (wasExploring); i.e. the goal place was not hypothetical
	SpatialData::PlacePtr curPlace = getPlaceFromNodeID(curNodeId);
	bool placeExisted = (curPlace != 0);

	if (!placeExisted) { 
	  arrivalCase = 4;
	  //CASE 4: We were *not* exploring, but a new node was discovered.
	  //We may have been going between known Places, or following a person
	  //or pushed around in Stage.
	  //Create a new Place for this node. If the node matches a hypothesis
	  //belonging to the Place we just came from, upgrade that node as in
	  //Case 1, above.
	  log("  CASE 4: Node (%d) found while not exploring", curNodeId);

	  //Check the previous Place for NodeHypotheses matching this one
	  bool foundHypothesis = 0;

	  debug("processPlaceArrival:1");
	  vector<FrontierInterface::NodeHypothesisPtr> hyps;
	  getMemoryEntries<FrontierInterface::NodeHypothesis>(hyps);
	  debug("processPlaceArrival:2");

	  if (wasComingFromNode >= 0) {
	    SpatialData::PlacePtr prevPlace = getPlaceFromNodeID(wasComingFromNode);
	    if (prevPlace != 0) {
	      NavData::FNodePtr prevNode = getNodeFromPlaceID(prevPlace->id);
	      for(vector<FrontierInterface::NodeHypothesisPtr>::iterator it2 =
		  hyps.begin(); it2 != hyps.end(); it2++) {
		FrontierInterface::NodeHypothesisPtr hyp = *it2;
		try {
		  if (prevPlace->id == hyp->originPlaceID) {
		    double distSq = (curNodeX-hyp->x)*(curNodeX-hyp->x) + 
		      (curNodeY-hyp->y)*(curNodeY-hyp->y);
		    log("  checking hypothesis %i with distance %f (%f,%f)-(%f,%f)",
			hyp->hypID, distSq, curNodeX, curNodeY, hyp->x, hyp->y);
		    if (distSq < 0.25*m_minNodeSeparation*m_minNodeSeparation) {
		      //Close enough 
		      //FIXME: should really check just bearing, not distance (because
		      //of m_hypPathLength
		      SpatialData::PlacePtr placeholder = getPlaceFromHypID(hyp->hypID);
		      if (placeholder == 0) {
			log("Could not find placeholder to upgrade");
			m_goalPlaceForCurrentPath = -1;
			m_isPathFollowing = false;
			debug("processPlaceArrival exited");
			return;
		      }

		      map<int, PlaceHolder>::iterator it3 = m_Places.find(placeholder->id);
		      if (it3 != m_Places.end()) {
			upgradePlaceholder(placeholder->id, it3->second,
			    curNode, hyp->hypID);

			//Write the Gateway property if present
			if (curNodeGateway == 1) {
			  addNewGatewayProperty(placeholder->id);
			}
		      }
		      else {
			log("Could not find Placeholder placeholder!");
			m_goalPlaceForCurrentPath = -1;
			m_isPathFollowing = false;
			debug("processPlaceArrival exited");
			return;
		      }
		      foundHypothesis = true;
		      break;
		    }
		  }
		}
		catch (IceUtil::NullHandleException) {
		  log("Error! hypothesis suddenly disappeared!");
		}
	      }
	    }
	  }

	  if (!foundHypothesis) {
	    //Found no prior hypothesis to upgrade. Create new Place instead.

	    PlaceHolder p;
	    p.m_data = new SpatialData::Place;   
	    //p.m_data->id = oobj->getData()->nodeId;

	    int newPlaceID = m_placeIDCounter;
	    m_placeIDCounter++;
	    p.m_data->id = newPlaceID;
	    m_PlaceIDToNodeMap[newPlaceID] = curNode;

	    p.m_data->status = SpatialData::TRUEPLACE;
	    p.m_WMid = newDataID();
	    log("Adding place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
	    addToWorkingMemory<SpatialData::Place>(p.m_WMid, p.m_data);

	    m_Places[newPlaceID] = p;

	    //Write the Gateway property if present
	    if (curNodeGateway == 1) {
	      addNewGatewayProperty(newPlaceID);
	    }
	  }
	}
	else {
	  arrivalCase = 5;
	  // We weren't exploring, and the place was known before - don't
	  // do anything.
	  // (Could check whether we ended up in the expected Place, but
	  // that's for the future)
	}

      }

      evaluateUnexploredPaths();
      debug("evaluateUnexploredPaths exited");

      //Once any new Placeholders have been added, it's safe to stop the robot
      //and signal the client component that we're done moving
      if (arrivalCase == 1) {
	// In Case 2, it's still quite likely that there's a new Place
	// at the location we're heading for.
	// In Case 3, the robot will already have stopped moving
	// In Case 4, we may be moving for some other reason and shouldn't stop
	// In Case 5, we don't need to stop
	cancelMovement();
      }
    }
  }
  catch(CASTException &e) {
    cout<<e.what()<<endl;
    cout<<e.message<<endl;
    abort();
  }
  m_isPathFollowing = false; //
  debug("processPlaceArrival exited");

}


FrontierInterface::NodeHypothesisPtr 
PlaceManager::PlaceServer::getHypFromPlaceID(int placeID,
    const Ice::Current &_context) {
  m_pOwner->lockComponent();
  FrontierInterface::NodeHypothesisPtr ptr(m_pOwner->getHypFromPlaceID(placeID));
  m_pOwner->unlockComponent();
  return ptr;
}

NavData::FNodePtr
PlaceManager::PlaceServer::getNodeFromPlaceID(int placeID,
    const Ice::Current &_context) {
  m_pOwner->lockComponent();
  NavData::FNodePtr ptr(m_pOwner->getNodeFromPlaceID(placeID));
  m_pOwner->unlockComponent();
  return ptr;
}

  SpatialData::PlacePtr 
PlaceManager::PlaceServer::getPlaceFromNodeID(int nodeID,
    const Ice::Current &_context)
{
  m_pOwner->lockComponent();
  SpatialData::PlacePtr ptr(m_pOwner->getPlaceFromNodeID(nodeID));
  m_pOwner->unlockComponent();
  return ptr;
}

SpatialData::PlacePtr PlaceManager::PlaceServer::getPlaceFromHypID(int hypID,
    const Ice::Current &_context)
{
  m_pOwner->lockComponent();
  SpatialData::PlacePtr ptr(m_pOwner->getPlaceFromHypID(hypID));
  m_pOwner->unlockComponent();
  return ptr;
}

void 
PlaceManager::PlaceServer::beginPlaceTransition(int goalPlaceID, const Ice::Current &_context)
{
  m_pOwner->lockComponent();
  m_pOwner->beginPlaceTransition(goalPlaceID);
  m_pOwner->unlockComponent();
}

void 
PlaceManager::PlaceServer::endPlaceTransition(int failed, const Ice::Current &_context)
{
  m_pOwner->lockComponent();
  m_pOwner->endPlaceTransition(failed);
  m_pOwner->unlockComponent();
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
	processPlaceArrival(false);
      }
    }
  }
  m_firstMovementRegistered = true;
  //log("robotMoved exited");
}

void 
PlaceManager::deletePlaceProperties(int placeID)
{
  log("deletePlaceProperties called");
  deletePlaceholderProperties(placeID);
  {
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
  }

  {
    //TODO: Delete connectivity properties
  }
  log("deletePlaceProperties exited");
}

void 
PlaceManager::deletePlaceholderProperties(int placeID)
{
  log("deletePlaceholderProperties called");
  {  //Delete free space property
    map<int, string>::iterator it = m_freeSpaceProperties.find(placeID);
    if (it != m_freeSpaceProperties.end()) {
      try {
	deleteFromWorkingMemory (it->second);
      }
      catch (Exception e) {
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
      catch (Exception e) {
	log("Border property could not be deleted; already missing!");
      }
      m_borderProperties.erase(it);
    }
  }
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

SpatialData::PlacePtr
PlaceManager::PlaceServer::getCurrentPlace(const Ice::Current &_context) {
  m_pOwner->lockComponent();
  NavData::FNodePtr curNode = m_pOwner->getCurrentNavNode();
  if(curNode !=NULL){
    SpatialData::PlacePtr curPlace = m_pOwner->getPlaceFromNodeID(curNode->nodeId);
    m_pOwner->unlockComponent();
    return curPlace;
  }else{
    m_pOwner->unlockComponent();
    return 0;

  }

FrontierInterface::PlaceMembership
PlaceManager::PlaceServer::getPlaceMembership(double x, double y,
    const Ice::Current &_context) {
  m_pOwner->lockComponent();
  FrontierInterface::PlaceMembership membership = m_pOwner->getPlaceMembership(x, y);
  m_pOwner->unlockComponent();
  return membership;
}

void
PlaceManager::upgradePlaceholder(int placeID, PlaceHolder &placeholder, NavData::FNodePtr newNode, int hypothesisID)
{
  log("  Upgrading Place %d from Placeholder status; associating with node %d", placeID, newNode->nodeId);
  string goalPlaceWMID = placeholder.m_WMid;
  try {
    debug("lock 1");
    lockEntry(goalPlaceWMID, cdl::LOCKEDODR);
    deletePlaceholderProperties(placeID);
    placeholder.m_data->status = SpatialData::TRUEPLACE;
    debug("overwrite 4: %s", goalPlaceWMID.c_str());
    overwriteWorkingMemory(goalPlaceWMID, placeholder.m_data);
    m_PlaceIDToNodeMap[placeID] = newNode;
    unlockEntry(goalPlaceWMID);
    debug("unlock 1");
  }
  catch (DoesNotExistOnWMException e) {
    log("The Place has disappeared! Re-adding it!");
    addToWorkingMemory<SpatialData::Place>(goalPlaceWMID, placeholder.m_data);
    m_PlaceIDToNodeMap[placeID] = newNode;
  }

  deleteFromWorkingMemory(m_HypIDToWMIDMap[hypothesisID]); //Delete NodeHypothesis
  m_HypIDToWMIDMap.erase(hypothesisID);
  m_PlaceIDToHypMap.erase(placeID);

  // Delete placeholder properties for the Place in question

  // 1: Free space properties
  if (m_freeSpaceProperties.find(placeID) != m_freeSpaceProperties.end()) {
    try {
      lockEntry(m_freeSpaceProperties[placeID], cdl::LOCKEDOD);
      deleteFromWorkingMemory(m_freeSpaceProperties[placeID]);
    }
    catch (DoesNotExistOnWMException) {
      log("Error! gateway property was already missing!");
    }
    m_freeSpaceProperties.erase(placeID);
  }

  // 2: Border properties
  if (m_borderProperties.find(placeID) != m_borderProperties.end()) {
    try {
      lockEntry(m_borderProperties[placeID], cdl::LOCKEDOD);
      deleteFromWorkingMemory(m_borderProperties[placeID]);
    }
    catch (DoesNotExistOnWMException) {
      log("Error! gateway property was already missing!");
    }
    m_borderProperties.erase(placeID);
  }
}

void
PlaceManager::createConnectivityProperty(double cost, int place1ID, int place2ID)
{
  set<int> &place1Connectivities = m_connectivities[place1ID];
  if (place1Connectivities.find(place2ID) == 
      place1Connectivities.end()) {
    SpatialProperties::FloatValuePtr costValue1 = 
      new SpatialProperties::FloatValue;
    SpatialProperties::FloatValuePtr costValue2 = 
      new SpatialProperties::FloatValue;
    costValue1->value = m_hypPathLength;
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
    addToWorkingMemory<SpatialProperties::ConnectivityPathProperty>(newID, connectivityProp1);

    set<int> &place1Connectivities = m_connectivities[place1ID];
    place1Connectivities.insert(place2ID); 
  }
}

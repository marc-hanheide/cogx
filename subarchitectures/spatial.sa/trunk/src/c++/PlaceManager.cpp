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
  addChangeFilter(createLocalTypeFilter<NavData::AEdge>(cdl::ADD),
      		  new MemberFunctionChangeReceiver<PlaceManager>(this,
		    			&PlaceManager::newEdge));
  addChangeFilter(createLocalTypeFilter<NavData::AEdge>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<PlaceManager>(this,
					&PlaceManager::modifiedEdge));

}

void 
PlaceManager::stop()
{
}

void 
PlaceManager::runComponent()
{
  frontierReader = FrontierInterface::FrontierReaderPrx(getIceServer<FrontierInterface::FrontierReader>("spatial.control"));
  hypothesisEvaluator = FrontierInterface::HypothesisEvaluatorPrx(getIceServer<FrontierInterface::HypothesisEvaluator>("map.manager"));

  debug("Interfaces created");
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
  //TODO: This will never work!
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

void 
PlaceManager::newEdge(const cast::cdl::WorkingMemoryChange &objID)
{
  NavData::AEdgePtr oobj =
    getMemoryEntry<NavData::AEdge>(objID.address);
  
  if (oobj != 0) {
    try {
      int newEdgeStartId = getPlaceFromNodeID(oobj->startNodeId)->id;
      int newEdgeEndId = getPlaceFromNodeID(oobj->endNodeId)->id;
      double newEdgeCost = oobj->cost;

      SpatialProperties::FloatValuePtr costValue1 = 
	new SpatialProperties::FloatValue;
      SpatialProperties::FloatValuePtr costValue2 = 
	new SpatialProperties::FloatValue;
      costValue1->value = newEdgeCost;
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
      connectivityProp1->place1Id = newEdgeStartId;
      connectivityProp1->place2Id = newEdgeEndId;
      connectivityProp1->distribution = discDistr;
      connectivityProp1->mapValue = costValue1;
      connectivityProp1->mapValueReliable = 1;
      SpatialProperties::ConnectivityPathPropertyPtr connectivityProp2 =
	new SpatialProperties::ConnectivityPathProperty;
      connectivityProp2->place1Id = newEdgeEndId;
      connectivityProp2->place2Id = newEdgeStartId;
      connectivityProp2->distribution = discDistr;
      connectivityProp2->mapValue = costValue1;
      connectivityProp2->mapValueReliable = 1;

      // Add the properties, if they're not already there
      set<int> &place1Connectivities = m_connectivities[connectivityProp1->place1Id];
      if (place1Connectivities.find(connectivityProp1->place2Id) ==
	  place1Connectivities.end()) {
	string newID = newDataID();
	addToWorkingMemory<SpatialProperties::ConnectivityPathProperty>(newID, connectivityProp1);
	place1Connectivities.insert(connectivityProp1->place2Id);
      }

      set<int> &place2Connectivities = m_connectivities[connectivityProp2->place1Id];
      if (place2Connectivities.find(connectivityProp2->place2Id) ==
	  place2Connectivities.end()) {
	string newID = newDataID();
	addToWorkingMemory<SpatialProperties::ConnectivityPathProperty>(newID, connectivityProp2);
	place2Connectivities.insert(connectivityProp2->place2Id);
      }
    }
    catch (IceUtil::NullHandleException e) {
      log("Error! edge objects disappeared from memory!");
    }
  }

}

void 
PlaceManager::modifiedEdge(const cast::cdl::WorkingMemoryChange &objID)
{
  // This will probably never be called...
  /*
  shared_ptr<CASTData<NavData::AEdge> > oobj =
    getWorkingMemoryEntry<NavData::AEdge>(objID.address);
  
  if (oobj != 0) {

    // Look for the place in the internal vector
*/
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
  FrontierInterface::FrontierPtSeq points = frontierReader->getFrontiers();

  log("Retrieved %i frontiers", points.size());

  NavData::FNodePtr curNode = getCurrentNavNode();
  if (curNode == 0) {
    log ("Could not determine current nav node!");
    return;
  }
  double nodeX = curNode->x;
  double nodeY = curNode->y;
  SpatialData::PlacePtr curPlace = getPlaceFromNodeID(curNode->nodeId);
  if (curPlace != 0) {
    int currentPlaceID = curPlace->id;

    sort(points.begin(), points.end(), FrontierPtCompare);

    vector<FrontierInterface::NodeHypothesisPtr> hypotheses;
    getMemoryEntries<FrontierInterface::NodeHypothesis>(hypotheses);

    // Find which hypotheses exist that originate in the current Place
    vector<FrontierInterface::NodeHypothesisPtr> relevantHyps;
    for (vector<FrontierInterface::NodeHypothesisPtr>::iterator it =
	hypotheses.begin(); it != hypotheses.end(); it++) {
      try {
	if ((*it)->originPlaceID == currentPlaceID) {
	  relevantHyps.push_back(*it);
	}
      }
      catch (IceUtil::NullHandleException e) {
	log("Error: hypothesis suddenly disappeared!");
      }
    }

    for (FrontierInterface::FrontierPtSeq::iterator it =
	points.begin(); it != points.end(); it++) {
      double x = (*it)->x;
      double y = (*it)->y;
      double nodeDistanceSq = (x - nodeX)*(x - nodeX) + (y - nodeY)*(y - nodeY);
      log("Evaluating frontier at (%f, %f) with square-distance %f and length %f", x, y, nodeDistanceSq, (*it)->mWidth);

      // Consider only frontiers with an open path to them
      if ((*it)->mState == FrontierInterface::FRONTIERSTATUSOPEN) {
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
		  relevantHyps.begin(); it2 != relevantHyps.end(); it2++) {
		try {
		  if ((*it2)->originPlaceID == currentPlaceID) {
		    double distanceSq = ((*it2)->x - x)*((*it2)->x - x) + ((*it2)->y - y)*((*it2)->y - y);
		    if (distanceSq < minDistanceSq) minDistanceSq = distanceSq;
		  }
		}
		catch (IceUtil::NullHandleException e) {
		  log("Error: hypothesis suddenly disappeared!");
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
		newHyp->originPlaceID = currentPlaceID;

		log("Adding new hypothesis at (%f, %f) with ID %i", newHyp->x,
		    newHyp->y, newHyp->hypID);

		string newID = newDataID();
		m_HypIDToWMIDMap[newHyp->hypID]=newID;
		addToWorkingMemory<FrontierInterface::NodeHypothesis>(newID, newHyp);
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
		set<int> &curPlaceConnectivities = m_connectivities[currentPlaceID];
		if (curPlaceConnectivities.find(newPlaceID) == 
		    curPlaceConnectivities.end())
		{
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
		  connectivityProp1->place1Id = currentPlaceID;
		  connectivityProp1->place2Id = newPlaceID;
		  connectivityProp1->distribution = discDistr;
		  connectivityProp1->mapValue = costValue1;
		  connectivityProp1->mapValueReliable = 1;

		  string newID = newDataID();
		  addToWorkingMemory<SpatialProperties::ConnectivityPathProperty>(newID, connectivityProp1);
		  curPlaceConnectivities.insert(newPlaceID); 
		}

		p.m_data->status = SpatialData::PLACEHOLDER;
		p.m_WMid = newDataID();
		log("Adding placeholder %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
		addToWorkingMemory<SpatialData::Place>(p.m_WMid, p.m_data);

		m_Places[newPlaceID]=p;
	      }
	    }
	  }
	}
      }
      else {
	log("Frontier not reachable, skipping");
      }

      for (vector<FrontierInterface::NodeHypothesisPtr>::iterator it =
	  relevantHyps.begin(); it != relevantHyps.end(); it++) {
	try {
	  int hypID = (*it)->hypID;
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
	      it2 = m_freeSpaceProperties.find(hypID);
	    if (it2 != m_freeSpaceProperties.end()) {
	      SpatialProperties::AssociatedSpacePlaceholderPropertyPtr
		freeProp = getMemoryEntry
		<SpatialProperties::AssociatedSpacePlaceholderProperty>
		(it2->second);

	      // Property exists; change it
	      freeProp->distribution = discDistr;
	      freeProp->placeId = getPlaceFromHypID(hypID);
	      freeProp->mapValue = freespacevalue;
	      freeProp->mapValueReliable = 1;
	      overwriteWorkingMemory
		<SpatialProperties::AssociatedSpacePlaceholderProperty>(it2->second,freeProp);
	    }
	    else {
	      SpatialProperties::AssociatedSpacePlaceholderPropertyPtr freeProp =
		new SpatialProperties::AssociatedSpacePlaceholderProperty;
	      freeProp->distribution = discDistr;
	      freeProp->placeId = getPlaceFromHypID(hypID);
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
	      it2 = m_borderProperties.find(hypID);
	    if (it2 != m_borderProperties.end()) {
	      SpatialProperties::AssociatedBorderPlaceholderPropertyPtr
		borderProp = getMemoryEntry
		<SpatialProperties::AssociatedBorderPlaceholderProperty>
		(it2->second);

	      // Property exists; change it
	      borderProp->distribution = discDistr;
	      borderProp->placeId = getPlaceFromHypID(hypID);
	      borderProp->mapValue = bordervalue;
	      borderProp->mapValueReliable = 1;
	      overwriteWorkingMemory
		<SpatialProperties::AssociatedBorderPlaceholderProperty>(it2->second,borderProp);
	    }
	    else {
	      SpatialProperties::AssociatedBorderPlaceholderPropertyPtr borderProp =
		new SpatialProperties::AssociatedBorderPlaceholderProperty;
	      borderProp->distribution = discDistr;
	      borderProp->placeId = getPlaceFromHypID(hypID);
	      borderProp->mapValue = bordervalue;
	      borderProp->mapValueReliable = 1;

	      string newID = newDataID();
	      addToWorkingMemory<SpatialProperties::AssociatedBorderPlaceholderProperty>
		(newID, borderProp);
	      m_borderProperties[hypID] = newID;
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
  if (curNode != 0) {
    m_startNodeForCurrentPath = curNode->nodeId;
  }
  else {
    log("Error! Could not find current Nav node!");
  }

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
  if (curNode != 0) {
    int curNodeId = curNode->nodeId;
    log("current node id: %i", curNodeId);
    double curNodeX = curNode->x;
    double curNodeY = curNode->y;

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
	  deletePlaceholderProperties(m_goalPlaceForCurrentPath);
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
	int currentPlaceID = curPlace->id;
	//CASE 2: We were exploring, but ended up in a known Place which was not
	//the one we started from.
	//Remove the NodeHypothesis and its Placeholder, and
	//send the Place merge message

	log("  CASE 2: Exploration action failed - place already known. Deleting Place %i",
	    m_goalPlaceForCurrentPath);

	deletePlaceProperties(m_goalPlaceForCurrentPath);

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
	newNotify->mergedPlaces.push_back(currentPlaceID);
	newNotify->resultingPlace = currentPlaceID;
	log("Sending merge notification between places %i and %i", 
	    currentPlaceID, m_goalPlaceForCurrentPath);

	addToWorkingMemory<SpatialData::PlaceMergeNotification>(newDataID(), newNotify);
	//TODO:delete notifications sometime
      }

      else {//curPlace != 0 && (failed || curNodeId == m_startNodeForCurrentPath))
	//CASE 3: We were exploring but one way or another, we ended up
	//were we'd started.
	//Just delete the NodeHypothesis and its Placeholder.
	log("  CASE 3: Exploration action failed; couldn't reach goal. Deleting place %i",
	    m_goalPlaceForCurrentPath);

	deletePlaceProperties(m_goalPlaceForCurrentPath);
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

	  p.m_data->id = m_placeIDCounter;
	  m_PlaceIDToNodeMap[m_placeIDCounter] = newNavNode;

	  p.m_data->status = SpatialData::TRUEPLACE;
	  p.m_WMid = newDataID();
	  log("Adding place %ld, with tag %s", p.m_data->id, p.m_WMid.c_str());
	  addToWorkingMemory<SpatialData::Place>(p.m_WMid, p.m_data);

	  m_Places[m_placeIDCounter] = p;
	  m_placeIDCounter++;

	  //Write the Gateway property if present
	  if (newNavNode->gateway == 1) {
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
	    gwProp->placeId = p.m_data->id;
	    gwProp->mapValue = trueValue;
	    gwProp->mapValueReliable = 1;

	    string newID = newDataID();
	    addToWorkingMemory<SpatialProperties::GatewayPlaceProperty>(newID, gwProp);
	    m_gatewayProperties[gwProp->placeId] = newID;
	  }
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
    NavData::FNodePtr curNode = getCurrentNavNode();
    if (curNode != 0) {
      m_startNodeForCurrentPath = curNode->nodeId;
    }
  }
}

void 
PlaceManager::deletePlaceProperties(int placeID)
{
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
}

void 
PlaceManager::deletePlaceholderProperties(int placeID)
{
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
}

//
// = Filename
//   PathQueryProcessor.cpp
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

#include "PathQueryProcessor.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <Navigation/NavGraphNode.hh>
#include <FrontierInterface.hpp>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new PathQueryProcessor();
  }
}

PathQueryProcessor::PathQueryProcessor()
{
  m_GotGraph = false;
  m_noIndirectPaths = false;
}

PathQueryProcessor::~PathQueryProcessor()
{
}

void 
PathQueryProcessor::start()
{
  addChangeFilter(createLocalTypeFilter<NavData::NavGraph>(cdl::ADD),
		  new MemberFunctionChangeReceiver<PathQueryProcessor>(this,
					&PathQueryProcessor::newNavGraph));
  
  addChangeFilter(createLocalTypeFilter<NavData::NavGraph>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<PathQueryProcessor>(this,
					&PathQueryProcessor::newNavGraph));    
  
  addChangeFilter(createLocalTypeFilter<SpatialData::PathTransitionProbRequest>
      (cdl::ADD),
      new MemberFunctionChangeReceiver<PathQueryProcessor>(this,
	&PathQueryProcessor::newPathTransitionProbRequest));

  addChangeFilter(createLocalTypeFilter<SpatialData::PathTransitionCostRequest>
      (cdl::ADD),
      new MemberFunctionChangeReceiver<PathQueryProcessor>(this,
	&PathQueryProcessor::newPathTransitionCostRequest));

  m_placeInterface = FrontierInterface::PlaceInterfacePrx(getIceServer<FrontierInterface::PlaceInterface>("place.manager"));

}

void 
PathQueryProcessor::stop()
{

}

void 
PathQueryProcessor::configure(const std::map<std::string, std::string>& _config)
{
  log("Configure entered");

  if(_config.find("--no-indirect-paths") != _config.end()) {
    m_noIndirectPaths = true;
  }
}

void 
PathQueryProcessor::runComponent()
{
}

void 
PathQueryProcessor::newNavGraph(const cast::cdl::WorkingMemoryChange &objID)
{
  m_GraphMutex.lock();
	
  m_NavGraph.clear();
  shared_ptr<CASTData<NavData::NavGraph> > oobj =
    getWorkingMemoryEntry<NavData::NavGraph>(objID.address);
  
  if (oobj != 0) {
    
    NavData::NavGraphPtr ng = oobj->getData();
        
    for (unsigned int i = 0; i < ng->fNodes.size(); i++) {
      
      double x = ng->fNodes[i]->x;
      double y = ng->fNodes[i]->y;
      double theta = ng->fNodes[i]->theta;
      int areaId = (int) ng->fNodes[i]->areaId;
      string areaType = "";
      if (!ng->fNodes[i]->type.empty()) {
        areaType = ng->fNodes[i]->type[0].name;
      }
      int nodeId = (int) ng->fNodes[i]->nodeId;

      const double maxSpeed = 2;
      
      if (ng->fNodes[i]->gateway){ // add gateway node
               
	double width = 1;
        if (!ng->fNodes[i]->width.empty()) {
          width = ng->fNodes[i]->width[0];
        }
	m_NavGraph.addDoorToNodeList(nodeId, areaId, x, y, theta, 
                                     width, areaType, maxSpeed);
      }

      else { // add ordinary node

	m_NavGraph.addNodeToNodeList(nodeId, areaId, x, y, theta, 
                                      areaType, maxSpeed);

      }
      
    }
    
    for (unsigned int i=0;i<ng->aEdges.size();i++){ // add edges
      
      int n1 = (int) ng->aEdges[i]->startNodeId;
      int n2 = (int) ng->aEdges[i]->endNodeId;
      
      m_NavGraph.addEdgeToEdgeList(n1, n2);
      
    }
    
    m_NavGraph.connectNodes();

    m_GotGraph = true;

    debug("Got a new graph with %d doors and a total of %d nodes", 
          m_NavGraph.m_Gateways.size(), m_NavGraph.m_Nodes.size());    
  }

  m_GraphMutex.unlock();
}

void 
PathQueryProcessor::newPathTransitionProbRequest(const cast::cdl::WorkingMemoryChange &objID)
{
  debug("newPathTransitionProbRequest called");
  CASTData<SpatialData::PathTransitionProbRequest> oobj =
    getMemoryEntryWithData<SpatialData::PathTransitionProbRequest>(objID.address);

  SpatialData::PathTransitionProbRequestPtr ptr = oobj.getData();

  // FIXME: Must check the src so that we do not react to our own change

  if (ptr->status != SpatialData::QUERYPENDING) {
    // For us to process this request we require that it is marked
    // as pending.
    log("Got a PathTransitionProbRequest already processed");
    return;
  }

  m_GraphMutex.lock();

  debug("1");


  debug("2");
  //Firstly, check if start place is a placeholder
  SpatialData::NodeHypothesisPtr startHyp =
    m_placeInterface->getHypFromPlaceID(ptr->startPlaceID);
  if (startHyp != 0) {
    debug("Requested path probability from Placeholder with Place ID %i",
	ptr->startPlaceID);
    //Return 0 probability for paths starting at placeholders
    SpatialData::PlaceProbability pp;
    pp.placeID = ptr->startPlaceID;
    pp.prob = 1;
    ptr->successProb = 0;
    ptr->successors.push_back(pp);

    // If we have asked for 
    if (ptr->noSuccessors < 0) {
      //Compute probabilities for all nav nodes
      for (std::list<Cure::NavGraphNode*>::iterator i = 
	  m_NavGraph.m_Nodes.begin(); i != m_NavGraph.m_Nodes.end(); i++) {
	unsigned int nodeID = (*i)->getId();
	SpatialData::PlacePtr place = m_placeInterface->getPlaceFromNodeID(nodeID);
	if (place != 0) {
	  pp.placeID = place->id;
	  pp.prob = 0;
	  ptr->successors.push_back(pp);
	}
      }
      // Compute probabilities for all placeholders
      std::vector<SpatialData::NodeHypothesisPtr> hypPts;
      getMemoryEntries<SpatialData::NodeHypothesis>(hypPts,0);
      for (std::vector<SpatialData::NodeHypothesisPtr>::iterator i = 
	  hypPts.begin(); i != hypPts.end(); i++) {
	if ((*i)->hypID == startHyp->hypID) {
	  // Already added
	}
	else{
	  unsigned int hypID = (*i)->hypID;
	  SpatialData::PlacePtr place = m_placeInterface->getPlaceFromHypID(hypID);
	  if (place != 0) {
	    pp.placeID = place->id;
	    pp.prob = 0;
	    ptr->successors.push_back(pp);
	  }
	}
      }
      NavData::FNodePtr startNode =
	m_placeInterface->getNodeFromPlaceID(ptr->startPlaceID);
      if (startNode == 0) {
	log("Could not find starting Place!");
	ptr->status = SpatialData::QUERYPLACE1INVALID;
      }
    }
  }

  else {
    debug("Start Place was a node");
    //Start Place is a node
    NavData::FNodePtr startNode =
      m_placeInterface->getNodeFromPlaceID(ptr->startPlaceID);
  debug("3");
    if (startNode == 0) {
      log("Could not find start Node!");
      ptr->status = SpatialData::QUERYPLACE1INVALID;
    }
    else {
      //Start place is an extant node
      //Secondly, check if goal place is a placeholder
  debug("4");
      SpatialData::NodeHypothesisPtr goalHyp =
	m_placeInterface->getHypFromPlaceID(ptr->goalPlaceID);
      if (goalHyp != 0) {
	//Goal place is a placeholder
	int goalHypID = goalHyp->hypID;

	SpatialData::PlaceProbability pp;

	// We use a super simple model in which it is prob 1 to reach
	// the goal (if reachable and 0 for the rest)
	bool reachable = goalHyp->originPlaceID == ptr->startPlaceID;
  debug("5");
	if (reachable) {
	  pp.placeID = ptr->goalPlaceID;
	  pp.prob = 1;
	  ptr->successProb = 1;
	}
	else {
	  pp.placeID = ptr->startPlaceID;
	  pp.prob = 1;
	  ptr->successProb = 0;
	}
	ptr->successors.push_back(pp);

  debug("6");
	// If we have asked for 
	if (ptr->noSuccessors < 0) {
  debug("7");

	  //Compute probabilities for all nav nodes
	  for (std::list<Cure::NavGraphNode*>::iterator i = 
	      m_NavGraph.m_Nodes.begin(); i != m_NavGraph.m_Nodes.end(); i++) {
	    if ((*i)->getId() == startNode->nodeId) {
	      // Already added
	    } else {
	      unsigned int nodeID = (*i)->getId();
	      SpatialData::PlacePtr place = m_placeInterface->getPlaceFromNodeID(nodeID);
	      if (place != 0) {
		pp.placeID = place->id;
		pp.prob = 0;
		ptr->successors.push_back(pp);
	      }
	    }
	  }
	  // Compute probabilities for all placeholders
	  std::vector<SpatialData::NodeHypothesisPtr> hypPts;
	  getMemoryEntries<SpatialData::NodeHypothesis>(hypPts,0);
	  for (std::vector<SpatialData::NodeHypothesisPtr>::iterator i = 
	      hypPts.begin(); i != hypPts.end(); i++) {
	    if ((*i)->hypID == goalHypID) {
	      // Already added
	    }
	    else{
	      unsigned int hypID = (*i)->hypID;
	      SpatialData::PlacePtr place = m_placeInterface->getPlaceFromHypID(hypID);
	      if (place != 0) {
		pp.placeID = place->id;
		pp.prob = 0;
		ptr->successors.push_back(pp);
	      }
	    }
	  }
	}

	ptr->status = SpatialData::QUERYCOMPLETED;
      }
      else {
	//Goal place is not a placeholder
	NavData::FNodePtr goalNode =
	  m_placeInterface->getNodeFromPlaceID(ptr->goalPlaceID);
	if (goalNode == 0) {
	  log("Could not find goal Place!");
	  ptr->status = SpatialData::QUERYPLACE2INVALID;
	}
	else {
	  //Goal place is a nav node

	  int startNodeID = startNode->nodeId;
	  int goalNodeID = goalNode->nodeId;
	  std::list<Cure::NavGraphNode> path;
	  double cost;
	  debug("findPath(%ld, %ld,...)", startNodeID, goalNodeID);
	  int ret = m_NavGraph.findPath(startNodeID, goalNodeID,
	      path, &cost);

	  SpatialData::PlaceProbability pp;

	  // We use a super simple model in which it is prob 1 to reach
	  // the goal (if reachable and 0 for the rest
	  if (m_noIndirectPaths) {
	    // Disallow paths that do not follow a NavGraph edge
	    // between two places directly
	    if (ret == 0 && path.size() < 3) {
	      pp.placeID = ptr->goalPlaceID;
	      pp.prob = 1;
	      ptr->successProb = 1;
	    }
	    else {
	      pp.placeID = ptr->startPlaceID;
	      pp.prob = 1;
	      ptr->successProb = 0;
	    }
	  }
	  else { //m_noIndirectPaths
	    if (ret == 0) {

	      pp.placeID = ptr->goalPlaceID;
	      pp.prob = 1;
	      ptr->successProb = 1;

	    } else {

	      pp.placeID = ptr->startPlaceID;
	      pp.prob = 1;
	      ptr->successProb = 0;
	    }

	  }

	  ptr->successors.push_back(pp);

	  // If we have asked for 
	  if (ptr->noSuccessors < 0) {

	    // Compute probabilities for all nav nodes
	    for (std::list<Cure::NavGraphNode*>::iterator i = 
		m_NavGraph.m_Nodes.begin(); i != m_NavGraph.m_Nodes.end(); i++) {

	      if ((*i)->getId() == startNodeID && ret) {
		// Already added
	      } else if ((*i)->getId() == goalNodeID && !ret) {
		// Already added
	      } else {
		unsigned int nodeID = (*i)->getId();
		SpatialData::PlacePtr place = m_placeInterface->getPlaceFromNodeID(nodeID);
		if (place != 0) {
		  pp.placeID = place->id;
		  pp.prob = 0;
		  ptr->successors.push_back(pp);
		}
	      }

	    }
	    // Compute probabilities for all placeholders
	    std::vector<SpatialData::NodeHypothesisPtr> hypPts;
	    getMemoryEntries<SpatialData::NodeHypothesis>(hypPts,0);
	    for (std::vector<SpatialData::NodeHypothesisPtr>::iterator i = 
		hypPts.begin(); i != hypPts.end(); i++) {
	      unsigned int hypID = (*i)->hypID;
	      SpatialData::PlacePtr place = m_placeInterface->getPlaceFromHypID(hypID);
	      if (place != 0) {
		pp.placeID = place->id;
		pp.prob = 0;
		ptr->successors.push_back(pp);
	      }
	    }
	  }
	}
	ptr->status = SpatialData::QUERYCOMPLETED;
      }
    }
  }
  debug("8");
  if (ptr->status == SpatialData::QUERYPENDING)
    ptr->status = SpatialData::QUERYCOMPLETED;
  overwriteWorkingMemory<SpatialData::PathTransitionProbRequest>(objID.address, ptr);

  m_GraphMutex.unlock();
}

void 
PathQueryProcessor::newPathTransitionCostRequest(const cast::cdl::WorkingMemoryChange &objID)
{
  CASTData<SpatialData::PathTransitionCostRequest> oobj =
    getMemoryEntryWithData<SpatialData::PathTransitionCostRequest>(objID.address);
  debug("entered newPathTransitionCostRequest");
  //if (oobj != 0) {

  SpatialData::PathTransitionCostRequestPtr ptr = oobj.getData();

  // FIXME: Must check the src so that we do not react to our own change

  if (ptr->status != SpatialData::QUERYPENDING) {
    // For us to process this request we require that it is marked
    // as pending.
    log("Got a PathTransitionCostRequest already processed");
    return;
  }

  m_GraphMutex.lock();
  debug("have lock");

  //Firstly, check if start place is a placeholder
  SpatialData::NodeHypothesisPtr startHyp =
    m_placeInterface->getHypFromPlaceID(ptr->startPlaceID);
  if (startHyp != 0) {
    debug("Requested path cost from Placeholder with Place ID %i",
	ptr->startPlaceID);
    ptr->cost = 1e100;
    ptr->status = SpatialData::QUERYCOMPLETED;
  }
  else {
    NavData::FNodePtr startNode =
      m_placeInterface->getNodeFromPlaceID(ptr->startPlaceID);
    if (startNode == 0) {
      log("Could not find starting Place!");
      ptr->status = SpatialData::QUERYPLACE1INVALID;
    }
    else { //startNode != 0
      //Start place is an extant node
      //Secondly, check if goal place is a placeholder
      NavData::FNodePtr goalNode;
      SpatialData::NodeHypothesisPtr goalHyp =
	m_placeInterface->getHypFromPlaceID(ptr->goalPlaceID);
      if (goalHyp != 0) {
	//Goal place is a placeholder

	//For unexplored edges, simply model cost as metric distance
	//between the node and the frontier
	bool directlyReachable = goalHyp->originPlaceID == ptr->startPlaceID;
	if (directlyReachable) {
	  double distSq = (goalHyp->x - startNode->x)*(goalHyp->x - startNode->x) +
	    (goalHyp->y - startNode->y)*(goalHyp->y - startNode->y);
	  ptr->cost = sqrt(distSq);
	  log("found direct connection... can break here");
	  ptr->status = SpatialData::QUERYCOMPLETED;
	}
	else {
	  goalNode =
	    m_placeInterface->getNodeFromPlaceID(goalHyp->originPlaceID);
	}
//	else
//	{
//	  log("frontier not reachable: %ld != %ld", goalHyp->originPlaceID, ptr->startPlaceID);
//	  ptr->cost = 1e100;
//	}
//	ptr->status = SpatialData::QUERYCOMPLETED;
      }
      else { //goalHyp == 0
	goalNode =
	  m_placeInterface->getNodeFromPlaceID(ptr->goalPlaceID);
      } //goalHyp != 0
      // if we are not yet done
      if (ptr->status != SpatialData::QUERYCOMPLETED) {
      if (goalNode == 0) {
	log("Could not find goal Place!");
	ptr->status = SpatialData::QUERYPLACE2INVALID;
      }
      else { //goalNode != 0
	//Goal place is an extant node
	int startNodeID = startNode->nodeId;
	int goalNodeID = goalNode->nodeId;

	std::list<Cure::NavGraphNode> path;
	double cost = 1e100;
	debug("findPath(%ld, %ld,....)",startNodeID, goalNodeID);
	int ret = m_NavGraph.findPath(startNodeID, goalNodeID,
	      path, &cost);

	if (m_noIndirectPaths) {
	  if (ret == 0 && path.size() < 3) {
	    ptr->cost = cost;
	  }
	  else {
	    ptr->cost = 1e100;
	  }
	}
	else { //m_noIndirectPaths
	  if (ret == 0) {
	    ptr->cost = cost;
	  }
	  else {
	    ptr->cost = 1e100;
	  }
	}
	ptr->status = SpatialData::QUERYCOMPLETED;
      } //goalHyp == 0
      } //ptr->status != SpatialData::QUERYCOMPLETED
    } //startNode != 0
  } //startHyp == 0
  debug("write PathTransitionCostRequest");
  overwriteWorkingMemory<SpatialData::PathTransitionCostRequest>(objID.address, ptr);
  m_GraphMutex.unlock();
  //}
}

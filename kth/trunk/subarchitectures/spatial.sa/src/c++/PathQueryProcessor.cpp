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
  
  //if (oobj != 0) {
    
    SpatialData::PathTransitionProbRequestPtr ptr = oobj.getData();

    // FIXME: Must check the src so that we do not react to our own change

    if (ptr->status != SpatialData::QUERYPENDING) {
      // For us to process this request we require that it is marked
      // as pending.
      log("Got a PathTransitionProbRequest already processed");
      return;
    }

    m_GraphMutex.lock();

    if (!m_NavGraph.getNode(ptr->startPlaceID)) {

      log("Start place invalid");
      ptr->status = SpatialData::QUERYPLACE1INVALID;

    } else if (!m_NavGraph.getNode(ptr->goalPlaceID)) {

      log("Goal place invalid");
      ptr->status = SpatialData::QUERYPLACE2INVALID;

    } else {

      std::list<Cure::NavGraphNode> path;
      double cost;
      debug("findPath(%ld, %ld,...)", ptr->startPlaceID, ptr->goalPlaceID);
      int ret = m_NavGraph.findPath(ptr->startPlaceID, ptr->goalPlaceID,
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

        for (std::list<Cure::NavGraphNode*>::iterator i = 
               m_NavGraph.m_Nodes.begin(); i != m_NavGraph.m_Nodes.end(); i++) {

          if ((*i)->getId() == ptr->startPlaceID && ret) {
            // Already added
          } else if ((*i)->getId() == ptr->goalPlaceID && !ret) {
            // Already added
          } else {
            pp.placeID = (*i)->getId();
            pp.prob = 0;
            ptr->successors.push_back(pp);
          }
          
        }

      }

      ptr->status = SpatialData::QUERYCOMPLETED;

    }
    overwriteWorkingMemory<SpatialData::PathTransitionProbRequest>(objID.address, ptr);

    m_GraphMutex.unlock();
  //}
}

void 
PathQueryProcessor::newPathTransitionCostRequest(const cast::cdl::WorkingMemoryChange &objID)
{
  CASTData<SpatialData::PathTransitionCostRequest> oobj =
    getMemoryEntryWithData<SpatialData::PathTransitionCostRequest>(objID.address);
  
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

    if (!m_NavGraph.getNode(ptr->startPlaceID)) {

      log("Start place invalid");
      ptr->status = SpatialData::QUERYPLACE1INVALID;

    } else if (!m_NavGraph.getNode(ptr->goalPlaceID)) {

      log("Goal place invalid");
      ptr->status = SpatialData::QUERYPLACE2INVALID;

    } else {

      std::list<Cure::NavGraphNode> path;
      double cost = 1e100;
      debug("findPath(%ld, %ld,....)",ptr->startPlaceID, ptr->goalPlaceID);
      int ret = m_NavGraph.findPath(ptr->startPlaceID, ptr->goalPlaceID,
                                    path, &cost);

      
      ptr->cost = cost;
      ptr->status = SpatialData::QUERYCOMPLETED;

    }

    overwriteWorkingMemory<SpatialData::PathTransitionCostRequest>(objID.address, ptr);
    m_GraphMutex.unlock();
  //}
}

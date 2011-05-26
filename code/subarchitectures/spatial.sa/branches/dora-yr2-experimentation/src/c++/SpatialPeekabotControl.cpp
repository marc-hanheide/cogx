//
// = FILENAME
//    SpatialPeekabotControl.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "SpatialPeekabotControl.hpp"

#include <AddressBank/ConfigFileReader.hh>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <FrontierInterface.hpp>
#include <float.h>

using namespace cast;
using namespace std;
using namespace spatial;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new SpatialPeekabotControl();
  }
}

SpatialPeekabotControl::SpatialPeekabotControl() : m_maxPlaces(0), m_pendingQueryReceiver(this), gadget_y(10), gadget_ystep(2), gadgetLineLength(20), m_maxPlaceholderID(0), m_hideGadget(false)
{
  m_CtrlAction = 0;
  m_doPathQuery = false;
}

SpatialPeekabotControl::~SpatialPeekabotControl() 
{}

void SpatialPeekabotControl::configure(const map<string,string>& config) 
{
  println("configure entered");

  m_RetryDelay = 10;
  if(config.find("--retry-interval") != config.end()){
    std::istringstream str(config.find("--retry-interval")->second);
    str >> m_RetryDelay;
  }

  if(config.find("--do-path-query") != config.end()) {
    m_doPathQuery = true;
  }

  Cure::ConfigFileReader *cfg = 0;

  map<string,string>::const_iterator it;

  it = config.find("-c");
  if (it != config.end()) {
    cfg = new Cure::ConfigFileReader;
    log("About to try to open the config file");
    if (cfg->init(it->second) != 0) {
      delete cfg;
      cfg = 0;
      log("Could not init Cure::ConfigFileReader with -c argument");
    } else {
      log("Managed to open the Cure config file");
    }
  }

  it = config.find("--action");
  if (it != config.end()) {
    std::string action(it->second);
    if (action == "object") {
      m_CtrlAction = 1;
    }
  }

  it = config.find("--hide-gadget");
  if (it != config.end()) {
    m_hideGadget = true;
  }

  m_PbPort = 5050;
  m_PbHost = "localhost";

  if (cfg) {

    // To be backward compatible with config files that specify the
    // RoboLook host and really mean peekabot we read that first and
    // overwrite it below if both are specified
    cfg->getRoboLookHost(m_PbHost);

    std::string usedCfgFile, tmp;
    if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
      m_PbHost = tmp;
    }

  }

  connectPeekabot();  

  println("configure done");
}

void SpatialPeekabotControl::start() 
{

  //Only for Year1 visualization!
  addChangeFilter(cast::createLocalTypeFilter<FrontierInterface::NodeHypothesis>(cast::cdl::ADD),
		  new cast::MemberFunctionChangeReceiver<SpatialPeekabotControl>(this,
										 &SpatialPeekabotControl::newNodeHypothesis));
  addChangeFilter(cast::createLocalTypeFilter<FrontierInterface::NodeHypothesis>(cast::cdl::DELETE),
		  new cast::MemberFunctionChangeReceiver<SpatialPeekabotControl>(this,
										 &SpatialPeekabotControl::deletedNodeHypothesis));

  println("start entered");
  
  m_placeInterface = getIceServer<FrontierInterface::PlaceInterface>("place.manager");
}

void SpatialPeekabotControl::runComponent() {

  println("runComponent");

  while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
    sleep(m_RetryDelay);
    connectPeekabot();
  }

  println("Connected to peekabot, ready to go");


  peekabot::GroupProxy root;
  //root.assign(m_PeekabotClient, "root");
  m_placeholderModule.add(m_PeekabotClient, "placeholder", peekabot::REPLACE_ON_CONFLICT);

}

void 
SpatialPeekabotControl::PendingQueryReceiver::workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc) 
{
  if (m_dependentCommand != 0) {
    m_parent->log("Received path transition probability query result");
    //Check if the query returned OK status for us
    SpatialData::PathTransitionProbRequestPtr query = 
      m_parent->getMemoryEntry<SpatialData::PathTransitionProbRequest>(_wmc.address);
    if (query && query->status == SpatialData::QUERYCOMPLETED) {
      if (query->successProb > 0.0) {
	m_parent->log("Probability > 0, dispatch pending nav command");
	string id = "gotoplace-" + m_parent->newDataID();
	m_parent->log("Sending robot to place %ld, task id: %s", 
		      m_dependentCommand->destId[0], id.c_str());

	m_parent->addToWorkingMemory<SpatialData::NavCommand>(id, m_dependentCommand);
      }
      else {
	m_parent->log("Probability 0; do not send nav command");
      }
    }
    m_dependentCommand = 0;
	  
    m_parent->deleteFromWorkingMemory(_wmc.address);
    m_parent->removeChangeFilter(this);
  }
}

void SpatialPeekabotControl::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort);

  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}

void
SpatialPeekabotControl::newPlace(const cast::cdl::WorkingMemoryChange &_wmc)
{
  //  cast::CASTData<SpatialData::Place> data = getMemoryEntryWithData<SpatialData::Place>(_wmc.address);
  //  int id = (int)data.getData()->id;
  //  m_Places.insert(id);
  //
  updatePeekabotGadget();
}

void
SpatialPeekabotControl::deletedPlace(const cast::cdl::WorkingMemoryChange &_wmc)
{
  updatePeekabotGadget();
}

bool placeOrder(SpatialData::PlacePtr a, SpatialData::PlacePtr b) {
  return a->id < b->id;
}

void SpatialPeekabotControl::updatePeekabotGadget()
{
  vector<SpatialData::PlacePtr> places;
  getMemoryEntries<SpatialData::Place>(places);
  sort(places.begin(), places.end(), placeOrder);

  int nPlaces = places.size();
  if (nPlaces == 0)
    return;
  int highestPlaceID = places[nPlaces-1]->id;
  m_maxPlaces = m_maxPlaces > highestPlaceID ? m_maxPlaces : highestPlaceID;

  log("nPlaces: %i maxPlaces: %i", nPlaces, m_maxPlaces);

  //peekabot::PolygonProxy pp;
  //pp.add(m_controlmodule, "placeidline", peekabot::REPLACE_ON_CONFLICT);
  //pp.add_vertex(0,gadget_y-1,0);
  //double maxX = 0 + 1.0*(nPlaces-1);
  //pp.add_vertex(maxX,gadget_y-1,0);
  //pp.add_vertex(maxX,gadget_y-1+0.05,0);
  //pp.add_vertex(0,gadget_y-1+0.05,0);
  //pp.set_color (0,0,0);

  peekabot::LabelProxy lp;
  lp.add(m_controlmodule, "placelabel", peekabot::REPLACE_ON_CONFLICT);
  lp.set_text("place id: ");
  lp.set_pose(0,gadget_y-0.5,0,0,0,0);
  lp.set_scale(50);
  lp.set_alignment(peekabot::ALIGN_RIGHT);

  int i = 0;
  for (vector<SpatialData::PlacePtr>::iterator it = places.begin(); it != places.end(); it++) {
    char identifier[100];
    for(;i < (*it)->id;i++) {
      sprintf(identifier, "label%d", i);
      lp.add(m_controlmodule, identifier, peekabot::REPLACE_ON_CONFLICT);
      lp.hide();
      sprintf(identifier, "circle%d", i);
      peekabot::CircleProxy cp;
      cp.add(m_controlmodule, identifier, peekabot::REPLACE_ON_CONFLICT);
      cp.hide();
    }

    peekabot::LabelProxy lp;
    sprintf(identifier, "label%d", i);
    lp.add(m_controlmodule, identifier, peekabot::REPLACE_ON_CONFLICT);
    sprintf(identifier, "%d", (int)(*it)->id);
    lp.set_text(identifier);
    lp.set_pose(1.0*(i % gadgetLineLength),gadget_y-0.5 - (gadget_ystep * (i / gadgetLineLength)),0,0,0,0);
    if ((*it)->status == SpatialData::PLACEHOLDER) {
      lp.set_scale(30);
    }
    else {
      lp.set_scale(50);
    }
    lp.set_alignment(peekabot::ALIGN_CENTER);

    NavData::FNodePtr node = m_placeInterface->getNodeFromPlaceID((*it)->id);
    if (node != 0) {
      peekabot::LabelProxy lp2;
      sprintf(identifier, "nodelabel%d", i);
      lp2.add(m_controlmodule, identifier, peekabot::REPLACE_ON_CONFLICT);
      sprintf(identifier, "%d", (int)(*it)->id);
      lp2.set_text(identifier);
      lp2.set_pose(node->x,node->y - 0.5,0,0,0,0);
      lp2.set_scale(30);
      lp2.set_alignment(peekabot::ALIGN_CENTER);
    }

    sprintf(identifier, "circle%d", i);
    peekabot::CircleProxy cp;
    cp.add(m_controlmodule, identifier, peekabot::REPLACE_ON_CONFLICT);
    cp.set_scale(0.04);
    cp.set_pose(1.0*(i % gadgetLineLength),gadget_y-1 - (gadget_ystep * (i / gadgetLineLength)),0,0,0,0);
    i++;
  }
  for (;i < m_maxPlaces; i++) {
    char identifier[100];
    sprintf(identifier, "label%d", i);
    lp.add(m_controlmodule, identifier, peekabot::REPLACE_ON_CONFLICT);
    lp.hide();
    sprintf(identifier, "circle%d", i);
    peekabot::CircleProxy cp;
    cp.add(m_controlmodule, identifier, peekabot::REPLACE_ON_CONFLICT);
    cp.hide();
  }
}

NavData::FNodePtr
SpatialPeekabotControl::getCurrentNavNode()
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

//Year1 visualization only!
void 
SpatialPeekabotControl::newNodeHypothesis(const cast::cdl::WorkingMemoryChange &_wmc)
{
  //  try {
  //    FrontierInterface::NodeHypothesisPtr newHypothesis = getMemoryEntry<FrontierInterface::NodeHypothesis>(_wmc.address);
  //
  //    m_nodeHypotheses[newHypothesis->hypID]=newHypothesis;
  //    
  //    m_maxPlaceholderID = (m_maxPlaceholderID > newHypothesis->hypID ? 
  //	m_maxPlaceholderID : newHypothesis->hypID);
  //    updatePlaceholderVisualization();
  //  }
  //  catch (cast::DoesNotExistOnWMException) {
  //
  //  }
  updatePlaceholderVisualization();
}

void
SpatialPeekabotControl::deletedNodeHypothesis(const cast::cdl::WorkingMemoryChange &_wmc)
{
  updatePlaceholderVisualization();
}

void
SpatialPeekabotControl::updatePlaceholderVisualization()
{
  vector<FrontierInterface::NodeHypothesisPtr> hypotheses; 
  getMemoryEntries<FrontierInterface::NodeHypothesis>(hypotheses);

  vector<FrontierInterface::NodeHypothesisPtr>::iterator it;
  for (it = hypotheses.begin(); it != hypotheses.end(); it++) {
    int hypID = (*it)->hypID;
    m_maxPlaceholderID = m_maxPlaceholderID > hypID ? m_maxPlaceholderID : hypID;
  }
  for (int i = 0; i <= m_maxPlaceholderID; i++) {
    //map<int, FrontierInterface::NodeHypothesisPtr>::iterator it =
    //m_nodeHypotheses.find(i);
    bool drawn = false;
    SpatialData::PlacePtr place = m_placeInterface->getPlaceFromHypID(i);
    if (place != 0) {
      int placeID = place->id;

      for (it = hypotheses.begin(); it != hypotheses.end(); it++) {
	if ((*it)->hypID == i)
	  break;
      }

      if (it != hypotheses.end()) {
	peekabot::CircleProxy cp;
	char buffer[256];
	sprintf(buffer, "plchldr%i", i);
	cp.add(m_placeholderModule, buffer, peekabot::REPLACE_ON_CONFLICT);
	cp.set_scale(0.1);
	cp.set_position((*it)->x, (*it)->y,0);

	peekabot::LabelProxy lp;
	sprintf(buffer, "plchldr_l%i", i);
	lp.add(m_placeholderModule, buffer, peekabot::REPLACE_ON_CONFLICT);
	lp.set_scale(30);
	sprintf(buffer, "%i(%i)", placeID, (*it)->originPlaceID);
	lp.set_text(buffer);
	lp.set_position((*it)->x, (*it)->y,0);

	drawn = true;
      }
    }

    if (!drawn) {
      peekabot::CircleProxy cp;
      char buffer[256];
      sprintf(buffer, "plchldr%i", i);
      cp.add(m_placeholderModule, buffer, peekabot::REPLACE_ON_CONFLICT);
      cp.hide();

      peekabot::LabelProxy lp;
      sprintf(buffer, "plchldr_l%i", i);
      lp.add(m_placeholderModule, buffer, peekabot::REPLACE_ON_CONFLICT);
      lp.hide();
    }
  }
}


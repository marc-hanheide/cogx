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

  m_RetryDelay = 1000;
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
  if(!m_hideGadget) {
    addChangeFilter(cast::createLocalTypeFilter<SpatialData::Place>(cast::cdl::ADD),
		    new cast::MemberFunctionChangeReceiver<SpatialPeekabotControl>(this,
										   &SpatialPeekabotControl::newPlace));    
    
    addChangeFilter(cast::createLocalTypeFilter<SpatialData::Place>(cast::cdl::DELETE),
		    new cast::MemberFunctionChangeReceiver<SpatialPeekabotControl>(this,
										   &SpatialPeekabotControl::deletedPlace));    
  }

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
    sleepComponent(m_RetryDelay);
    connectPeekabot();
  }

  println("Connected to peekabot, ready to go");


  peekabot::GroupProxy root;
  root.assign(m_PeekabotClient, "root");
  m_placeholderModule.add(root, "placeholder", peekabot::REPLACE_ON_CONFLICT);


  if(!m_hideGadget) {
    
    bool sentplancommand = false;
    double radius = 0.75;
    double xNoA = 10, yNoA = 10;
    double searchx = 13, searchy = 13;
    
    
    m_controlmodule.add(root, "ctrl", peekabot::REPLACE_ON_CONFLICT);

    // Create an "icon" to drag around and show where you want the robot
    // to go
    debug("Adding target widget");
    peekabot::CylinderProxy target;
    target.add(m_controlmodule, "target", peekabot::REPLACE_ON_CONFLICT);
    target.set_scale(0.5*radius,0.5*radius,0.1);
    target.set_position(xNoA - 2*radius,yNoA,0.05);
    target.set_color(0.5,0.5,0.5);
    
    // Create an "icon" to mark if you want to control according to the marker
    debug("Adding master control widget");
    peekabot::SphereProxy control;
    control.add(m_controlmodule, "control", peekabot::REPLACE_ON_CONFLICT);
    control.set_scale(0.5*radius,0.5*radius,0.1);
    control.set_position(xNoA,yNoA+2*radius,0.05);
    control.set_color(0,1,0);
    
    debug("Adding control zone widget");
    peekabot::CylinderProxy actionZone;
    actionZone.add(m_controlmodule, "action-zone", 
		   peekabot::REPLACE_ON_CONFLICT);
    actionZone.set_scale(radius,radius,0);
    actionZone.set_position(xNoA,yNoA,0);
    actionZone.set_color(0,1,0);
    
    peekabot::CylinderProxy searchhere;
    searchhere.add(m_controlmodule, "search-here",
                   peekabot::REPLACE_ON_CONFLICT);
    searchhere.set_scale(radius,radius,0);
    searchhere.set_position(searchx,searchy,0);
    searchhere.set_color(0,1,0);

    peekabot::CylinderProxy startspot;
    startspot.add(m_controlmodule, "start-spot ",
                  peekabot::REPLACE_ON_CONFLICT);
    startspot.set_scale(0.2,0.2,0.3);
    startspot.set_position(searchx,searchy,0);
    startspot.set_color(1,0,0);

    
    updatePeekabotGadget();
    
    bool wasInCtrl = false;
    float xT, yT;
    short lsize = 10;
    std::vector<int> dirlist(lsize,0);
    double oldposx = searchx;
    double oldposy = searchy;
    int i = 0;
    
    
    debug("Entering main loop");
    if (m_PeekabotClient.is_connected()) {
      
      while (isRunning()) {
	double dir;
	peekabot::Result<peekabot::Vector3ru> r;
	
	r = searchhere.get_position(peekabot::WORLD_COORDINATES);
	if (r.succeeded()) {
	  dir = atan2( (r.get_result().m_c[1] - oldposy), (r.get_result().m_c[0] - oldposx))*180/M_PI;
	  if (dir < 0)
	    dir += 360;
	  //log("dir of search: %.2f",dir);
	  
	  oldposx = r.get_result().m_c[0];
	  oldposy = r.get_result().m_c[1];
	  if (dir >= 0 and dir <= 90)
	    dirlist[i % lsize] = 0;
	  else if (dir >= 90 and dir <= 180)
	    dirlist[i % lsize] = 1;
	  else if (dir >= 180 and dir <= 270)
	    dirlist[i % lsize] = 2;
	  else if (dir >= 270 and dir <= 360)
	    dirlist[i % lsize] = 3;
	  /*                for (unsigned int j = 0; j < dirlist.size(); j++)
			    printf("%i",dirlist[j]);
			    printf("\n");*/

	  i++;
	  // if dirlist contains all 1 2 3 directions we have a wobble!
	  bool has1 = false;
	  bool has2 = false;
	  bool has3 = false;

	  for (unsigned int j = 0; j < dirlist.size(); j++) {
	    if (dirlist[j] == 1)
	      has1 = true;
	    if (dirlist[j] == 2)
	      has2 = true;
	    if (dirlist[j] == 3)
	      has3 = true;
	  }
	  // if we have a wobble check if we are in a free node and add that to search list

	  if (has1 and has2 and has3) {
	    NavData::FNodeSequence fnodeseq;
	    //                    log("WOOBLEE!!!");
	    std::vector< boost::shared_ptr<CASTData<NavData::FNode> > > obj;
	    while (obj.empty()) {
	      getWorkingMemoryEntries<NavData::FNode>(20, obj);
	      usleep(1000);
	    }
	    for (unsigned int i = 0; i < obj.size() ; i++) {
	      fnodeseq.push_back(obj[i]->getData());
	    }


	    for (unsigned int h = 0; h < fnodeseq.size(); h++) {

	      if (hypot(fnodeseq[h]->y - r.get_result().m_c[1],
			fnodeseq[h]->x - r.get_result().m_c[0]) < radius) {
		// seems we are asked to search this node
		//log("fnodeid %i",fnodeseq[h]->nodeId);
		SpatialData::PlacePtr place = m_placeInterface->getPlaceFromNodeID(fnodeseq[h]->nodeId);
		//check if we already added this 
		bool isadded = false;
		for (unsigned int l = 0; l < placeseq.size(); l++){
		  if (placeseq[l] == place->id)
		    isadded = true;
		}
		if (!isadded){
		  placeseq.push_back(place->id);
		  log("placeid : %lli (fnodeid: %lli) added to search plan!",place->id,fnodeseq[h]->nodeId);
		}
	      }
	    }

	  }

	  //check if we are back to start zone
	  r = searchhere.get_position(peekabot::WORLD_COORDINATES);

	  if ( r.succeeded() && hypot(searchy - r.get_result().m_c[1],
				      searchx - r.get_result().m_c[0]) < radius && !placeseq.empty()
	       && !sentplancommand) {

	    SpatialData::AVSCommandPtr avscmd = new SpatialData::AVSCommand;
	    avscmd->cmd = SpatialData::PLAN;
	    avscmd->placestosearch = placeseq;
	    addToWorkingMemory(newDataID(), avscmd);
	    sentplancommand = true;
	  }
	}


	// Check if the control marker is inside or outside the control
	// zone. If it is 
	r = actionZone.get_position(peekabot::WORLD_COORDINATES);

	if (r.succeeded()) {
	  xNoA = r.get_result().m_c[0];
	  yNoA = r.get_result().m_c[1];
	}

	// Get the position of the control marker
	debug("Checking master control position");
	r = control.get_position(peekabot::WORLD_COORDINATES);
	debug("Checked master control position");

	if (r.succeeded()) {

	  // Check if the control marker is inside the action zone
	  if (hypot(gadget_y - r.get_result().m_c[1],
		    xNoA - r.get_result().m_c[0]) < radius) {

	    // We should listen to the control marker, since it is
	    // inside the action zone

	    // Get position of the target marker
	    debug("Checking target position");
	    r = target.get_position(peekabot::WORLD_COORDINATES);

	    if (r.succeeded()) {

	      bool sendCmd = true;

	      // Check if we already were in control
	      if (wasInCtrl &&               
		  (xT == r.get_result().m_c[0]) &&
		  (yT == r.get_result().m_c[1])) {
		sendCmd = false;
	      }

	      xT = r.get_result().m_c[0];
	      yT = r.get_result().m_c[1];

	      if (sendCmd) {
		debug("sendCmd");

		SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand;

		cmd->prio = SpatialData::NORMAL;

		// If we are close enough to the line with place ids
		// we interpret the command as a place index

		std::string id = "";
		if (yT - (gadget_y-1) < 0.5 &&
		    yT - (gadget_y-1) > -(0.5 + gadget_ystep * (m_maxPlaces / gadgetLineLength))) { //fabs(yT-(gadget_y-1)) < 0.5) {

		  // Get the place id
		  cmd->destId.resize(1);
		  int ypos = (int)(((gadget_y-1+0.5)-yT)/gadget_ystep);
		  cmd->destId[0] = long(xT + 0.5) + ypos * gadgetLineLength;
		  log("Reading command: %i, %i, %i", ypos, long(xT + 0.5), (int)cmd->destId[0]);
		  if (cmd->destId[0] < 0) cmd->destId[0] = 0;

		  cmd->cmd = SpatialData::GOTOPLACE;
		  cmd->status = SpatialData::NONE;
		  cmd->comp = SpatialData::COMMANDPENDING;

		  if (m_doPathQuery) {
		    // Check that the path has some chance of completing, first
		    SpatialData::PathTransitionProbRequestPtr probRequest =
		      new SpatialData::PathTransitionProbRequest;

		    NavData::FNodePtr curNode = getCurrentNavNode();
		    if (curNode == 0) {
		      log("Could not compute current nav node!");
		      probRequest->startPlaceID = 0;
		    }
		    else {
		      SpatialData::PlacePtr curPlace =
			m_placeInterface->getPlaceFromNodeID(curNode->nodeId);
		      if (curPlace == 0) {
			log("Could not compute current Place!");
			probRequest->startPlaceID = 0;
		      }
		      else {
			probRequest->startPlaceID = curPlace->id;
		      }
		    }

		    probRequest->goalPlaceID = cmd->destId[0];
		    probRequest->noSuccessors = 1;
		    probRequest->status = SpatialData::QUERYPENDING;

		    string queryID = "probreq-" + newDataID();
		    log("Sending query: %s", queryID.c_str());
		    m_pendingQueryReceiver.setDependentCommand(cmd);
		    addChangeFilter(cast::createIDFilter(queryID, cast::cdl::OVERWRITE), &m_pendingQueryReceiver);
		    addToWorkingMemory<SpatialData::PathTransitionProbRequest>(queryID, probRequest);


		  }
		  else {
		    id = "gotoplace-" + newDataID();

		    log("Sending robot to place %ld, task id: %s", 
			cmd->destId[0], id.c_str());

		    addToWorkingMemory<SpatialData::NavCommand>(id, cmd);
		  } 
		}
		else {

		  cmd->pose.resize(2);
		  cmd->pose[0] = xT;
		  cmd->pose[1] = yT;
		  cmd->cmd = SpatialData::GOTOPOSITION;
		  cmd->status = SpatialData::NONE;
		  cmd->comp = SpatialData::COMMANDPENDING;
		  id = "gotoxy-" + newDataID();
		  log("Sending robot to new target position (%.2f, %.2f) task id: %s", xT, yT, id.c_str());

		  addToWorkingMemory<SpatialData::NavCommand>(id, cmd);
		}

	      }

	    }

	    wasInCtrl = true;

	  } else {

	    // No longer in control
	    wasInCtrl = false;

	  }
	}


	// Sleep for a second and check again
	sleepComponent(100);

      }
    }
  }
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

    m_PeekabotClient.connect(m_PbHost, m_PbPort, true);

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
	lp.set_scale(20);
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


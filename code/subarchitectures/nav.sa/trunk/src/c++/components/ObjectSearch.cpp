#include "ObjectSearch.hpp"
#include <NavData.hpp>
#include <VisionData.hpp>
#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <AddressBank/ConfigFileReader.hh>
#include <Navigation/GridContainer.hh>
#include "ObjPdf.hpp"
#include "XVector3D.h"
#include <FrontierInterface.hpp>
#include <float.h>

using namespace cast;
using namespace std;
using namespace boost;
//using namespace Cure;
extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new ObjectSearch();
  }
}

ObjectSearch::ObjectSearch() {}

ObjectSearch::~ObjectSearch() {
  
  
}
void ObjectSearch::start() {
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ObjectSearch>(this,
								 &ObjectSearch::newRobotPose));
  
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<ObjectSearch>(this,
								 &ObjectSearch::newRobotPose));
  
  
  addChangeFilter(createChangeFilter<VisionData::VisualObject>
		  (cdl::ADD,
		   "",
		   "",
		   "vision.sa",
		   cdl::ALLSA),
		  new MemberFunctionChangeReceiver<ObjectSearch>(this,
								 &ObjectSearch::ObjectDetected));
  
  addChangeFilter(createChangeFilter<VisionData::VisualObject>
		  (cdl::OVERWRITE,
		   "",
		   "",
		   "vision.sa",
		   cdl::ALLSA),
		  new MemberFunctionChangeReceiver<ObjectSearch>(this,
								 &ObjectSearch::ObjectDetected));
  
  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::ADD),
		  new MemberFunctionChangeReceiver<ObjectSearch>(this,
								 &ObjectSearch::newNavGraphNode));  
  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<ObjectSearch>(this,
								 &ObjectSearch::newNavGraphNode)); 
  
  
  addChangeFilter(createLocalTypeFilter<SpatialData::AVSCommand>(cdl::ADD),
                  new MemberFunctionChangeReceiver<ObjectSearch>(this,
								 &ObjectSearch::newAVSCommand));  
  
  addChangeFilter(createLocalTypeFilter<SpatialData::AVSCommand>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<ObjectSearch>(this,
								 &ObjectSearch::newAVSCommand)); 
}

void ObjectSearch::stopAVS() {
    log("stopping AVS");
    whereinplan = -1;
    m_command = STOP;
    m_status = STOPPED;
    whereinplan = m_plan.plan.size();
    m_Displaykrsjlgm = 0;
    m_Displaycoverage = 0;
    m_samples = new int[2*m_samplesize];
    
    try {
      deleteFromWorkingMemory(m_lastCmdAddr);
    }	    
    catch(const CASTException &e) {
      println("failed to delete AVSCommand: %s", e.message.c_str());
    }

}

void ObjectSearch::newAVSCommand(const cdl::WorkingMemoryChange &objID){
  
  shared_ptr<CASTData<SpatialData::AVSCommand> > oobj =
    getWorkingMemoryEntry<SpatialData::AVSCommand>(objID.address);
  
  //store for later possible deletion
  m_lastCmdAddr = objID.address;
  
  if (oobj->getData()->cmd == SpatialData::PLAN){    
    log("will plan.");

    placestosearch = oobj->getData()->placestosearch;
    m_command = PLAN;
  }
  else if (oobj->getData()->cmd == SpatialData::STOPAVS) {
    stopAVS();
  }
}

void ObjectSearch::newNavGraphNode(const cdl::WorkingMemoryChange &objID)
{
  debug("new NavGraphNode");
  
  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  NavData::FNodePtr fnode = oobj->getData();
  fnodeseq.push_back(fnode);  
}

void ObjectSearch::configure(const map<string,string>& _config) {
  log("Configuring ObjectSearch");
  map<string,string>::const_iterator it = _config.find("-c");
  
  if (it== _config.end()) {
    log("configure(...) Need config file (use -c option)\n");
    std::abort();
  }
  std::string configfile = it->second;
  Cure::ConfigFileReader cfg;
  if (cfg.init(configfile)) {
    log("configure(...) Failed to open with %s\n",
	configfile.c_str());
    std::abort();
  }
  //Exploration range
  m_MaxExplorationRange = 1;
  it = _config.find("--explore-range");
  if (it != _config.end()) {
    m_MaxExplorationRange = (atof(it->second.c_str()));
  }
  
  m_fov = M_PI/4;
  it = _config.find("--cam-fov");
  if (it != _config.end()) {
    m_fov = (atof(it->second.c_str()))*M_PI/180;
  }
  
  

  m_ptustep = M_PI/6;
  it = _config.find("--cam-step");
  if (it != _config.end()) {
    m_ptustep = (atof(it->second.c_str()))*M_PI/180;
  }
  
  m_tiltRads = 0.0;
  it = _config.find("--tilt");
  if (it != _config.end()) {
    m_tiltRads = -(atof(it->second.c_str()))*M_PI/180;
  }
  log("Tilt pose set to: %f",m_tiltRads);
  
  
  //Coverage percent treshold
  m_covthresh = 70.0;
  it = _config.find("--coverage-threshold");
  if (it != _config.end()) {
    m_covthresh = (atof(it->second.c_str()));
  }
  log("Coverage threshold set to: %f",m_covthresh);
  
  
  m_vpthreshold = 10.0;
  it = _config.find("--vp-threshold");
  if (it != _config.end()) {
    m_vpthreshold = (atof(it->second.c_str()));
  }
  log("Viewpoint threshold set to: %f",m_vpthreshold);
  
  m_CamRange = 1;
  it = _config.find("--cam-range");
  if (it != _config.end()) {
    m_CamRange = (atof(it->second.c_str()));
  }
  log("Camera range set to: %f",m_CamRange);

  
  m_awayfromobstacles = 1.0;
  it = _config.find("--away-from-obstacles");
  if (it != _config.end()) {
        m_awayfromobstacles = (atof(it->second.c_str()));
  }
  log("Away from obstacle set to: %f",m_awayfromobstacles);
  
  
  
  //Laser pose
  if (cfg.getSensorPose(1, m_LaserPoseR)) {
    log("configure(...) Failed to get sensor pose");
    std::abort();
  }
  
  
  m_gridsize = 400;
  m_cellsize = 0.1;
  it = _config.find("--gridsize");
  if (it != _config.end()) {
    
    m_gridsize = (atof(it->second.c_str()));
        log("Gridsize set to: %f",m_CamRange);
  }
  
  
  it = _config.find("--cellsize");
  if (it != _config.end()) {
    m_cellsize = (atof(it->second.c_str()));
    log("Cellize set to: %f",m_cellsize);
  }
  
  int magnification = 1;
  it = _config.find("--disp-magn");
  if (it != _config.end()) {
    magnification = (atoi(it->second.c_str()));
    log("Display magnification set to: %f", magnification);
  }
  
  m_CtrlPTU = (_config.find("--ctrl-ptu") != _config.end());
  
  m_samplesize = 100;
  it = _config.find("--samplesize");
  if (it != _config.end()) {
    m_samplesize = (atof(it->second.c_str()));
    log("Samplesize set to: %d",m_samplesize);
  }
  m_samples = new int[2*m_samplesize];
  m_samplestheta = new double[m_samplesize];
  /*    if (_config.find("--no-x-window") == _config.end()) {
	
	m_Displaylgm = new Cure::X11DispLocalGridMap<double>(*m_lgm,magnification);
        log("Will use X window to show the exploration map");
	} else {
        m_Displaylgm = 0;
        log("Will NOT use X window to show the exploration map");
	}
	
	if (_config.find("--display-coverage") != _config.end()) {
	m_Displaycoverage = new Cure::X11DispLocalGridMap<unsigned int>(*coveragemap,magnification);
	} else {
        m_Displaycoverage = 0;
	}*/
  m_Displaykrsjlgm = 0;
  m_Displaycoverage = 0;
  displayOn = true;
  //Objects
  if((it = _config.find("--objects")) != _config.end()) {
    istringstream istr(it->second);
    string label;
    Object* obj;
    
    while(istr >> label) {
            obj = new Object();
            obj->ObjID = label;
            m_objectlist.push_back(obj);
    }
  }
  log("Loaded objects.");
  
  cmp = NavData::SUCCEEDED;
  
  m_coveragetotal = -1;
  m_covered = 0;
  m_status = STOPPED;
  whereinplan = -1;
  
  if (m_CtrlPTU)  {
    log("connecting to PTU");
    Ice::CommunicatorPtr ic = getCommunicator();
    
    Ice::Identity id;
    id.name = "PTZServer";
    id.category = "PTZServer";
    
    std::ostringstream str;
    str << ic->identityToString(id) 
	<< ":default"
	<< " -h localhost"
	<< " -p " << cast::cdl::CPPSERVERPORT;
    
    Ice::ObjectPrx base = ic->stringToProxy(str.str());    
    m_PTUServer = ptz::PTZInterfacePrx::uncheckedCast(base);
  }
  
  log("Configured ObjectSearch");
}

void ObjectSearch::runComponent() {
  log("ObjectSearch running ");
  
  
  lockComponent();
  MovePanTilt(0, 0);
  unlockComponent();
  
  //clock_t start_time,elapsed;
  //double elapsed_time;
  m_command = IDLE; //TURN;
  log("hey I run");
  while(isRunning()) {	
    lockComponent();
    InterpretCommand ();
    if (m_Displaykrsjlgm != 0){
      m_Displaykrsjlgm->updateDisplay(0,0,0,m_samplesize, m_samples,tpoints,ViewConePts,
				      1,m_plan.plan,m_plan.indexarray);
      m_Displaycoverage->updateCoverageDisplay();
    }
    unlockComponent();
    sleepComponent(1000);
  }
  
  
}
void ObjectSearch::MovePanTilt(double pan, double tilt, double tolerance){
  if (m_CtrlPTU)
    {
      log(" Moving pantilt to: %f %f with %f tolerance", pan, tilt, tolerance);
      ptz::PTZPose p;
      ptz::PTZReading ptuPose;
      p.pan = pan;
      p.tilt = tilt;
      p.zoom = 0;
      m_PTUServer->setPose(p);
      bool run = true;
      ptuPose = m_PTUServer->getPose();
      double actualpan = ptuPose.pose.pan;
      double actualtilt = ptuPose.pose.tilt;

      while(run){
	//m_PTUServer->setPose(p);
	ptuPose = m_PTUServer->getPose();
	actualpan = ptuPose.pose.pan;
	actualtilt = ptuPose.pose.tilt;

	log("desired pan tilt is: %f %f", pan, tilt);
	log("actual pan tilt is: %f %f", actualpan, actualtilt);
	log("tolerance is: %f", tolerance);
	

	//check that pan falls in tolerance range
	if(actualpan < (pan + tolerance) && 
	   actualpan > (pan - tolerance)) {
	  run = false;
	}

	//only check tilt if pan is ok
	if(!run) {
	  if(actualtilt < (tilt + tolerance) && 
	     actualtilt > (tilt - tolerance)) {
	    run = false;
	  }
	  else {
	    //if the previous check fails, loop again
	    run = true;
	  }
	}
	
	usleep(10000);
      }
      log("Moved.");
      sleep(1);
    }
}
NavData::ObjectSearchPlanPtr ObjectSearch::ConvertPlantoIce()
{ 
	NavData::ObjectSearchPlanPtr obs = new NavData::ObjectSearchPlan;
	cogx::Math::Vector3 a;
	for (unsigned int i = 0; i < m_plan.plan.size(); i++){
		a.x = m_plan.plan[i].getX(); a.y = m_plan.plan[i].getY(); a.z = m_plan.plan[i].getTheta();
		obs->planlist.push_back(a);
	}
	return obs;
}
void ObjectSearch::Plan () {
	
    GenViewPoints();
    m_plan = GeneratePlan(m_covthresh, ScorebyCoverage(*fcm));
    addToWorkingMemory(newDataID(), ConvertPlantoIce());
    log("Plan generated %i view points with %f coverage",m_plan.plan.size(),m_plan.totalcoverage);
    m_command = EXECUTE;
}
void ObjectSearch::InterpretCommand () {
  switch(m_command) {
  case STOP: {
    m_command = IDLE;
    m_status = STOPPED;
    log("Command: STOP");
    SpatialData::NavCommandPtr cmd = newNavCommand();
    cmd->prio = SpatialData::URGENT;
    cmd->cmd = SpatialData::STOP;
    new NavCommandReceiver(*this, cmd);
    break;
  }
  case TURN: {
    log("Command: TURN");
    m_command = IDLE;
    SpatialData::NavCommandPtr cmd = newNavCommand();
    cmd->prio = SpatialData::URGENT;
    cmd->cmd = SpatialData::TURN;
    cmd->angle.resize(1);
    cmd->angle[0] = M_PI;
    new NavCommandReceiver(*this, cmd);
    break;
  }
  case PLAN:
    log("Command: PLAN");
    m_command = IDLE;
    m_status = PLANNING;
    Plan();
    break;
  case EXECUTE:
    log("Command: EXECUTE");
    m_command = IDLE;
    m_status = EXECUTINGPLAN;
    ExecutePlan();
    break;
  case RECOGNIZE:
    log("Command: RECOGNIZE");
    m_command = IDLE;
    m_status = RECOGNITIONINPROGRESS;
    Recognize();
  case IDLE:
    if(m_status == RECOGNITIONCOMPLETE ) {
	m_command = EXECUTE;
	log("Recognition complete. Execute next in plan.");
      }
    //        log("Command: IDLE");
    break;
  default:
    log("Command: Default.");
    break;
  }
}
void ObjectSearch::ExecuteNextInPlan () {
  whereinplan++;
  log("Plan size %i, where in plan: %i.",m_plan.plan.size(),whereinplan);
  if (whereinplan >= (int)m_plan.plan.size()){
    log("Plan finished.");
    try {
      whereinplan = -1;
      deleteFromWorkingMemory(m_lastCmdAddr);
    }	    
    catch(const CASTException &e) {
      println("failed to delete AVSCommand: %s", e.message.c_str());
    }
    m_command = IDLE;
    m_status = STOPPED;
    return;
  }
  if (m_plan.plan.size() == 0) {
    log("Nothing to execute.");
    try {
      whereinplan = -1;
      deleteFromWorkingMemory(m_lastCmdAddr);
    }	    
    catch(const CASTException &e) {
      println("failed to delete AVSCommand: %s", e.message.c_str());
    }
    m_command = IDLE;
    m_status = STOPPED;
    return;
  }
  if (m_status != NAVCOMMANDINPROGRESS) {
    log("Posting NavCommand");
    PostNavCommand(m_plan.plan[whereinplan]);
    m_status = NAVCOMMANDINPROGRESS;
    
  } else if (m_status == NAVCOMMANDINPROGRESS) {
    log("NavCommand in progress.");
  }
}




ObjectSearch::NavCommandReceiver::NavCommandReceiver(ObjectSearch & _component, SpatialData::NavCommandPtr _cmd) :
  m_component(_component), m_cmd(_cmd) {
  m_component.log("received NavCommandReceiver notification");
  string id(m_component.newDataID());
  m_component.log("ID post: %s",id.c_str());

  m_component.addChangeFilter(createIDFilter(id,cdl::OVERWRITE),this);  
  m_component.addToWorkingMemory<SpatialData::NavCommand>(id, m_cmd);  

}

void ObjectSearch::NavCommandReceiver::workingMemoryChanged(const cast::cdl::WorkingMemoryChange &_wmc) {
  m_component.log("received inner notification");
    try { 
	  m_component.owtNavCommand(_wmc);

  SpatialData::NavCommandPtr cmd(m_component.getMemoryEntry<SpatialData::NavCommand>(_wmc.address));

  if(cmd->comp == SpatialData::COMMANDSUCCEEDED) {
    m_component.log("receiver cleaning up on success");
    //delete nav cmd
    //m_component.deleteFromWorkingMemory(_wmc.address);
    //get CAST to delete self
    //m_component.removeChangeFilter(this, cdl::DELETERECEIVER);

  }
    }	    
    catch(const CASTException &e) {
//      log("failed to delete SpatialDataCommand: %s", e.message.c_str());
    }
  
}


SpatialData::NavCommandPtr ObjectSearch::newNavCommand() {
  SpatialData::NavCommandPtr cmd = new SpatialData::NavCommand();
  cmd->prio = SpatialData::NORMAL;
  cmd->cmd = SpatialData::STOP;
  cmd->status = SpatialData::NONE;
  cmd->comp = SpatialData::COMMANDPENDING;
  return cmd;
}

void ObjectSearch::PostNavCommand(Cure::Pose3D position) {
    SpatialData::NavCommandPtr cmd = newNavCommand();
    cmd->prio = SpatialData::URGENT;
    cmd->cmd = SpatialData::GOTOPOSITION;
    cmd->pose.resize(3);
    cmd->pose[0] = position.getX();
    cmd->pose[1] = position.getY();
    cmd->pose[2] = position.getTheta();
    cmd->tolerance.resize(1);
    cmd->tolerance[0] = 0.1;
    cmd->status = SpatialData::NONE;
    cmd->comp = SpatialData::COMMANDPENDING;

    new NavCommandReceiver(*this, cmd);
}

void ObjectSearch::ExecutePlan () {
		ExecuteNextInPlan ();
}


void ObjectSearch::IcetoCureLGM(FrontierInterface::LocalGridMap icemap){
	log("icemap.size: %d, icemap.data.size %d, icemap.cellSize: %f, centerx,centery: %f,%f",icemap.size, icemap.data.size(), icemap.cellSize, icemap.xCenter, icemap.yCenter);
	m_krsjlgm = new Cure::LocalGridMap<unsigned char>(icemap.size, icemap.cellSize, '2', Cure::LocalGridMap<char>::MAP1, icemap.xCenter,icemap.yCenter);
	int lp = 0;
	for(int x = -icemap.size ; x <= icemap.size; x++){
		for(int y = -icemap.size ; y <= icemap.size; y++){ 
			(*m_krsjlgm)(x,y) = (icemap.data[lp]);
			lp++;
		}
	}
    log("converted icemap to krsjmap");	
}
void ObjectSearch::GenViewPoints() {
  m_coveragetotal = -1;
  whereinplan = -1;
  srand ( time(NULL) );
  log("Generating %i random samples", m_samplesize);
  ViewConePts.clear();
  candidatePoses.clear();
  int randx,randy;
  double xW,yW;
  int i=0;
  std::vector<double> angles;
  log("creating placeinterface proxy");
  FrontierInterface::LocalGridMap combined_lgm;
  FrontierInterface::PlaceInterfacePrx agg(getIceServer<FrontierInterface::PlaceInterface>("place.manager"));
  log("getting combined lgm");
  FrontierInterface::LocalMapInterfacePrx agg2(getIceServer<FrontierInterface::LocalMapInterface>("map.manager"));
  for (unsigned int g = 0; g < placestosearch.size(); g++)
    log("%d",placestosearch[g]);
  combined_lgm = agg2->getCombinedGridMap(placestosearch);
  log("have combined lgm");
  IcetoCureLGM(combined_lgm);
  
  // set total coverage
  m_coveragetotal = -1;
  coveragemap = new Cure::LocalGridMap<unsigned char>(*m_krsjlgm);          
  fcm = new Cure::LocalGridMap<unsigned char>(*coveragemap);
  for(int x = -combined_lgm.size ; x <= combined_lgm.size; x++){
    for(int y = -combined_lgm.size ; y <= combined_lgm.size; y++){ 
      if ((*m_krsjlgm)(x,y) == '1') {
	m_coveragetotal++;
      }
    }
  }
  log("created placeinterface proxy");
  
  Cure::Pose3D currPose = m_currPose;
  /*Checking if a point in x,y is reachable */
  
  /// Binary grid that is 0 for foree space and 1 for occupied
  Cure::BinaryMatrix m_NonFreeSpace;
  
  
  // We make the number of columns of the BinaryMatrix a multiple
  // of 32 so that we get the benefit of the representation.
  // Here m_LGMap is assumed to be the LocalGridMap
  int rows = 2 * combined_lgm.size + 1;
  int cols = ((2 * combined_lgm.size + 1) / 32 + 1) * 32;
  m_NonFreeSpace.reallocate(rows, cols);
  m_NonFreeSpace = 0; // Set all cells to zero
  
  // First we create a binary matrix where all cells that
  // corresponds to known obstacles are set to "1".
  for (int x = -combined_lgm.size; x <= combined_lgm.size; x++) {
    for (int y = -combined_lgm.size; y <= combined_lgm.size; y++) {
      if ((*m_krsjlgm)(x,y) == '1') { // FIXME: =='1' need to be changed
	m_NonFreeSpace.setBit(x + combined_lgm.size,
			      y + combined_lgm.size,
			      true);
      }
    }
  }
  
  
  // Create an istance of BinaryMatrx which will hold the result of
  // expanding the obstacles
  Cure::BinaryMatrix m_PathGrid;
  m_PathGrid = 0;
  
  // Grow each occupied cell to account for the size of the
  // robot. We put the result in another binary matrix, m_PathGrid
  m_NonFreeSpace.growInto(m_PathGrid,
			  0.5*0.45/combined_lgm.cellSize, // 0.45 is robot width hard coded here.
			  true);
  
  // We treat all unknown cells as occupied so that the robot only
  // uses paths that it knowns to be free. Note that we perfom this
  // operation directly on the m_PathGrid, i.e. the grid with the
  // expanded obstacle. The reasoning behind this is that we do not
  // want the unknown cells to be expanded as well as we would have
  // to recalculate the position of the frontiers otherwise, else
  // they might end up inside an obstacle (could happen now as well
  // from expanding the occupied cell but then it is known not to be
  // reachable).
  for (int x = -combined_lgm.size; x <= combined_lgm.size; x++) {
    for (int y = -combined_lgm.size; y <= combined_lgm.size; y++) {
      if ((*m_krsjlgm)(x,y) == '2') {
	m_PathGrid.setBit(x + combined_lgm.size, 
			  y + combined_lgm.size, 
			  true);
      }
    }
  }
  
  
  /*Checking if a point in x,y is reachable */
  
  
  for (double rad= 0 ; rad < M_PI*2 ; rad = rad + M_PI/3) {
    angles.push_back(rad);
  }
  log("pushed angles");

  while (i < m_samplesize) {
    

//    for (double rad= 0 ; rad < M_PI*2 ; rad = rad + M_PI/3) {
//        angles.push_back(rad);
//    }

    //      log("processing sample %i/%i", i+1, m_samplesize);
    
    randx = rand();
    randy = rand();
    randx = (randx % (2*combined_lgm.size)) - combined_lgm.size;
    randy = (randy % (2*combined_lgm.size)) - combined_lgm.size;
    m_krsjlgm->index2WorldCoords(randx,randy,xW,yW);
    if ((*m_krsjlgm)(randx,randy) == '0') {
      if (m_krsjlgm->isRectangleObstacleFree(xW,yW-0.2, xW,yW+0.2,1)){
	long nodeid = GetClosestFNode(xW,yW);
	SpatialData::PlacePtr place = agg->getPlaceFromNodeID(nodeid);
	long id = -1;
	if (place != NULL)
	  id = place->id;
	bool isincluded = false;
	for (unsigned int q= 0; q < placestosearch.size(); q++){
	  if (placestosearch[q] == id){
	    isincluded = true;
	    break;
	  }
	}
	if (isincluded) { //if sample is in a place we were asked to search
	  /*if reachable*/
	  // Get the indices of the destination coordinates
	  int rS, cS, rE, cE;
	  if (m_krsjlgm->worldCoords2Index(currPose.getX(), currPose.getY(), rS, cS) == 0 &&	      m_krsjlgm->worldCoords2Index(xW, yW, rE, cE) == 0) {
	    // Compensate for the fact that the PathGrid is just a normal matrix where the cells are numbers from the corner
	    cS += combined_lgm.size;
	    rS += combined_lgm.size;
	    cE += combined_lgm.size;
	    
rE += combined_lgm.size;
	    
	    Cure::ShortMatrix path;
	    double d = (m_PathGrid.path(rS, cS, rE, cE, path,
					20 * combined_lgm.size) *
			combined_lgm.cellSize);
	    if (d > 0 && d < 2) {
         // There is a path to this destination
	      m_samples[2*i] = randx;
	      m_samples[2*i+1] = randy;
	      int the = (int)(rand() % angles.size());
	      m_samplestheta[i] = angles[the];
	      i++;
	    }
	    /*if reachable*/
	  }
	}
	    
      }
    }
  }


    log("Calculating view cones for generated samples");
    Cure::Pose3D candidatePose;
    XVector3D a;
    
    for (int y=0; y < m_samplesize; y++) { //calc. view cone for each sample
      
      m_krsjlgm->index2WorldCoords(m_samples[y*2],m_samples[2*y+1],a.x,a.y);
      a.theta =  m_samplestheta[y];
      tpoints = GetInsideViewCone(a, true);
      ViewConePts.push_back(tpoints);
      candidatePose.setTheta(m_samplestheta[y]);
      candidatePose.setX(a.x);
      candidatePose.setY(a.y);
      //log("CurrentPose.Theta : %f", candidatePose.getTheta());
      candidatePoses.push_back(candidatePose);
    }
    log("View Cones calculated.");
    if (m_Displaykrsjlgm == 0)
      m_Displaykrsjlgm = new Cure::X11DispLocalGridMap<unsigned char>(*m_krsjlgm);
    if (m_Displaycoverage == 0)
      m_Displaycoverage = new Cure::X11DispLocalGridMap<unsigned char>(*fcm);
    
    
    displayOn = true;
  }
  long ObjectSearch::GetClosestFNode(double xW, double yW){
    long nodeid;
    double hdist = DBL_MAX;
    for (unsigned int i = 0; i < fnodeseq.size() ; i++){
      double dist = sqrt( pow((xW - fnodeseq[i]->x),2) + pow((yW - fnodeseq[i]->y),2) );
      //log("node pose: %.2f,%.2f dist: %f", fnodeseq[i]->x,fnodeseq[i]->y, dist); 
      if ( dist < hdist){
	hdist = dist;
	nodeid = fnodeseq[i]->nodeId;
      }
    }
    //log("closest node id : for point %f,%f is %i",xW,yW,nodeid);
    return nodeid;
  }
  std::vector<int> ObjectSearch::GetInsideViewCone(XVector3D &a, bool addall) {
    tpoints.clear();
    XVector3D b,c,p;
    XVector3D m_a,m_b,m_c;
    int* rectangle = new int[4];
    int h,k;
    CalculateViewCone(a,a.theta,m_CamRange,m_fov,b,c);
    
    m_krsjlgm->worldCoords2Index(a.x,a.y,h,k);
    m_a.x = h;
    m_a.y = k;
    m_krsjlgm->worldCoords2Index(b.x,b.y,h,k);
    m_b.x = h;
    m_b.y = k;
    m_krsjlgm->worldCoords2Index(c.x,c.y,h,k);
    m_c.x = h;
    m_c.y = k;
    //  log("Got Map triangle coordinates: A:(%f,%f),B:(%f,%f),C:(%f,%f) \n", m_a.x,m_a.y,m_b.x,m_b.y,m_c.x,m_c.y);
    
    FindBoundingRectangle(m_a,m_b,m_c,rectangle);
    //log("XRectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",rectangle[0],rectangle[2]
    //,rectangle[1], rectangle[3]);
    for (int x=rectangle[0]; x < rectangle[1] ; x++) // rectangle bounding triangle
      {
	for (int y=rectangle[2]; y < rectangle[3]; y++) {
	  p.x = x;
	  p.y = y;
	  if (addall){
	    if (isPointInsideTriangle(p, m_a,m_b,m_c)) {
	      tpoints.push_back(x);
	      tpoints.push_back(y);
	    }
	  }
	  else {
	    if ((*coveragemap)(x,y) == 1) {
	      if (isPointInsideTriangle(p, m_a,m_b,m_c)) {
		tpoints.push_back(x);
		tpoints.push_back(y);
	      }
	    }
	  }
	}
      }
    vector<int>::iterator theIterator = tpoints.begin();
    tpoints.insert( theIterator, 1, m_a.y);
    theIterator = tpoints.begin();
    tpoints.insert( theIterator, 1, m_a.x);
    return tpoints;
    
  }
  
  std::vector<double> ObjectSearch::ScorebyCoverage(Cure::LocalGridMap<unsigned char> fcm ) {
    std::vector<double> CoverageSum;
    int covered = m_covered;
    for (unsigned int i = 0; i < ViewConePts.size(); i++) {
		double score = GetExtraCoverage(ViewConePts[i], covered, fcm);
        CoverageSum.push_back(score);
//        log("Coverage Sum for view cone %i is: %f",i, score);
	}
	return CoverageSum;
}

bool comp (double i,double j) {
    return (i>j);
}
ObjectSearch::SearchPlan ObjectSearch::GeneratePlan(double covpercent,std::vector<double> PDFsum) {
    SearchPlan searchplan;
    log("GeneratePlan");
    searchplan.plan.clear();
    searchplan.totalcoverage = 0;
    searchplan.indexarray.clear();
    std::vector<double> copy = PDFsum;

    Cure::LocalGridMap<unsigned char> fcm(*coveragemap);
    int covered = m_covered;
    sort(copy.begin(),copy.end(),comp);
    while(copy.size() > 0) {
        double val = copy[0];
        unsigned int j = 0;
        debug("val is: %f",val);
        for (;j < PDFsum.size(); j++) {
            if (PDFsum[j] == val) { //find the index with value val in PDFsum
                //log("found at %ith view point",j);
                break;
            }
        }

        copy.erase(copy.begin()); //chop off the first element so next time in loop copy[0] is the next element
		std::vector<int> rollback;
		double extracoverage = GetExtraCoverage(ViewConePts[j],covered, fcm, true, rollback);
		//println("Extracoverage: %f",extracoverage);
        if (extracoverage > m_vpthreshold) { // if worthy of adding to the plan
        	//just to display the selected viewcone
			
        	XVector3D a;
        	m_krsjlgm->index2WorldCoords(m_samples[j*2],m_samples[2*j+1],a.x,a.y);
        	a.theta =  m_samplestheta[j];
        	ViewConePts[j] = GetInsideViewCone(a, true); 
			searchplan.extracoverage.push_back(extracoverage);
            searchplan.plan.push_back(candidatePoses[j]);
            searchplan.indexarray.push_back(j);
            searchplan.totalcoverage += extracoverage;
            //println("Total Coverage %f",  searchplan.totalcoverage);
            //println("Added to plan");
            if (searchplan.totalcoverage >= m_covthresh) {
              //  println("Found plan covers %f of the area", searchplan.totalcoverage);
			//	println("%i out of  %i points", covered, m_coveragetotal);
				return searchplan;
            }
        } else {
			int x,y;
			for (unsigned int j = 0; j < rollback.size()/2; j++) {
				x = rollback[2*j];
				y = rollback[2*j + 1];
					fcm(x,y) = 1;
			}
        }


    }
    if (searchplan.totalcoverage < m_covthresh) {
        //println("Plan does not cover %f only %f", covpercent, searchplan.totalcoverage);
		//println("%i out of  %i points", covered, m_coveragetotal);
    }
    	return searchplan;
}

double ObjectSearch::GetExtraCoverage(std::vector<int> tpoints, int &covered,Cure::LocalGridMap<unsigned char> &fcm) {
	std::vector<int> rollback;
return GetExtraCoverage(tpoints, covered, fcm, false, rollback);
}
double ObjectSearch::GetExtraCoverage(std::vector<int> tpoints, int &covered,Cure::LocalGridMap<unsigned char> &fcm, bool changefcm,
										std::vector<int> &rollback ) {
    double oldcov = (double)100*covered/m_coveragetotal;
  //  log("total coverage space: %i, covered: %i,oldcov: %f",m_coveragetotal,covered, oldcov);

    int h = 0;
    int x,y;
    for (unsigned int j = 0; j < tpoints.size()/2; j++) {
        x = tpoints[2*j];
        y = tpoints[2*j + 1];
        if ((fcm)(x,y) == '1') {
            h++;
			if(changefcm)
				fcm(x,y) = '2';
			rollback.push_back(x);
			rollback.push_back(y);
        }
    }
    double cov = (double)100*(covered+h)/m_coveragetotal;
//    log("h: %i, Difference in cov: %f", h, (cov-oldcov));
	return (cov-oldcov);
}

void ObjectSearch::CalculateViewCone(XVector3D a, double direction, double range, double fov, XVector3D &b,XVector3D &c) {
    //log("Direction: %f, FOV:%f, range: %f", direction, fov, range);
    float angle1 = direction + fov/2;
    float angle2 = direction - fov/2;
    b.x = cos(angle1)*range + a.x;
    c.x = cos(angle2)*range + a.x;
    b.y = sin(angle1)*range + a.y;
    c.y = sin(angle2)*range + a.y;
    //log("Got triangle coordinates: A:(%f,%f),B:(%f,%f),C:(%f,%f) \n", a.x,a.y,b.x,b.y,c.x,c.y);
}
void ObjectSearch::FindBoundingRectangle(XVector3D a,XVector3D b,XVector3D c, int* rectangle) {
    int maxx,maxy,minx,miny;
    maxy = max(max(a.y,b.y),c.y);
    maxx = max(max(a.x,b.x),c.x);
    miny = min(min(a.y,b.y),c.y);
    minx = min(min(a.x,b.x),c.x);
    rectangle[0] = minx;
    rectangle[1] = maxx;
    rectangle[2] = miny;
    rectangle[3] = maxy;
    //log("Rectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",minx,miny,maxx, maxy);
}
bool ObjectSearch::isPointInsideTriangle(XVector3D p,XVector3D a,XVector3D b,XVector3D c) { //the first one is the point the rest triangle

    if (isPointSameSide(p,a, b,c) && isPointSameSide(p,b, a,c)
            && isPointSameSide(p,c, a,b)) {
        return true;
    } else {
        return false;
    }


}
bool ObjectSearch::isPointSameSide(XVector3D p1,XVector3D p2,XVector3D a,XVector3D b) {
    XVector3D cp1 = (b - a).crossVector3D((p1-a));
    XVector3D cp2 = (b - a).crossVector3D((p2-a));
    if (cp1.dotVector3D(cp2) >= 0 ) {
        return true;
    } else {
        return false;
    }

}
void ObjectSearch::owtNavCommand(const cdl::WorkingMemoryChange & objID) {

  if(m_status == STOPPED) {
    log("Ignoring NAV command overwrite after STOP received");
    return;
  }

  log("NavCommand Overwritten: %s", objID.address.id.c_str());
  boost::shared_ptr<CASTData<SpatialData::NavCommand> > oobj =
    getWorkingMemoryEntry<SpatialData::NavCommand>(objID.address);
  
  if (oobj != 0) {
    switch(oobj->getData()->comp) {
    case SpatialData::COMMANDSUCCEEDED:
      m_status = NAVCOMMANDCOMPLETED;
      if (m_plan.plan.size() == 0) {
	m_command = PLAN;
      } 
      else {
	
	Cure::Pose3D currpos = m_currPose;
	double plantheta = m_plan.plan[whereinplan].getTheta();
	double anglediff = Cure::HelpFunctions::angleDiffRad(plantheta,currpos.getTheta());
	log("plantheta : %f, currtheta, %f", plantheta, currpos.getTheta());
	log("anglediff is: %f", anglediff);
	if ( fabs(anglediff) > M_PI_2 ) {
	  log("First turning the robot a little.");
	  double turnangle;
	  if (anglediff < 0) {
	    turnangle = anglediff + M_PI_2 - 0.2;
	  } else {
	    turnangle = anglediff - M_PI_2 + 0.2;
	  }
	  SpatialData::NavCommandPtr cmd = newNavCommand();
	  cmd->prio = SpatialData::URGENT;
	  cmd->cmd = SpatialData::TURN;
	  cmd->angle.resize(1);
	  cmd->angle[0] = turnangle;
	  cmd->tolerance.resize(1);
	  cmd->tolerance[0] = 0.1;
	  
	  new NavCommandReceiver(*this, cmd);
	  break;
	}
	
	log("command set to recognize");
	m_command = RECOGNIZE;
      }
      log("Task accomplished!");
      break;
    case SpatialData::COMMANDFAILED:
      m_status = NAVCOMMANDCOMPLETED;
      m_command = EXECUTE;
      log("Task failed");
      break;
    case SpatialData::COMMANDABORTED:
      log("Task aborted :(");
      break;
    case SpatialData::COMMANDINPROGRESS:
      log("Task in progress...");
      break;
    case SpatialData::COMMANDPENDING:
      log("Task pending...");
      break;
    }
  }
}
void ObjectSearch::ObjectDetected(const cast::cdl::WorkingMemoryChange &objID) {

  if(continueToRecognize()) {
  shared_ptr<CASTData<VisionData::VisualObject> > oobj =
    getWorkingMemoryEntry<VisionData::VisualObject>(objID.address);
  
  VisionData::VisualObjectPtr obj = oobj->getData();
  if(obj->detectionConfidence >= 0.5){
    log("ok, detected '%s'", obj->label.c_str());
    
    NavData::ObjObsPtr obs = new NavData::ObjObs;
    obs->category = obj->label;
    obs->time = obj->time;
    
    static long objid = 0;
    obs->id.resize(1);
    obs->id[0] = objid++;
    
    if (m_CtrlPTU) {
      ptz::PTZReading ptz = m_PTUServer->getPose();
      obs->angles.resize(2);
      obs->angles[0] = ptz.pose.pan;
      obs->angles[1] = ptz.pose.tilt;
    }
    
    // FIXME Use the position of the object in the image as well
    // to get a better estimate of the direction to the object
    
    // FIXME Estimate the distance as well based on eg bounding box size
    
    addToWorkingMemory(newDataID(), obs);


    //HACK stopping AVS on first detection for AAAI comparison runs
    
    println("HACK: stopping AVS on first detection");
    stopAVS();
    return;
  }
  else{
    log("nah, did not detect '%s'", obj->label.c_str());	  
  }
  
  if (obj->label == m_objectlist[m_objectlist.size()-1]->ObjID) {
    log("got the last object. recognition complete.");
    if(m_status != STOPPED) {
      m_status = RECOGNITIONCOMPLETE;  
    }
  }
  }
  
}

bool ObjectSearch::continueToRecognize() const {
  return m_status == RECOGNITIONINPROGRESS;
}

void ObjectSearch::Recognize(){
	ptz::PTZReading ptz;
	ptz.pose.pan = 0;
	if (m_CtrlPTU)
		ptz::PTZReading ptz = m_PTUServer->getPose();
		
	Cure::Pose3D currpos = m_currPose;
	double plantheta = m_plan.plan[whereinplan].getTheta();
	double anglediff = Cure::HelpFunctions::angleDiffRad(plantheta,currpos.getTheta());
	
	log("plantheta : %f, currtheta, %f", plantheta, currpos.getTheta());
	log("anglediff is: %f", anglediff);
	log("ptz reading: %f", ptz.pose.pan);
	
	MovePanTilt(anglediff,0);

	if(m_status == STOPPED) {
	  log("Stopping in Recognize()");
	  return;
	}
	else {
	  m_status = RECOGNITIONINPROGRESS;
	}

	//need to unlock to allow changes through for the original design, but this is dodgy
	unlockComponent();

	PostRecognitionCommand();
	while(continueToRecognize()) {
	  sleepComponent(10);
	}

	//this only makes sense if using ptu
	if(m_CtrlPTU && m_status != STOPPED) {

	  log("now moving extras");
	  int n = 1;

	  //postive
	  while(continueToRecognize() && anglediff + n*m_ptustep < M_PI/2){
	    MovePanTilt(anglediff + n*m_ptustep,0);
	    PostRecognitionCommand();
	    while(continueToRecognize())  {
	      sleepComponent(10);
	    }
	    n++;
	  }

	  //negative
	  n= 1;
	  while(continueToRecognize() && anglediff - n*m_ptustep > -M_PI/2){
	    MovePanTilt(anglediff - n*m_ptustep,0);
	    PostRecognitionCommand();
	    while(continueToRecognize())  {
	      sleepComponent(10);
	    }
	    n++;
	  }

	  if(m_tiltRads != 0) {

	    //negative with tilt
	    n= 1;
	    while(continueToRecognize() && anglediff - n*m_ptustep > -M_PI/2){
	      MovePanTilt(anglediff - n*m_ptustep,m_tiltRads);
	      PostRecognitionCommand();
	      while(continueToRecognize())  {
		sleepComponent(10);
	      }
	      n++;
	    }
	  

	    MovePanTilt(anglediff,m_tiltRads);
	    PostRecognitionCommand();
	    while(continueToRecognize()) {
	      sleepComponent(10);
	    }
	    

	    //postive with tilt
	    n = 1;
	    while(continueToRecognize() && anglediff + n*m_ptustep < M_PI/2){
	      MovePanTilt(anglediff + n*m_ptustep,m_tiltRads);
	      PostRecognitionCommand();
	      while(continueToRecognize()) {
		sleepComponent(10);
	      }
	      n++;
	    }	    
	  }
	}
	
	MovePanTilt(0,0);
	
	//belt up for safety
	lockComponent();	

	
}
void ObjectSearch::PostRecognitionCommand(){
    log("Posting Recog. Command now");	
    VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
    vector<string> obj_labels;
    for (unsigned int i= 0; i < m_objectlist.size(); i++)
      {
	obj_labels.push_back((m_objectlist[i]->ObjID));
      }
    cmd->labels = obj_labels;
    addToWorkingMemory(newDataID(), "vision.sa", cmd);
    log("DetectionCommand added.");
}

void ObjectSearch::newRobotPose(const cast::cdl::WorkingMemoryChange &objID) {
    shared_ptr<CASTData<NavData::RobotPose2d> > oobj =
        getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);

        m_currPose.setX(oobj->getData()->x);
        m_currPose.setY(oobj->getData()->y);
        m_currPose.setTheta(oobj->getData()->theta);

}

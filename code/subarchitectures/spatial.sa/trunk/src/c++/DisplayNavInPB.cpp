//
// = FILENAME
//    DisplayNavInPB.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//	  Alper Aydemir
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include <list>
#include <string>
#include "DisplayNavInPB.hpp"
#include <NavData.hpp>
#include <Laser.hpp>
#include <CureHWUtils.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>

#ifdef DISPLAY_COMA
#include <FrontierInterface.hpp>
#include <ComaData.hpp>
#endif
#include <Navigation/NavGraph.hh>
#include <Transformation/Pose3D.hh>
#include <AddressBank/ConfigFileReader.hh>
#include <Utils/CureDebug.hh>
#include <Navigation/NavGraphNode.hh>
#include <Navigation/NavGraphEdge.hh>
#include <Navigation/NavGraphGateway.hh>
#include <boost/numeric/ublas/matrix.hpp>
#include <VisionData.hpp>
#include <SpatialData.hpp>
using namespace std;
using namespace cast;
using namespace boost;
using namespace navsa;
//using namespace VisionData;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new DisplayNavInPB();
  }
}

DisplayNavInPB::DisplayNavInPB() {
  m_LaserConnected = false;

  m_NoPeopleModel = false;
  
  m_LaserServerHost = "localhost";

  m_RobotPose = 0;
  m_LineMap = 0;

  m_FovH = 45.0;
  m_FovV = 35.0;
  previouscenter.assign(3,0.0);
}

DisplayNavInPB::~DisplayNavInPB() 
{
}

void DisplayNavInPB::configure(const map<string,string>& _config) 
{
  log("configure entered");

  m_ShowRobot = (_config.find("--no-robot") == _config.end());
  m_ShowWalls = (_config.find("--no-walls") == _config.end());
  m_ShowGraph = (_config.find("--no-graph") == _config.end());
  m_ShowPeople = (_config.find("--no-people") == _config.end());
  m_ShowScans = (_config.find("--no-scans") == _config.end());
  m_ShowObjects = (_config.find("--no-objects") == _config.end());
  m_ShowNodeClass = (_config.find("--no-nodeclass") == _config.end());
  m_ShowAreaClass = (_config.find("--no-areaclass") == _config.end());
  m_ShowRobotViewCone = (_config.find("--no-robotviewcone") == _config.end());
  m_ShowPlanePoints = (_config.find("--no-plane-points") == _config.end());
  m_ShowSOIs = (_config.find("--no-sois") == _config.end());
  m_ShowPeopleId = (_config.find("--people-id") != _config.end());
  m_NonUniqueObjects = (_config.find("--non-unique") != _config.end());
  m_ReadPTU = (_config.find("--read-ptu") != _config.end());

  if (_config.find("--laser-server-host") != _config.end()) {
    std::istringstream str(_config.find("--laser-server-host")->second);
    str >> m_LaserServerHost;
  }

  if (_config.find("--fov-hor") != _config.end()) {
    std::istringstream str(_config.find("--fov-hor")->second);
    str >> m_FovH;
  }
  if (_config.find("--fov-vert") != _config.end()) {
    std::istringstream str(_config.find("--fov-vert")->second);
    str >> m_FovV;
  }

  m_RetryDelay = 10;
  if(_config.find("--retry-interval") != _config.end()){
    std::istringstream str(_config.find("--retry-interval")->second);
    str >> m_RetryDelay;
  }

  Cure::ConfigFileReader *cfg = 0;

  map<string,string>::const_iterator confIter = _config.find("-c");
  if (confIter != _config.end()) {
    cfg = new Cure::ConfigFileReader;
    log("About to try to open the config file");
    if (cfg->init(confIter->second) != 0) {
      delete cfg;
      cfg = 0;
      log("Could not init Cure::ConfigFileReader with -c argument");
    } else {
      log("Managed to open the Cure config file");
    }
  }

  m_PbPort = 5050;
  m_PbHost = "localhost";
  m_PbRobotName = "robot";
  m_PbRobotFile = "CogXp3.xml";
  m_PbPersonFile = "rolf.pbmf";

  if (cfg) {

    //cfg->getRobotName(m_PbRobotName);
  
    // To be backward compatible with config files that specify the
    // RoboLook host and really mean peekabot we read that first and
    // overwrite it below if both are specified
    cfg->getRoboLookHost(m_PbHost);

    std::string usedCfgFile, tmp;
    if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
      m_PbHost = tmp;
    }
    if (cfg->getString("PEEKABOT_ROBOT_XML_FILE", true, tmp, usedCfgFile) == 0){
      m_PbRobotFile = tmp;
    }
    if (cfg->getString("PEEKABOT_PERSON_PBMF_FILE", true, tmp, usedCfgFile) == 0){
      m_PbPersonFile = tmp;
    }

    if (cfg->getSensorPose(2, m_CameraPoseR)) {
      println("configure(...) Failed to get sensor pose for camera. (Run with --no-planes to skip)");
      std::abort();
    } 
  }

  log("Using %s as the robotfile in peekabot", m_PbRobotFile.c_str());
  log("Using %s as the person in peekabot", m_PbPersonFile.c_str());

  connectPeekabot();  

  if (m_ReadPTU) {
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

  log("configure done");
}

void DisplayNavInPB::start() {

  // Hook up changes to the robot pose to a callback function
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newRobotPose));  

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newRobotPose));  
	
  // Hook up changes to the nav data to a callback function
  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::ADD),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newNavGraphNode));  
  addChangeFilter(createLocalTypeFilter<NavData::FNode>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newNavGraphNode));  

  addChangeFilter(createLocalTypeFilter<NavData::ObjData>(cdl::ADD),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newNavGraphObject));  
  addChangeFilter(createLocalTypeFilter<NavData::ObjData>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newNavGraphObject));  

  addChangeFilter(createLocalTypeFilter<NavData::Area>(cdl::ADD),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                      &DisplayNavInPB::newArea));  
  addChangeFilter(createLocalTypeFilter<NavData::Area>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                      &DisplayNavInPB::newArea));  
  
  addChangeFilter(createLocalTypeFilter<NavData::AEdge>(cdl::ADD),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newNavGraphEdge));  
  addChangeFilter(createLocalTypeFilter<NavData::AEdge>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newNavGraphEdge));  

  // Hook up changes to the tracked People to a callback function
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::ADD),
		  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                           &DisplayNavInPB::newPerson));
  
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                           &DisplayNavInPB::newPerson));    
  
  addChangeFilter(createLocalTypeFilter<NavData::Person>(cdl::DELETE),
		  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                           &DisplayNavInPB::deletePerson));    


  // Hook up changes to the id of the tacked person to a callback function
  addChangeFilter(createLocalTypeFilter<NavData::PersonFollowed>(cdl::ADD),
		  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                               &DisplayNavInPB::newPersonFollowed));
  addChangeFilter(createLocalTypeFilter<NavData::PersonFollowed>(cdl::OVERWRITE),
		  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                               &DisplayNavInPB::newPersonFollowed));

  // Hook up changes to the LineMap to a callback function

  addChangeFilter(createLocalTypeFilter<NavData::LineMap>(cdl::ADD),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newLineMap));  
  addChangeFilter(createLocalTypeFilter<NavData::LineMap>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newLineMap));  
										
  addChangeFilter(createLocalTypeFilter<NavData::ObjectSearchPlan>(cdl::ADD),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newVPlist));  
  addChangeFilter(createLocalTypeFilter<NavData::ObjectSearchPlan>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newVPlist));

  if (m_ShowSOIs) {
    addChangeFilter(createGlobalTypeFilter<VisionData::SOI>(cdl::ADD),
	new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
	  &DisplayNavInPB::newPointCloud));

    addChangeFilter(createGlobalTypeFilter<VisionData::SOI>(cdl::OVERWRITE),
	new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
	  &DisplayNavInPB::newPointCloud));
  }

  if (m_ShowPlanePoints) {
    addChangeFilter(createLocalTypeFilter<SpatialData::PlanePoints>(cdl::ADD),
	new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
	  &DisplayNavInPB::newPlanePointCloud));

    addChangeFilter(createLocalTypeFilter<SpatialData::PlanePoints>(cdl::OVERWRITE),
	new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
	  &DisplayNavInPB::newPlanePointCloud));
  }

#ifdef DISPLAY_COMA
    // Hook up listener for changes to the coma rooms
  addChangeFilter(createGlobalTypeFilter<comadata::ComaRoom>(cdl::ADD),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newComaRoom));
  addChangeFilter(createGlobalTypeFilter<comadata::ComaRoom>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<DisplayNavInPB>(this,
                                        &DisplayNavInPB::newComaRoom));
#endif

  log("start done");  
}



void DisplayNavInPB::newPlanePointCloud(const cast::cdl::WorkingMemoryChange &objID) {
  log("new PlanePointCloud received.");

  try {

    SpatialData::PlanePointsPtr objData = getMemoryEntry<SpatialData::PlanePoints>(objID.address);


    double color[3] = { 0.9, 0, 0};

    numeric::ublas::matrix<double> m (3, 3);
    m(0,0) = 0; m(0,1) = 1; m(0,2) = 0;
    m(1,0) = 0; m(1,1) = 0; m(1,2) = -1;
    m(2,0) = 1; m(2,1) = 0; m(2,2) = 0;


    peekabot::PointCloudProxy pcloud;
    char tmp[256];
    sprintf(tmp, "planepoints%s", objID.address.id.c_str());
    pcloud.add(m_PeekabotClient,tmp, peekabot::REPLACE_ON_CONFLICT);

    numeric::ublas::vector<double> v (3);
    numeric::ublas::vector<double> t (3);
    //add plane points
    log("points size: %d",objData->points.size());
    for (unsigned int i =0; i < objData->points.size(); i++){
      v(0) = objData->points.at(i).x;
      v(1) = objData->points.at(i).y;
      v(2) = objData->points.at(i).z;
      //   t = prod(v,m);
      pcloud.add_vertex(v(0),v(1),v(2));

    }
    pcloud.set_color(color[0],color[1],color[2]);
  }
  catch (DoesNotExistOnWMException) {
    log("Error! plane point cloud disappeared from WM.");
  }

}

#ifdef DISPLAY_COMA
void DisplayNavInPB::newComaRoom(const cast::cdl::WorkingMemoryChange &objID){
 
    if (!m_PeekabotClient.is_connected()) return;

    shared_ptr<CASTData<comadata::ComaRoom> > oobj =
        getWorkingMemoryEntry<comadata::ComaRoom>(objID.address);

    comadata::ComaRoomPtr croom = (oobj->getData());

    ::Ice::Int roomId = croom->roomId;

    ::FrontierInterface::PlaceInterfacePrx agg(getIceServer<FrontierInterface::PlaceInterface>("place.manager"));
	
    println("New Coma Room recieved: id=%d",roomId);

    m_Mutex.lock();
    m_PeekabotClient.begin_bundle();
	
    // for each node in room change 
    for (::std::vector< ::Ice::Long>::iterator iter = croom->containedPlaceIds.begin();
        iter != croom->containedPlaceIds.end();
        iter++ )
    {
        // get node id from place id
        ::NavData::FNodePtr fnodePtr = agg->getNodeFromPlaceID( (::Ice::Int) *iter); // why is iter a long?
	    
        // search m_Nodes for place and change its room id	    
        std::map<long,Node>::iterator nodeIter = m_Nodes.find(fnodePtr->nodeId);

        if (nodeIter == m_Nodes.end()) {
            println("error: coma room %d contains a node I do not know about: placeID = %d",roomId, *iter);
            continue;
        }

        DisplayNavInPB::Node node = nodeIter->second;
        node.m_areaId = roomId; // is this correct?


        if(node.m_Gateway) {
            // If there is a gateway node here it means the node has changed to
            // a door while we were processing this ComaRoom, so we can ignore it
            println("Wasn't expecting a gateway node (id %d) here! (DisplayNavInPB::newComaRoom)",node.m_Id);
            log    ("Wasn't expecting a gateway node (id %d) here! (DisplayNavInPB::newComaRoom)",node.m_Id);
        } else {
            // let peekabot know
            peekabot::SphereProxy sp;
            char name[32];
            sprintf(name, "node%ld", (long)node.m_Id);
            sp.add(m_ProxyNodes, name, peekabot::REPLACE_ON_CONFLICT);
            sp.set_position(node.m_X, node.m_Y, 0);

            float r,g,b;
            getColorByIndex(roomId, r, g, b);
            sp.set_scale(0.1, 0.1, 0.05);
            println("Added coma place with id %d", node.m_Id);
            sp.set_color(r,g,b);    
            
            if (m_ShowNodeClass) 
            {
                peekabot::CylinderProxy cp;
                cp.add(sp, "class", peekabot::REPLACE_ON_CONFLICT);
                cp.set_scale(0.04, 0.04, 0.16);
                cp.set_position(0,0,0.08);
                //FIXME
                //place::ColorMap::getColorForPlaceClass(fnode->areaTypeNo, r, g, b);
                getColorByIndex(roomId, r, g, b);
                cp.set_color(r,g,b);

                peekabot::SphereProxy mp;
                mp.add(sp, "mushroom");
                mp.set_scale(0.08, 0.08, 0.05);
                mp.set_position(0, 0, 0.16);
                mp.set_color(r,g,b);
            }
            
            if (m_ShowAreaClass) {
                peekabot::CylinderProxy acp;
                char name2[32];
                sprintf(name2, "node%ld.area_class", node.m_Id);
                acp.add(m_ProxyNodes, name2, peekabot::REPLACE_ON_CONFLICT);
                acp.set_scale(0.5, 0.5, 0.0);
                acp.set_position(0,0,0);
                acp.set_opacity(0.3);
                //FIXME
                //place::ColorMap::getColorForPlaceClass(fnode->areaTypeNo, r, g, b);
                getColorByIndex(roomId, r, g, b);
                //getColorByIndex(3+(fnode->areaId%6), r, g, b);
                acp.set_color(r,g,b);
            }
            
            // We also redraw the edge just to make sure that it is drawn
            // correctly if the position of the node changed.
            redisplayEdgesToNode(node);
        }
    } // end for
    m_PeekabotClient.end_bundle();
    m_Mutex.unlock();
}
#endif

void DisplayNavInPB::newVPlist(const cast::cdl::WorkingMemoryChange &objID) {
  debug("VPnodelist called"); 

  if (!m_PeekabotClient.is_connected()) return;

  // Visualize Viewplan boost::shared_ptr<CASTData<NavData::NavCommand> >
  boost::shared_ptr<CASTData<NavData::ObjectSearchPlan> > oobj =
    getWorkingMemoryEntry<NavData::ObjectSearchPlan>(objID.address);

  NavData::ObjectSearchPlanPtr plan = oobj->getData();

  m_ProxyViewPoints.add(m_PeekabotClient, "viewpoints",peekabot::REPLACE_ON_CONFLICT);

  // Get nodeIDs stated in the plan, and search through navGraph to
  // each nodeID and finally put ordered numbers on top of nodes that
  // are in the plan.
  char path[32];
  if(plan->planlist.size() > 0)
  {

    double color[3];
    color[0] = 0.9;
    color[1] = 0.1;
    color[2] = 0.1;
    for (unsigned int i = 0; i < plan->planlist.size(); i++){
      sprintf(path,"viewpoint_%i",i);
      createFOV(m_ProxyViewPoints, path, m_FovH, m_FovV, color, 0.5, plan->planlist[i], false);

    }

  }

}



void DisplayNavInPB::createRobotFOV() 
{
  if (!m_ShowRobotViewCone) return;

  std::string path = "peoplebot_base.ptu.pan.tilt.stereo_cam.cam_right";

  double color[3];
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;

  if (m_PbRobotFile == "Robone.xml") {
    path = "peoplebot_base.ptu.pan.tilt.stereo_cam";
  } else {
    path = "model.camera";
  }

  peekabot::GroupProxy cam;
  cam.assign(m_ProxyRobot, path);
  NavData::ViewPoint vp;
  vp.pos.z = 0;
  vp.pan = 0;
  vp.tilt = 0;
  vp.length = 1.0;
  createFOV(cam, "cam_right.cone", m_FovH, m_FovV, color, 0.3, vp);
}

void DisplayNavInPB::createFOV(peekabot::GroupProxy &proxy, const char* path, 
                               double fovHorizAngle, double fovVertiAngle, 
                               double* color, double opacity, 
                               NavData::ViewPoint viewpoint, bool robotfov){

		peekabot::GroupProxy proxyCone;	
		proxyCone.add(proxy, path, peekabot::REPLACE_ON_CONFLICT);
  
  const double coneLen = viewpoint.length;
  // The "half angle" of the field of view
  const double fovHoriz = fovHorizAngle*M_PI/180.0/2;
  const double fovVerti = fovVertiAngle*M_PI/180.0/2;
  peekabot::PolygonProxy proxyConeParts[5];
  proxyConeParts[0].add(proxyCone, "top");
  proxyConeParts[0].add_vertex(0,0,0);
  proxyConeParts[0].add_vertex(coneLen,
                               coneLen*tan(fovHoriz),
                               coneLen*tan(fovVerti));
  proxyConeParts[0].add_vertex(coneLen,
                               coneLen*tan(-fovHoriz),
                               coneLen*tan(fovVerti));                         
  proxyConeParts[1].add(proxyCone, "bottom");
  proxyConeParts[1].add_vertex(0,0,0);
  proxyConeParts[1].add_vertex(coneLen,
                          coneLen*tan(fovHoriz),
                          coneLen*tan(-fovVerti));
  proxyConeParts[1].add_vertex(coneLen,
                          coneLen*tan(-fovHoriz),
                          coneLen*tan(-fovVerti));                         
  proxyConeParts[2].add(proxyCone, "left");
  proxyConeParts[2].add_vertex(0,0,0);
  proxyConeParts[2].add_vertex(coneLen,
                               coneLen*tan(fovHoriz),
                               coneLen*tan(-fovVerti));
  proxyConeParts[2].add_vertex(coneLen,
                               coneLen*tan(fovHoriz),
                               coneLen*tan(fovVerti));                         
  proxyConeParts[3].add(proxyCone, "right");
  proxyConeParts[3].add_vertex(0,0,0);
  proxyConeParts[3].add_vertex(coneLen,
                          coneLen*tan(-fovHoriz),
                          coneLen*tan(-fovVerti));
  proxyConeParts[3].add_vertex(coneLen,
                          coneLen*tan(-fovHoriz),
                          coneLen*tan(fovVerti));                         
  proxyConeParts[4].add(proxyCone, "image");
  proxyConeParts[4].add_vertex(coneLen,
                          coneLen*tan(fovHoriz),
                          coneLen*tan(fovVerti));
  proxyConeParts[4].add_vertex(coneLen,
                          coneLen*tan(fovHoriz),
                          coneLen*tan(-fovVerti));
  proxyConeParts[4].add_vertex(coneLen,
                          coneLen*tan(-fovHoriz),
                          coneLen*tan(-fovVerti));
  proxyConeParts[4].add_vertex(coneLen,
                          coneLen*tan(-fovHoriz),
                               coneLen*tan(fovVerti));                         
 
  for (int i = 0; i < 5; i++) {
    proxyConeParts[i].set_color(color[0],color[1],color[2]);
    proxyConeParts[i].set_opacity(opacity);
    proxyConeParts[i].set_scale(1);  // This is how I make the cone
                                     // larger or smaller
    
  }	if (!robotfov )
	{

	//proxyCone.set_rotation(viewpoint.pan,viewpoint.tilt,0);
	proxyCone.rotate(viewpoint.pan,0,0,1);
	proxyCone.rotate(viewpoint.tilt,0,-1,0);
	proxyCone.set_position(viewpoint.pos.x, viewpoint.pos.y, viewpoint.pos.z);
			
	}
	else
	{

			proxyCone.set_rotation(viewpoint.pan,viewpoint.tilt,0);
		proxyCone.set_position(0,0,viewpoint.pos.z);
	}
}


void DisplayNavInPB::newPointCloud(const cdl::WorkingMemoryChange &objID){
  log("Got new SOI points.");
  double color[3] = { 0.9, 0, 0};

  try {
    VisionData::SOIPtr objData = getMemoryEntry<VisionData::SOI>(objID.address);

    Cure::Transformation3D cam2WorldTrans =
      getCameraToWorldTransform();
    double tmp[6];
    cam2WorldTrans.getCoordinates(tmp);
    log("total transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
	tmp[4], tmp[5]);

    //Convert hull to world coords
    for (unsigned int i = 0; i < objData->points.size(); i++) {
      Cure::Vector3D from(objData->points[i].p.x, objData->points[i].p.y, objData->points[i].p.z);
      Cure::Vector3D to;
      cam2WorldTrans.invTransform(from, to);
      //    log("vertex at %f, %f, %f", objData->points[i].p.x, objData->points[i].p.y, objData->points[i].p.z);
      objData->points[i].p.x = to.X[0];
      objData->points[i].p.y = to.X[1];
      objData->points[i].p.z = to.X[2];
      //    log("Transformed vertex at %f, %f, %f", to.X[0], to.X[1], to.X[2]);
    }

    //  numeric::ublas::matrix<double> m (3, 3);
    //  m(0,0) = 0; m(0,1) = 1; m(0,2) = 0;
    //  m(1,0) = 0; m(1,1) = 0; m(1,2) = -1;
    //  m(2,0) = 1; m(2,1) = 0; m(2,2) = 0;
    peekabot::PointCloudProxy pcloud;
    pcloud.add(m_PeekabotClient,"planepopout", peekabot::REPLACE_ON_CONFLICT);



    numeric::ublas::vector<double> v (3);
    //add plane points
    for (unsigned int i =0; i < objData->points.size(); i++){

      v(0) = objData->points.at(i).p.x;
      v(1) = objData->points.at(i).p.y;
      v(2) = objData->points.at(i).p.z;
      pcloud.add_vertex(v(0),v(1),v(2));

    }
    pcloud.set_color(color[0],color[1],color[2]);
  }
  catch (DoesNotExistOnWMException) {
    log("Error! SOI WM entry went missing!");
  }
  
}
void DisplayNavInPB::runComponent() {

  log("runComponent");

  setupPushScan2d(*this, 0.2, m_LaserServerHost);

  log("Connected to the laser");

  while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
    sleep(m_RetryDelay);
    connectPeekabot();
  }

  log("Connected to peekabot, ready to go");
  if (m_PeekabotClient.is_connected()) {
    while (isRunning()) {
      
      ptz::PTZReading ptuPose;
      if (m_ReadPTU) {
	ptuPose = m_PTUServer->getPose();
	//log("Read ptu pose %f %f", ptuPose.pose.pan, ptuPose.pose.tilt);
      } else {
	// If we are not connecting and reading from PTU we assume
	// angles are 0
	ptuPose.pose.pan = 0;
	ptuPose.pose.tilt = -0.75;
      }
      
      m_Mutex.lock();

      m_PeekabotClient.begin_bundle();


      // Display the last laser scan 
      if(m_ShowScans && m_LaserConnected && !m_Scan.ranges.empty()) {        


        m_ProxyScan.clear_vertices();
        double angStep = m_ScanAngFOV / (m_Scan.ranges.size() - 1);
        double startAng = -m_ScanAngFOV / 2;
        for (unsigned int i = 0; i < m_Scan.ranges.size(); i++) {
          float x,y;
          x = cos(startAng + i * angStep) * m_Scan.ranges[i];
          y = sin(startAng + i * angStep) * m_Scan.ranges[i];
          m_ProxyScan.add_vertex(x,y,0);
        }
        

      }

      // Display robot pose
      if(m_ShowRobot && m_RobotPose) {
        m_ProxyRobot.set_pose(m_RobotPose->x,
                              m_RobotPose->y,
                              0,
                              m_RobotPose->theta);

	m_ProxyPan.set_dof(ptuPose.pose.pan);
	m_ProxyTilt.set_dof(-ptuPose.pose.tilt);
      }

      // Display the line map
      if(m_ShowWalls && m_LineMap) {

        peekabot::GroupProxy walls;
        walls.add(m_PeekabotClient,
                  "walls",
                  peekabot::REPLACE_ON_CONFLICT);
        
        for (unsigned int i = 0; i < m_LineMap->lines.size(); i++) {

          peekabot::PolygonProxy pp;
          char buf[32];
          sprintf(buf, "poly%d", i);
          pp.add(walls, buf);

          pp.add_vertex(m_LineMap->lines[i].start.x,
                        m_LineMap->lines[i].start.y,
                        0);
          pp.add_vertex(m_LineMap->lines[i].start.x,
                        m_LineMap->lines[i].start.y,
                        1.0);
          pp.add_vertex(m_LineMap->lines[i].end.x,
                        m_LineMap->lines[i].end.y,
                        1.0);
          pp.add_vertex(m_LineMap->lines[i].end.x,
                        m_LineMap->lines[i].end.y,
                        0.0);

          pp.set_color(255./255, 198./255, 0.);
          pp.set_opacity(0.2);

        }

      }

      // Display the people being tracked
      if(m_ShowPeople) displayPeople();

      peekabot::Status s = m_PeekabotClient.end_bundle().status();

      m_Mutex.unlock();
      
      // Make sure the server processed what we've sent
      m_PeekabotClient.sync();

      if( s.failed() ) {
        debug("Bundle failed with error message: %s", 
	      s.get_error_message().c_str());
      }
      


      usleep(250000);
    }
  }
}

/*void DisplayNavInPB::newConvexHull(const cdl::WorkingMemoryChange
				   &objID){

  debug("newConvexHull called");
  shared_ptr<CASTData<VisionData::ConvexHull> > oobj =
 getWorkingMemoryEntry<VisionData::ConvexHull>(objID.address);
  m_Mutex.lock();
  VisionData::ConvexHullPtr m_ConvexHull = oobj->getData();

  if (previouscenter.at(0) == 0.0)

    {
      previouscenter.at(0) = m_ConvexHull->center.x;
      previouscenter.at(1) = m_ConvexHull->center.y;
      previouscenter.at(2) = m_ConvexHull->center.z;
      
      peekabot::GroupProxy planes;
      planes.add(m_PeekabotClient,
		 
		 "root.ConvexHull", peekabot::REPLACE_ON_CONFLICT);
      
      peekabot::PolygonProxy pp;
      char buf[32];
      pp.add(planes, buf);
      for (unsigned int i = 0; i < m_ConvexHull->PointsSeq.size(); i++)
	pp.add_vertex(m_ConvexHull->PointsSeq[i].x,
		      m_ConvexHull->PointsSeq[i].y, m_ConvexHull->PointsSeq[i].z);
      pp.set_color(1.0, 0.0, 0.0);
      pp.set_opacity(0.2);
    }
  else
    {
      
      double dist =
	sqrt((previouscenter.at(0)-m_ConvexHull->center.x)*(previouscenter.at(0)-m_ConvexHull->center.x)+(previouscenter.at(1)-m_ConvexHull->center.y)*(previouscenter.at(1)-m_ConvexHull->center.y)+(previouscenter.at(2)-m_ConvexHull->center.z)*(previouscenter.at(2)-m_ConvexHull->center.z));
      
      if (dist > m_ConvexHull->radius)
	
	{
	  
	  peekabot::GroupProxy planes;
	  planes.add(m_PeekabotClient,
		     "root.ConvexHull", peekabot::REPLACE_ON_CONFLICT);
	  
	  peekabot::PolygonProxy pp;
	  char buf[32];
	  pp.add(planes, buf);
	  for (unsigned int i = 0; i < m_ConvexHull->PointsSeq.size(); i++)
	    pp.add_vertex(m_ConvexHull->PointsSeq[i].x,
			  m_ConvexHull->PointsSeq[i].y, m_ConvexHull->PointsSeq[i].z);
	  pp.set_color(1.0, 0.0, 0.0);
	  pp.set_opacity(0.2);
	}
      
    }
  m_Mutex.unlock();
}*/

void DisplayNavInPB::displayPeople()
{
  peekabot::GroupProxy people;
  people.add(m_PeekabotClient,
             "people",
             peekabot::REPLACE_ON_CONFLICT);

  for (unsigned int i = 0; i < m_People.size(); i++) {
    
    char buf[32];
    sprintf(buf, "person%ld", (long)m_People[i].m_data->id);
    
    if (m_NoPeopleModel) {
      
      // Using a cylinde rmodel instead of beautiful Rolf
      
      peekabot::CylinderProxy cp;
      cp.add(people, buf, peekabot::REPLACE_ON_CONFLICT);
      cp.set_pose(m_People[i].m_data->x,
                  m_People[i].m_data->y,
                  0.9,
                  m_People[i].m_data->direction);      
      cp.set_scale(0.1, 0.2, 1.8);

      if (m_CurrPersonId == m_People[i].m_data->id) {
        cp.set_opacity(1);
      } else {
        cp.set_opacity(0.2);
      }

    } else {

      peekabot::ModelProxy mp;
      mp.add(people, buf, m_PbPersonFile, 
             peekabot::REPLACE_ON_CONFLICT);
      mp.set_pose(m_People[i].m_data->x,
                  m_People[i].m_data->y,
                  0,
                  m_People[i].m_data->direction);

      if (m_CurrPersonId == m_People[i].m_data->id) {
        mp.set_opacity(1);
      } else {
        mp.set_opacity(0.2);
      }

      if (m_ShowPeopleId) {
        sprintf(buf, "id%ld", (long)m_People[i].m_data->id);
        peekabot::LabelProxy text;
        text.add(mp, buf, peekabot::REPLACE_ON_CONFLICT);
        sprintf(buf, "%ld", (long)m_People[i].m_data->id);
        text.set_text(buf);
        text.set_pose(0,0,1.8,M_PI/2,0,M_PI/2);
        text.set_scale(30, 30, 30);
        text.set_alignment(peekabot::ALIGN_CENTER);
        text.set_color(1,0,0);
      }      
    }
  }
  
}

void DisplayNavInPB::receiveScan2d(const Laser::Scan2d &scan)
{
  cast::cdl::CASTTime ct;
  ct = getCASTTime();
  debug("Got scan n=%d, r[0]=%.3f a[0]=%.4f r[n-1]=%.3f da=%.4f t=%ld.%06ld @ t=%ld.%06ld",
        scan.ranges.size(), scan.ranges[0], scan.startAngle,
        scan.ranges[scan.ranges.size()-1], scan.angleStep,
        (long)scan.time.s, (long)scan.time.us,
        (long)ct.s, (long)ct.us);
  
  m_Mutex.lock();
  m_Scan = scan;
  m_Mutex.unlock();
}
     
void DisplayNavInPB::newArea(const cdl::WorkingMemoryChange &objID)
{
  log("new Area");

  if (!m_PeekabotClient.is_connected()) return;

  // Get the area struct
  shared_ptr<CASTData<NavData::Area> > oobj =
      getWorkingMemoryEntry<NavData::Area>(objID.address);
  NavData::AreaPtr area = oobj->getData();

  if (!m_ShowAreaClass) 
    return;

  // Protect
  m_Mutex.lock();
  m_PeekabotClient.begin_bundle();

  // Find nodes associated with this area
  list<NavData::LineMapSegement> walls;
  std::map<long, Node>::const_iterator ni;
  for (ni=m_Nodes.begin(); ni!=m_Nodes.end(); ni++)
  {
    if ((*ni).second.m_areaId == area->id)
    {
      if (!(*ni).second.m_Gateway) 
      {
        peekabot::CylinderProxy acp;
        char name2[32];
        sprintf(name2, "node%ld/area_class", (*ni).second.m_Id);
        log("adding: %s", name2);
	sleep(1);
	acp.add(m_ProxyNodes, name2, peekabot::REPLACE_ON_CONFLICT);
        acp.set_scale(0.5, 0.5, 0.0);
        acp.set_position(0,0,0);
        acp.set_opacity(0.2);
        float r,g,b;
        //FIXME
	//place::ColorMap::getColorForPlaceClass(area->m_areaTypeNo, r, g, b);
        getColorByIndex(3+(area->id%6), r, g, b);
        acp.set_color(r,g,b);
      }
    }
  }

  m_PeekabotClient.end_bundle();
  m_Mutex.unlock();
}

void DisplayNavInPB::newRobotPose(const cdl::WorkingMemoryChange &objID) 
{
  shared_ptr<CASTData<NavData::RobotPose2d> > oobj =
    getWorkingMemoryEntry<NavData::RobotPose2d>(objID.address);
  
  m_Mutex.lock();
  m_RobotPose = oobj->getData();
  m_Mutex.unlock();
  debug("newRobotPose(x=%.2f y=%.2f a=%.4f t=%ld.%06ld",
        m_RobotPose->x, m_RobotPose->y, m_RobotPose->theta,
        (long)m_RobotPose->time.s, (long)m_RobotPose->time.us); 
}

void DisplayNavInPB::newNavGraphObject(const cdl::WorkingMemoryChange &objID)
{  
  if (!m_ShowObjects) return;
  
  shared_ptr<CASTData<NavData::ObjData> > oobj =
    getWorkingMemoryEntry<NavData::ObjData>(objID.address);

  NavData::ObjDataPtr objData = oobj->getData();
  
  if (!m_PeekabotClient.is_connected()) {
    log("Received an object of category %s, not displaying it since not connected to peekabot", objData->category.c_str());
    return;
  }
  log("Received an object of category %s", objData->category.c_str());
  
  m_Mutex.lock();    

  m_PeekabotClient.begin_bundle();

  peekabot::CylinderProxy sp;
  peekabot::LabelProxy text;
  peekabot::ModelProxy objProxy;
  peekabot::CubeProxy centerProxy;

  char filename[128];
  if (objData->category == "borland_book") {
    sprintf(filename, "book_cpp.pbmf");
  } else {
    sprintf(filename, "%s.pbmf", objData->category.c_str());
  }

  if (m_NonUniqueObjects) {
    objProxy.add(m_ProxyObjects, objData->category, filename);
  } else {
    objProxy.add(m_ProxyObjects, objData->category, filename, 
                 peekabot::REPLACE_ON_CONFLICT);
  }

  if (objData->angles.empty()) {

    if (m_RobotPose) {
      
      objProxy.set_pose(objData->x, objData->y, objData->z,
                        atan2(objData->y - m_RobotPose->y,
                              objData->x - m_RobotPose->x), 0, 0);
    } else {
      // Assume robot is at 0,0,0
      objProxy.set_pose(objData->x, objData->y, objData->z,
                        atan2(objData->y - 0,
                              objData->x - 0), 0, 0);
    }

  } else {
    double ang[3] = {0,0,0};
    for (unsigned int i = 0; i < objData->angles.size(); i++) {
      ang[i] = objData->angles[i];
    }
    objProxy.set_pose(objData->x, objData->y, objData->z,
                      ang[0], ang[1], ang[2]);
  }

  // Add a center marker just in case the model file did not exist
  centerProxy.add(objProxy, "center");
  centerProxy.set_scale(0.05, 0.05, 0.05);
  centerProxy.set_color(0,1,0);

  text.add(objProxy,"label");
  text.set_text(objData->category);
  text.set_position(0, 0, 0.5);
  text.set_rotation(-M_PI_2,0,M_PI_2);
  text.set_scale(20, 20, 20);
  text.set_alignment(peekabot::ALIGN_CENTER); //see TextAlignment in peekabot/src/Types.hh for more.
  text.set_color(0,0,1);
  
  m_PeekabotClient.end_bundle();

  m_Mutex.unlock();
}
  
void DisplayNavInPB::newLineMap(const cdl::WorkingMemoryChange &objID)
{
  debug("newLineMap called");

  shared_ptr<CASTData<NavData::LineMap> > oobj =
    getWorkingMemoryEntry<NavData::LineMap>(objID.address);

  m_Mutex.lock();
  m_LineMap = oobj->getData();
  m_Mutex.unlock();
}

void DisplayNavInPB::newPerson(const cdl::WorkingMemoryChange &objID)
{
  // Person entries can be removed at any time
  try {
    shared_ptr<CASTData<NavData::Person> > oobj =
      getWorkingMemoryEntry<NavData::Person>(objID.address);
    
    NavData::PersonPtr p = oobj->getData();
    
    bool addNewPerson = true;
    
    m_Mutex.lock();

    // Check if the person already exists, otherwise add it
    for (unsigned int i = 0; i < m_People.size(); i++) {
      if (m_People[i].m_data->id == p->id) {
        // Update it
        
        char buf[256];
        sprintf(buf, "Got new person at x=%.2f y=%.2f theta=%.2f id=%ld",
                p->x, p->y, p->direction, (long)p->id);
        debug(buf);
        
        m_People[i].m_data = p;
        addNewPerson = false;
        break;
      }
    }
    
    if (addNewPerson) {
      DisplayNavInPB::PersonData pd;
      pd.m_WMid = objID.address.id;
      pd.m_data = p;
      m_People.push_back(pd);
    } 

  } catch(DoesNotExistOnWMException){}
  
  m_Mutex.unlock();
}

void DisplayNavInPB::deletePerson(const cdl::WorkingMemoryChange &objID)
{
  int i = 0;
  for (std::vector<DisplayNavInPB::PersonData>::iterator pi = m_People.begin();
       pi != m_People.end(); pi++, i++) {

    if (objID.address.id == pi->m_WMid) {
      m_People.erase(pi);
      break;
    }
  }
}

void DisplayNavInPB::newPersonFollowed(const cdl::WorkingMemoryChange &objID)
{
  shared_ptr<CASTData<NavData::PersonFollowed> > oobj =
    getWorkingMemoryEntry<NavData::PersonFollowed>(objID.address);
  
  m_Mutex.lock();
  m_CurrPersonId = oobj->getData()->id;
  char buf[256];
  sprintf(buf, "Got id of person being tracked %d", m_CurrPersonId);
  debug(buf);
  m_Mutex.unlock();
}

void DisplayNavInPB::newNavGraphNode(const cdl::WorkingMemoryChange &objID)
{
  debug("new NavGraphNode");

  if (!m_PeekabotClient.is_connected()) return;

  shared_ptr<CASTData<NavData::FNode> > oobj =
    getWorkingMemoryEntry<NavData::FNode>(objID.address);
  
  NavData::FNodePtr fnode = oobj->getData();

  m_Mutex.lock();
  m_PeekabotClient.begin_bundle();
  
  std::map<long,Node>::iterator n = m_Nodes.find(fnode->nodeId);

  if (n == m_Nodes.end()) {  // Node does not exist
    debug("Node %d new", (int)fnode->nodeId);

    DisplayNavInPB::Node node;
    node.m_Id = fnode->nodeId;
    node.m_Gateway = (fnode->gateway != 0);
    node.m_X = fnode->x;
    node.m_Y = fnode->y;
    node.m_areaId = fnode->areaId;
    if (!fnode->type.empty()) node.m_AreaClassNo = fnode->type[0].id;
    else node.m_AreaClassNo = -1;
    m_Nodes.insert(std::make_pair(node.m_Id, node));
    
    peekabot::SphereProxy sp; // Create Root Sphere Proxy
    char name[32];
    sprintf(name, "node%ld", (long)fnode->nodeId);
    sp.add(m_ProxyNodes, name);
    sp.set_position(fnode->x, fnode->y, 0);
    
    float r,g,b;
    if (fnode->gateway) {
        // add gateway sphere
        sp.set_scale(0.2, 0.2, 0.05);
        getColorByIndex(1, r, g, b);
      
        double width = 1;
        if (!fnode->width.empty()) width = fnode->width[0];
            addDoorpost(fnode->x, fnode->y, fnode->theta, width, sp);
      
        debug("Added gateway with id %d", fnode->nodeId);
    } else {
#ifdef DISPLAY_COMA
        r = 0;
        g = 0;
        b = 0;
        // set root sphere size zero
        sp.set_scale(0, 0, 0);
        debug("Added normal node with id %d", fnode->nodeId);
#else
        getColorByIndex(3+(fnode->areaId%6), r, g, b);
        sp.set_scale(0.1, 0.1, 0.05);  
        debug("Added normal node with id %d", fnode->nodeId);
#endif
    }
    sp.set_color(r,g,b);

    
#ifdef DISPLAY_COMA
#else
    if (m_ShowNodeClass && !fnode->gateway) 
    {
        peekabot::CylinderProxy cp;
        cp.add(sp, "class", peekabot::REPLACE_ON_CONFLICT);
        cp.set_scale(0.04, 0.04, 0.16);
        cp.set_position(0,0,0.08);
        //FIXME
        //place::ColorMap::getColorForPlaceClass(fnode->areaTypeNo, r, g, b);
        getColorByIndex(3+(fnode->areaId%6), r, g, b);
        cp.set_color(r,g,b);

        peekabot::SphereProxy mp;
        mp.add(sp, "mushroom");
        mp.set_scale(0.08, 0.08, 0.05);
        mp.set_position(0, 0, 0.16);
        mp.set_color(r,g,b);
    }
#endif

    if (m_ShowAreaClass && !fnode->gateway) { 
	    // add translucent area circle
        peekabot::CylinderProxy acp;
        char name2[32];
        sprintf(name2, "node%ld/area_class", (long)fnode->nodeId);
        acp.add(m_ProxyNodes, name2, peekabot::REPLACE_ON_CONFLICT);
        acp.set_scale(0.5, 0.5, 0.0);
        acp.set_position(0,0,0);
        acp.set_opacity(0.3);
        //FIXME
        //place::ColorMap::getColorForPlaceClass(fnode->areaTypeNo, r, g, b);
#ifdef DISPLAY_COMA
        acp.set_color(0,0,0); // Node is not assigned to a comaRoom so colour it black
#else
        getColorByIndex(3+(fnode->areaId%6), r, g, b);
        acp.set_color(r,g,b);        
#endif
    }

  } else { // Node already exists
    log("Node %d already there, should be changed", fnode->nodeId);

    n->second.m_Id = fnode->nodeId;
    n->second.m_Gateway = (fnode->gateway != 0);
    n->second.m_X = fnode->x;
    n->second.m_Y = fnode->y;
    n->second.m_areaId = fnode->areaId;
    if (!fnode->type.empty()) 
      n->second.m_AreaClassNo = fnode->type[0].id;
    else 
      n->second.m_AreaClassNo = -1;

    peekabot::SphereProxy sp;  // Create root sphere proxy
    char name[32];
    sprintf(name, "node%ld", (long)fnode->nodeId);
    sp.add(m_ProxyNodes, name, peekabot::REPLACE_ON_CONFLICT);
    sp.set_position(fnode->x, fnode->y, 0);

    float r,g,b;
    if (fnode->gateway) {
        sp.set_scale(0.2, 0.2, 0.05);
        getColorByIndex(1, r, g, b);

        double width = 1;
        if (!fnode->width.empty()) width = fnode->width[0];
            addDoorpost(fnode->x, fnode->y, fnode->theta, width, sp);

        debug("modified gateway with id %d", fnode->nodeId);
    } else { 
#ifdef DISPLAY_COMA
        // Do not show root sphere for non-doorways
        r= 0; g=0; b=0;
        sp.set_scale(0, 0, 0);
        debug("modified normal node with id %d", fnode->nodeId);
#else
        getColorByIndex(3+(fnode->areaId%6), r, g, b);
        sp.set_scale(0.1, 0.1, 0.05);
        debug("Added normal node with id %d", fnode->nodeId);
#endif
    }
    sp.set_color(r,g,b);    

#ifdef DISPLAY_COMA
#else
    if (m_ShowNodeClass && !fnode->gateway)
    {
        peekabot::CylinderProxy cp;
        cp.add(sp, "class", peekabot::REPLACE_ON_CONFLICT);
        cp.set_scale(0.04, 0.04, 0.16);
        cp.set_position(0,0,0.08);
        //FIXME
	    //place::ColorMap::getColorForPlaceClass(fnode->areaTypeNo, r, g, b);
        getColorByIndex(3+(fnode->areaId%6), r, g, b);
        cp.set_color(r,g,b);

        peekabot::SphereProxy mp;
        mp.add(sp, "mushroom");
        mp.set_scale(0.08, 0.08, 0.05);
        mp.set_position(0, 0, 0.16);
        mp.set_color(r,g,b);
    }
#endif
    if (m_ShowAreaClass && !fnode->gateway) 
    {
        peekabot::CylinderProxy acp;
        char name2[32];
        sprintf(name2, "node%ld/area_class", (long)fnode->nodeId);
        acp.add(m_ProxyNodes, name2, peekabot::REPLACE_ON_CONFLICT);
        acp.set_scale(0.5, 0.5, 0.0);
        acp.set_position(0,0,0);
        acp.set_opacity(0.3);
        //FIXME
        //place::ColorMap::getColorForPlaceClass(fnode->areaTypeNo, r, g, b);
        //getColorByIndex(3+(fnode->areaId%6), r, g, b);
        acp.set_color(r,g,b);
    }

    // We also redraw the edge just to make sure that it is drawn
    // correctly if the position of the node changed.
    redisplayEdgesToNode(n->second);
  }

  for (std::list< std::pair<long,long> >::iterator ei = m_NewEdges.begin();
       ei != m_NewEdges.end();) {

    // Check if the nodes are there now
    std::map<long,Node>::iterator n1 = m_Nodes.find(ei->first);
    std::map<long,Node>::iterator n2 = m_Nodes.find(ei->second);

    if (n1 != m_Nodes.end() && n2 != m_Nodes.end()) { // Found nodes

      displayEdge(n1->second, n2->second);

      addEdgeToList(n1->second.m_Id, n2->second.m_Id);

      ei = m_NewEdges.erase(ei);

    } else {
      ei++;
    }
  }

  m_PeekabotClient.end_bundle();
  m_Mutex.unlock();
}

void DisplayNavInPB::addDoorpost(double x, double y, double theta, 
                                 double width, 
                                 peekabot::SphereProxy &node)
{                                 
  peekabot::CubeProxy cpL;
  cpL.add(node, "doorpostleft");
  cpL.set_scale(0.1, 0.1, 2.0);
  cpL.set_pose(0.5*width*cos(theta), 0.5*width*sin(theta), 1.0, 
               theta, 0, 0);
  cpL.set_color(1.0, 0.817, 0.269);

  peekabot::CubeProxy cpR;
  cpR.add(node, "doorpostright");
  cpR.set_scale(0.1, 0.1, 2.0);
  cpR.set_pose(0.5*width*cos(theta+M_PI), 0.5*width*sin(theta+M_PI), 1.0,
               theta, 0, 0);
  cpR.set_color(1.0, 0.817, 0.269);
  
  peekabot::CubeProxy cpT;
  cpT.add(node, "doorposttop");
  cpT.set_scale(width+0.1, 0.1, 0.1);
  cpT.set_pose(0, 0, 2.0, theta, 0, 0);
  cpT.set_color(1.0, 0.817, 0.269);
}

void DisplayNavInPB::newNavGraphEdge(const cdl::WorkingMemoryChange &objID)
{
  debug("new NavGraphEdge");

  if (!m_PeekabotClient.is_connected()) return;

  shared_ptr<CASTData<NavData::AEdge> > oobj =
    getWorkingMemoryEntry<NavData::AEdge>(objID.address);
  
  NavData::AEdgePtr aedge = oobj->getData();

  m_Mutex.lock();

  m_PeekabotClient.begin_bundle();

  // Check if the nodes are there now
  std::map<long,Node>::iterator n1 = m_Nodes.find(aedge->startNodeId);
  std::map<long,Node>::iterator n2 = m_Nodes.find(aedge->endNodeId);

  if (n1 != m_Nodes.end() && n2 != m_Nodes.end()) { // Found nodes

    displayEdge(n1->second, n2->second);

    addEdgeToList(n1->second.m_Id, n2->second.m_Id);

  } else {
    m_NewEdges.push_back(std::make_pair(aedge->startNodeId, 
                                        aedge->endNodeId));
  }

  debug("Got a new edge connecting nodes %d and %d",
        aedge->startNodeId, aedge->endNodeId);

  m_PeekabotClient.end_bundle();

  m_Mutex.unlock();
}

void DisplayNavInPB::getColorByIndex(int id, float &r, float &g, float &b)
{
  //log("Getting color %d",id);

  switch (id) {
  case 0: // Green
    r = 1.0/0xFF*0x00;
    g = 1.0/0xFF*0xFF;
    b = 1.0/0xFF*0x00;
    break;
  case 1: // Red
    r = 1.0/0xFF*0xFF; 
    g = 1.0/0xFF*0x00;
    b = 1.0/0xFF*0x00;
    break;
  case 2: // Blue
    r = 1.0/0xFF*0x00;
    g = 1.0/0xFF*0x00;
    b = 1.0/0xFF*0xFF;
    break;
  case 3: 
    r = 1.0/0xFF*0xFF;
    g = 1.0/0xFF*0xFF;
    b = 1.0/0xFF*0x00;
    break;
  case 4:
    r = 1.0/0xFF*0x00; 
    g = 1.0/0xFF*0xFF;
    b = 1.0/0xFF*0xFF;
    break;
  case 5:
    r = 1.0/0xFF*0xFF;
    g = 1.0/0xFF*0x00;
    b = 1.0/0xFF*0xFF;
    break;
  case 6:
    r = 1.0/0xFF*0x00; 
    g = 1.0/0xFF*0x00;
    b = 1.0/0xFF*0x00;
    break;
  case 7:
    r = 1.0/0xFF*0xFF;
    g = 1.0/0xFF*0x24;
    b = 1.0/0xFF*0x00;
    break;
  case 8:
    r = 1.0/0xFF*0x6F;
    g = 1.0/0xFF*0x42;
    b = 1.0/0xFF*0x42;
    break;
  case 9:
    r = 1.0/0xFF*0x8C;
    g = 1.0/0xFF*0x17;
    b = 1.0/0xFF*0x17;
    break;
  case 10:
    r = 1.0/0xFF*0x5C; 
    g = 1.0/0xFF*0x33;
    b = 1.0/0xFF*0x17;
    break;
  case 11:// If more colours are added change MAX_COLORS below
    r = 1.0/0xFF*0x2F; 
    g = 1.0/0xFF*0x4F;
    b = 1.0/0xFF*0x2F;
    break;
  default: // If more colours are added change
    const int MAX_COLORS = 11;
    const int useColor = id % MAX_COLORS;
    debug("Only handles color with indices 0-%d, not %d, reusing colour %d", MAX_COLORS, id, useColor);
    getColorByIndex(useColor, r, g, b);
  }
}

void DisplayNavInPB::addEdgeToList(long id1, long id2)
{
  if (id1 > id2) {
    m_Edges.push_back( std::make_pair(id1, id2) );
  } else if (id2 > id1) {
    m_Edges.push_back( std::make_pair(id2, id1) );    
  } else {
    log("WARNING: Trying to connect node with id %d with itself", id1);
  }
}

void DisplayNavInPB::displayEdge(const DisplayNavInPB::Node &node1,
                                 const DisplayNavInPB::Node &node2)
{
  peekabot::LineCloudProxy lp;
  char name[32];
  if (node1.m_Id > node2.m_Id) {
    sprintf(name, "edge%06ld", node1.m_Id*1000+node2.m_Id);
  } else {
    sprintf(name, "edge%06ld", node2.m_Id*1000+node1.m_Id);
  }

  debug("Adding edge between (%f,%f) %ld and (%f,%f) %ld", 
      node1.m_X, node1.m_Y, node1.m_Id, 
      node2.m_X, node2.m_Y, node2.m_Id);

  lp.add(m_ProxyEdges, name, peekabot::REPLACE_ON_CONFLICT);
  lp.add_line(node1.m_X, node1.m_Y, 0,
              node2.m_X, node2.m_Y, 0);
  lp.set_color(0., 0., 0.);
  lp.set_opacity(1);
}

void DisplayNavInPB::redisplayEdgesToNode(const DisplayNavInPB::Node &node)
{
  std::map<long,Node>::iterator n;

  for (std::list< std::pair<long,long> >::iterator e = m_Edges.begin();
       e != m_Edges.end(); e++) {
    if (e->first == node.m_Id) {
      n = m_Nodes.find(e->second);
      if (n != m_Nodes.end()) {
        displayEdge(node, n->second);
        log("Redisplaying edge between nodes %ld and %ld", 
            node.m_Id, e->second);
      } else {
        log("WARNING: Did not find node %ld, couldn't redisplay edge from %ld",
            e->second, node.m_Id);
      }
    } else if (e->second == node.m_Id) {
      n = m_Nodes.find(e->first);
      if (n != m_Nodes.end()) {
        displayEdge(node, n->second);
        log("Redisplaying edge between nodes %ld and %ld", 
            node.m_Id, e->first);
      } else {
        log("WARNING: Did not find node %ld, couldn't redisplay edge from %ld",
            e->first, node.m_Id);
      }
    }
  }
}

void DisplayNavInPB::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again?) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort);

 //   m_PeekabotClient.assign(m_PeekabotClient, "root");    
    peekabot::Status s0,s1, s2, s3,s4;

    s0 = m_ProxyRobot.add(m_PeekabotClient, 
	m_PbRobotName,
	peekabot::REPLACE_ON_CONFLICT).status();
    if (s0.failed())
      log("failed to add robot");
    else
      log("added robot.");
    
      log("Loading robot file \"%s\"", 
              m_PbRobotFile.c_str());
    
   s1 = m_ProxyRobot.load_scene(m_PbRobotFile).status();
    if( s1.failed() ) {
      log("Could not load robot file \"%s\"", 
              m_PbRobotFile.c_str());
      peekabot::CubeProxy cube;
      cube.add(m_PeekabotClient, m_PbRobotName, peekabot::REPLACE_ON_CONFLICT);
      cube.set_scale(0.4, 0.3, 0.2);
      cube.set_position(0,0,0.1);
      cube.set_color(0,1,0);

      peekabot::CubeProxy nose;
      nose.add(cube, "nose", peekabot::REPLACE_ON_CONFLICT);
      nose.set_scale(0.2, 0.05, 0.05);
      nose.set_position(0.1, 0, 0.125);
      nose.set_color(1,0,0);

    } else {
      if (m_PbRobotFile == "CogXp3.xml" ||
	  m_PbRobotFile == "CogX_base_arm.xml" ||
	  m_PbRobotFile == "CogX_base.xml") {
        s2 = m_ProxyLaser.assign(m_ProxyRobot, "chassis/rangefinder").status();
        m_ScanAngFOV = M_PI/180.0*240;
        m_ScanMaxRange = 5.6;
		
		std::string path = "robot/chassis/superstructure/ptu/pan/tilt/baseline/cam_left";
		s4 = m_ProxyCam.assign(m_PeekabotClient, path).status();
		if(s4.failed()){
			log("cam proxy failed.");
		}
		
		s4 = m_ProxyPan.assign(m_PeekabotClient, "robot/chassis/superstructure/ptu/pan").status();
		if(s4.failed()){
			log("cam proxy failed.");
		}
		s4 = m_ProxyTilt.assign(m_PeekabotClient, "robot/chassis/superstructure/ptu/pan/tilt").status();
		if(s4.failed()){
			log("cam proxy failed.");
		}
		m_ProxyPan.set_dof(0);
		m_ProxyTilt.set_dof(30*M_PI/180.0);
		
		
      } else if (m_PbRobotFile == "B21.xml") {
        s2 = m_ProxyLaser.assign(m_ProxyRobot, "model/rangefinder").status();
        m_ScanAngFOV = M_PI/180.0*180;
        m_ScanMaxRange = 8.0;
      } else {
        s2 = m_ProxyLaser.assign(m_ProxyRobot, "peoplebot_base/rangefinder").status();
        m_ScanAngFOV = M_PI/180.0*180;
        m_ScanMaxRange = 8.0;
      }
      if( s2.failed() ) {
        log("Could not hook up to laser scanner, not using laser");
        m_LaserConnected = false;        
      } else {

        m_ProxyScan.add(m_ProxyLaser, "scan", peekabot::REPLACE_ON_CONFLICT);
        m_ProxyScan.set_color(0,0,1);
        m_LaserConnected = true;

      }
    }
 
    m_ProxyGraph.add(m_PeekabotClient,
                     "graph",
                     peekabot::REPLACE_ON_CONFLICT);

    m_ProxyNodes.add(m_ProxyGraph,
                     "nodes",
                     peekabot::REPLACE_ON_CONFLICT);

    m_ProxyEdges.add(m_ProxyGraph,
                     "edges",
                     peekabot::REPLACE_ON_CONFLICT);

    m_ProxyObjects.add(m_ProxyGraph,
                       "objects",
                       peekabot::REPLACE_ON_CONFLICT);
                       
    m_ProxyObjectLabels.add(m_ProxyGraph,
                       "labels",
                       peekabot::REPLACE_ON_CONFLICT);
    m_ProxyViewPoints.add(m_PeekabotClient, "viewpoints",peekabot::REPLACE_ON_CONFLICT);

    createRobotFOV();
    log("Connection to Peekabot established");

  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}

Cure::Transformation3D DisplayNavInPB::getCameraToWorldTransform()
{
  //Get camera ptz from PTZServer
  Cure::Transformation3D cameraRotation;
  if (m_PTUServer != 0) {
    ptz::PTZReading reading = m_PTUServer->getPose();

    double angles[] = {reading.pose.pan, -reading.pose.tilt, 0.0};
//    double angles[] = {0.0, M_PI/4, 0.0};
    cameraRotation.setAngles(angles);
  }
//
//  //Get camera position on robot from Cure
//  m_CameraPoseR;

  //Additional transform of ptz base frame
  Cure::Transformation3D ptzBaseTransform;
//  double nR[] = {0.0, 0.0, 1.0, 
//    1.0, 0.0, 0.0,
//    0.0, 1.0, 0.0};
  double camAngles[] = {-M_PI/2, 0.0, -M_PI/2};
  ptzBaseTransform.setAngles(camAngles);

  //Get robot pose
  Cure::Transformation2D robotTransform;
  if (m_RobotPose != 0) {
    robotTransform.setXYTheta(m_RobotPose->x, m_RobotPose->y,
      m_RobotPose->theta);
  }
  Cure::Transformation3D robotTransform3 = robotTransform;
  double tmp[6];
  robotTransform3.getCoordinates(tmp);
  log("robot transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);
  cameraRotation.getCoordinates(tmp);
  log("ptz transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);
  m_CameraPoseR.getCoordinates(tmp);
  log("cam transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);

  Cure::Transformation3D cameraOnRobot = m_CameraPoseR + cameraRotation + ptzBaseTransform ;
  return robotTransform3 + cameraOnRobot;
} 

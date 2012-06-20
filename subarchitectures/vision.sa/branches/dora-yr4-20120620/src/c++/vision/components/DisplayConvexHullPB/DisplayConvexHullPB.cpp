//
// = FILENAME
//    DisplayConvexHullPB.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    
//	
//
// 
//    
//
/*----------------------------------------------------------------------*/

#include <list>
#include <string>
#include "DisplayConvexHullPB.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VisionData.hpp>
#include <AddressBank/ConfigFileReader.hh>
#include <Transformation/Transformation3D.hh>

using namespace std;
using namespace cast;
using namespace VisionData;
using namespace boost;
/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new DisplayConvexHullPB();
  }
}

DisplayConvexHullPB::DisplayConvexHullPB() {
  previouscenter.assign(3,0.0);
  m_ptzInterface = 0;
}

DisplayConvexHullPB::~DisplayConvexHullPB() 
{
}

void DisplayConvexHullPB::configure(const map<string,string>& _config) 
{
  log("configure entered");
  //CURE config (for camera position)
  map<string,string>::const_iterator it = _config.find("-c");
  if (it!= _config.end()) {
    std::string configfile = it->second;

    Cure::ConfigFileReader cfg;
    if (cfg.init(configfile)) {
      println("configure(...) Failed to open with \"%s\"\n",
	  configfile.c_str());
      std::abort();
    }  

    if (cfg.getSensorPose(2, m_CameraPoseR)) {
      println("configure(...) Failed to get sensor pose");
      std::abort();
    } 
    double coords[6];
    m_CameraPoseR.getCoordinates(coords);
    log("m_CameraPoseR = (%f, %f, %f)", coords[0], coords[1], coords[2]);
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

  m_PbPort = 5050;
  m_PbHost = "localhost";

  if(_config.find("--pb-host") != _config.end()){
    std::istringstream str(_config.find("--pb-host")->second);
    str >> m_PbHost;
  }

  m_PTZServer = "";
  if(_config.find("--ptz-server") != _config.end()){
    std::istringstream str(_config.find("--ptz-server")->second);
    str >> m_PTZServer;
  }

  connectPeekabot();  
  log("configure done");
}

void DisplayConvexHullPB::start() {

  addChangeFilter(createChangeFilter<VisionData::ConvexHull>(cdl::ADD,
	"",
	"",
	"vision.sa",
	cdl::ALLSA),
      new MemberFunctionChangeReceiver<DisplayConvexHullPB>(this,
	&DisplayConvexHullPB::newConvexHull));
  addChangeFilter(createChangeFilter<VisionData::ConvexHull>(cdl::OVERWRITE,
	"",
	"",
	"vision.sa",
	cdl::ALLSA),
      new MemberFunctionChangeReceiver<DisplayConvexHullPB>(this,
	&DisplayConvexHullPB::newConvexHull));

  addChangeFilter(createGlobalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayConvexHullPB>(this,
	&DisplayConvexHullPB::robotPoseChanged));

  addChangeFilter(createGlobalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayConvexHullPB>(this,
	&DisplayConvexHullPB::robotPoseChanged));  

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
    m_ptzInterface = ptz::PTZInterfacePrx::uncheckedCast(base);
//  if (m_PTZServer != "") {
//    m_ptzInterface = getIceServer<ptz::PTZInterface>(m_PTZServer);
//  }
//  log("LocalMapManager started");

  log("start done");  
}


void DisplayConvexHullPB::runComponent() {

  log("runComponent");


  log("Connected to the laser");

  while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
    sleep(m_RetryDelay);
    connectPeekabot();
  }

  log("Connected to peekabot, ready to go");
  if (m_PeekabotClient.is_connected()) {
    while (isRunning()) {
      usleep(250000);
    }
  }
}

void DisplayConvexHullPB::newConvexHull(const cdl::WorkingMemoryChange
				   &objID){

  debug("newConvexHull called");
  shared_ptr<CASTData<VisionData::ConvexHull> > oobj =
 getWorkingMemoryEntry<VisionData::ConvexHull>(objID.address);
  m_Mutex.lock();
  VisionData::ConvexHullPtr m_ConvexHull = oobj->getData();

  log("Convex hull center at %f, %f, %f", m_ConvexHull->center.pos.x,m_ConvexHull->center.pos.y,m_ConvexHull->center.pos.z);
  Cure::Transformation3D cam2WorldTrans =
    getCameraToWorldTransform();
  double tmp[6];
  cam2WorldTrans.getCoordinates(tmp);
  log("total transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);

  //Convert hull to world coords
  for (unsigned int i = 0; i < m_ConvexHull->PointsSeq.size(); i++) {
    Cure::Vector3D from(m_ConvexHull->PointsSeq[i].x, m_ConvexHull->PointsSeq[i].y, m_ConvexHull->PointsSeq[i].z);
    Cure::Vector3D to;
    cam2WorldTrans.invTransform(from, to);
    log("vertex at %f, %f, %f", m_ConvexHull->PointsSeq[i].x, m_ConvexHull->PointsSeq[i].y, m_ConvexHull->PointsSeq[i].z);
    m_ConvexHull->PointsSeq[i].x = to.X[0];
    m_ConvexHull->PointsSeq[i].y = to.X[1];
    m_ConvexHull->PointsSeq[i].z = to.X[2];
    log("Transformed vertex at %f, %f, %f", to.X[0], to.X[1], to.X[2]);
  }
  Cure::Vector3D from(m_ConvexHull->center.pos.x, m_ConvexHull->center.pos.y, m_ConvexHull->center.pos.z);
  Cure::Vector3D to;
  cam2WorldTrans.invTransform(from, to);
  m_ConvexHull->center.pos.x = to.X[0];
  m_ConvexHull->center.pos.y = to.X[1];
  m_ConvexHull->center.pos.z = to.X[2];
  cam2WorldTrans.getCoordinates(tmp);


  if (previouscenter.at(0) == 0.0)
    {
      previouscenter.at(0) = m_ConvexHull->center.pos.x;
      previouscenter.at(1) = m_ConvexHull->center.pos.y;
      previouscenter.at(2) = m_ConvexHull->center.pos.z;
      
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
	sqrt((previouscenter.at(0)-m_ConvexHull->center.pos.x)*(previouscenter.at(0)-m_ConvexHull->center.pos.x)+(previouscenter.at(1)-m_ConvexHull->center.pos.y)*(previouscenter.at(1)-m_ConvexHull->center.pos.y)+(previouscenter.at(2)-m_ConvexHull->center.pos.z)*(previouscenter.at(2)-m_ConvexHull->center.pos.z));
      
      if (dist > 0.1*m_ConvexHull->radius)
	
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
}


void DisplayConvexHullPB::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again?) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort, true);

  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}

void DisplayConvexHullPB::robotPoseChanged(const cdl::WorkingMemoryChange &objID) 
{
  try {
    lastRobotPose =
      getMemoryEntry<NavData::RobotPose2d>(objID.address);
  }
  catch (DoesNotExistOnWMException e) {
    log("Error! robotPose missing on WM!");
  }
}
  
Cure::Transformation3D DisplayConvexHullPB::getCameraToWorldTransform()
{
  //Get camera ptz from PTZServer
  Cure::Transformation3D cameraRotation;
  if (m_ptzInterface != 0) {
    ptz::PTZReading reading = m_ptzInterface->getPose();

    double angles[] = {reading.pose.pan, -reading.pose.tilt, 0.0};
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
  if (lastRobotPose != 0) {
    robotTransform.setXYTheta(lastRobotPose->x, lastRobotPose->y,
      lastRobotPose->theta);
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

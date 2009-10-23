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
}

DisplayConvexHullPB::~DisplayConvexHullPB() 
{
}

void DisplayConvexHullPB::configure(const map<string,string>& _config) 
{
  log("configure entered");
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


/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <../../VisionUtils.h>
#include "GazeboVision.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::GazeboVision();
  }
}

namespace cast
{

using namespace std;
using namespace PlayerCc;
using namespace VisionData;

GazeboVision::GazeboVision()
{
  robot = 0;
  sim = 0;
  playerHost = PlayerCc::PLAYER_HOSTNAME;
  playerPort = PlayerCc::PLAYER_PORTNUM;
}

void GazeboVision::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--playerhost")) != _config.end())
  {
    playerHost = it->second;
  }
  if((it = _config.find("--playerport")) != _config.end())
  {
    istringstream str(it->second);
    str >> playerPort;
  }

  if((it = _config.find("--labels")) != _config.end())
  {
    istringstream istr(it->second);
    string label;
    while(istr >> label)
      labels.push_back(label);

    ostringstream ostr;
    for(size_t i = 0; i < labels.size(); i++)
      ostr << " '" << labels[i] << "'";
    log("detecting objects: %s", ostr.str().c_str());
  }
  objAddrs.resize(labels.size());
}

void GazeboVision::start()
{
  robot = new PlayerCc::PlayerClient(playerHost, playerPort);
  sim = new PlayerCc::SimulationProxy(robot, 0);
  ostringstream s;
  s << "connected to player robot '" << robot << "'\n";
  log(s.str());
}

void GazeboVision::destroy()
{
  delete sim;
  delete robot;
}

/**
 * NOTE: This is hackishly only implemented for table objects.
 */
bool GazeboVision::newObject(string &label, cogx::Math::Pose3 &pose, VisualObjectPtr &obj)
{
  if(label.find("table") == string::npos)
  {
    error("NOT IMPLEMENTED: can not get object '%s', only tables (objects with label 'table1, table2, ..) are supported",
      label.c_str());
    return false;
  }

	// geometry model
  obj->model = new GeometryModel;

	VisionData::Face face1, face2;
	double tableSizeX = 0.6;
	double tableSizeY = 0.6;
	double tableSizeZ = 0.5;
  Vertex v;

  v.pos = cogx::Math::vector3(tableSizeX/2., tableSizeY/2., tableSizeZ);
  v.normal = cogx::Math::vector3(0., 0., 1.);
  obj->model->vertices.push_back(v);
  // fill one face clockwise, the other counter-clockwise
  face1.vertices.push_back(0);
  face2.vertices.push_back(3);

  v.pos = cogx::Math::vector3(-tableSizeX/2., tableSizeY/2., tableSizeZ);
  v.normal = cogx::Math::vector3(0., 0., 1.);
  obj->model->vertices.push_back(v);
  face1.vertices.push_back(1);
  face2.vertices.push_back(2);

  v.pos = cogx::Math::vector3(-tableSizeX/2., -tableSizeY/2., tableSizeZ);
  v.normal = cogx::Math::vector3(0., 0., 1.);
  obj->model->vertices.push_back(v);
  face1.vertices.push_back(3);
  face2.vertices.push_back(1);

  v.pos = cogx::Math::vector3(tableSizeX/2., -tableSizeY/2., tableSizeZ);
  v.normal = cogx::Math::vector3(0., 0., 1.);
  obj->model->vertices.push_back(v);
  face1.vertices.push_back(3);
  face2.vertices.push_back(0);

	obj->model->faces.push_back(face1);
	obj->model->faces.push_back(face2);
  computeNormalsFromFaces(obj->model);

  // create a very simple distribution: label and unknown
  obj->identLabels.push_back(label);
  obj->identLabels.push_back("unknown");
  // note: distribution must of course sum to 1
  obj->identDistrib.push_back(1.);
  obj->identDistrib.push_back(0.);
  // the information gain if we know the label, just set to 1, cause we don't
  // have any alternative thing to do
  obj->identGain = 1.;
  // ambiguity in the distribution: we use the distribution's entropy
  obj->identAmbiguity = 0.;
  for(size_t i = 0; i < obj->identDistrib.size(); i++)
    if(fpclassify(obj->identDistrib[i]) != FP_ZERO)
      obj->identAmbiguity -= obj->identDistrib[i]*::log(obj->identDistrib[i]);
  obj->pose = pose;
  obj->componentID = getComponentID();

  return true;
}

void GazeboVision::runComponent()
{
  while(isRunning())
  {
    for(size_t i = 0; i < labels.size(); i++)
    {
      double x, y, z, roll, pitch, yaw, time;

      sim->GetPose3d((char*)labels[i].c_str(), x, y, z, roll, pitch, yaw, time);
        VisualObjectPtr obj;
      cogx::Math::Pose3 pose;
      pose.pos = cogx::Math::vector3(x, y, z);
      fromRPY(pose.rot, roll, pitch, yaw);
      if(objAddrs[i].empty())
      {
        objAddrs[i] = newDataID();
        VisualObjectPtr obj = new VisualObject;
        if(newObject(labels[i], pose, obj))
          addToWorkingMemory(objAddrs[i], obj);
        else
          objAddrs[i].clear();
      }
      else
      {
        obj = getMemoryEntry<VisualObject>(objAddrs[i]);
        obj->pose = pose;
        overwriteWorkingMemory(objAddrs[i], obj);
      }
      debug("%s Pose3d: XYZ[%f %f %f] RPY[%f %f %f]\n", labels[i].c_str(), x, y, z, roll, pitch, yaw); 
    }
    sleepComponent(100);
  }
}

}


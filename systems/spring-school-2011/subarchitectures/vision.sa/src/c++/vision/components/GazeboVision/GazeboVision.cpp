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
using namespace cogx::Math;

const string GazeboVision::robotName = "robot";

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

  if((it = _config.find("--sizes")) != _config.end())
  {
    istringstream istr(it->second);
    Vector3 size;
    while(istr >> size)
      sizes.push_back(size);
  }

  if(labels.size() != sizes.size())
    throw runtime_error("number of labels must match number of sizes");

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
 * Create a new visual object with given pose.
 * TODO: create proper box.
 */
bool GazeboVision::newObject(string &label, cogx::Math::Pose3 &pose, VisualObjectPtr &obj)
{
	// geometry model
  obj->model = new GeometryModel;

	size_t i = find(labels.begin(), labels.end(), label) - labels.begin();
	Vector3 size = sizes[i];
  Vertex v;
	VisionData::Face face;

  v.pos = cogx::Math::vector3(size.x/2., size.y/2., -size.z/2.);
  obj->model->vertices.push_back(v);
  v.pos = cogx::Math::vector3(-size.x/2., size.y/2., -size.z/2.);
  obj->model->vertices.push_back(v);
  v.pos = cogx::Math::vector3(-size.x/2., -size.y/2., -size.z/2.);
  obj->model->vertices.push_back(v);
  v.pos = cogx::Math::vector3(size.x/2., -size.y/2., -size.z/2.);
  obj->model->vertices.push_back(v);

  v.pos = cogx::Math::vector3(size.x/2., size.y/2., size.z/2.);
  obj->model->vertices.push_back(v);
  v.pos = cogx::Math::vector3(-size.x/2., size.y/2., size.z/2.);
  obj->model->vertices.push_back(v);
  v.pos = cogx::Math::vector3(-size.x/2., -size.y/2., size.z/2.);
  obj->model->vertices.push_back(v);
  v.pos = cogx::Math::vector3(size.x/2., -size.y/2., size.z/2.);
  obj->model->vertices.push_back(v);

  face.vertices.clear();
  face.vertices.push_back(0);
  face.vertices.push_back(1);
  face.vertices.push_back(2);
  face.vertices.push_back(3);
  obj->model->faces.push_back(face);
  face.vertices.clear();
  face.vertices.push_back(3);
  face.vertices.push_back(7);
  face.vertices.push_back(4);
  face.vertices.push_back(0);
  obj->model->faces.push_back(face);
  face.vertices.clear();
  face.vertices.push_back(0);
  face.vertices.push_back(4);
  face.vertices.push_back(5);
  face.vertices.push_back(1);
  obj->model->faces.push_back(face);
  face.vertices.clear();
  face.vertices.push_back(1);
  face.vertices.push_back(5);
  face.vertices.push_back(6);
  face.vertices.push_back(2);
  obj->model->faces.push_back(face);
  face.vertices.clear();
  face.vertices.push_back(2);
  face.vertices.push_back(6);
  face.vertices.push_back(7);
  face.vertices.push_back(3);
  obj->model->faces.push_back(face);
  face.vertices.clear();
  face.vertices.push_back(7);
  face.vertices.push_back(6);
  face.vertices.push_back(5);
  face.vertices.push_back(4);
  obj->model->faces.push_back(face);

 /* v.pos = cogx::Math::vector3(size.x/2., size.y/2., size.z/2.);
  v.normal = cogx::Math::vector3(0., 0., 1.);
  v.pos = transform(pose, v.pos);
  v.normal = transformDirection(pose, v.normal);
  obj->model->vertices.push_back(v);
  // fill one face clockwise, the other counter-clockwise
  face1.vertices.push_back(0);
  face2.vertices.push_back(3);

  v.pos = cogx::Math::vector3(-size.x/2., size.y/2., size.z/2.);
  v.normal = cogx::Math::vector3(0., 0., 1.);
  v.pos = transform(pose, v.pos);
  v.normal = transformDirection(pose, v.normal);
  obj->model->vertices.push_back(v);
  face1.vertices.push_back(1);
  face2.vertices.push_back(2);

  v.pos = cogx::Math::vector3(-size.x/2., -size.y/2., size.z/2.);
  v.normal = cogx::Math::vector3(0., 0., 1.);
  v.pos = transform(pose, v.pos);
  v.normal = transformDirection(pose, v.normal);
  obj->model->vertices.push_back(v);
  face1.vertices.push_back(3);
  face2.vertices.push_back(1);

  v.pos = cogx::Math::vector3(size.x/2., -size.y/2., size.z/2.);
  v.normal = cogx::Math::vector3(0., 0., 1.);
  v.pos = transform(pose, v.pos);
  v.normal = transformDirection(pose, v.normal);
  obj->model->vertices.push_back(v);
  face1.vertices.push_back(3);
  face2.vertices.push_back(0);

	obj->model->faces.push_back(face1);
	obj->model->faces.push_back(face2);*/

  computeNormalsFromFaces(obj->model);

  // create a very simple distribution: label and unknown
  obj->identLabels.push_back(label);
  obj->identLabels.push_back("unknown");
  // note: distribution must of course sum to 1
  obj->identDistrib.push_back(1.);
  obj->identDistrib.push_back(0.);

	// NOTE: for now use a stupid fixed probability
	if(label.find("table") != string::npos)
    obj->shapeLabels.push_back("plane");
  else
    obj->shapeLabels.push_back("box");
	obj->shapeLabels.push_back("unknown");
	obj->shapeDistrib.push_back(0.9);
	obj->shapeDistrib.push_back(0.1);

  obj->pose = pose;
  obj->componentID = getComponentID();

  return true;
}

void GazeboVision::runComponent()
{
  while(isRunning())
  {
    double x, y, z, roll, pitch, yaw, time;
    Pose3 robotPose;

    sim->GetPose3d((char*)robotName.c_str(), x, y, z, roll, pitch, yaw, time);
    robotPose.pos = cogx::Math::vector3(x, y, z);
    fromRPY(robotPose.rot, roll, pitch, yaw);
    log(robotName + " pose " + toString(robotPose));

    for(size_t i = 0; i < labels.size(); i++)
    {
      VisualObjectPtr obj;
      Pose3 pose;
      
      sim->GetPose3d((char*)labels[i].c_str(), x, y, z, roll, pitch, yaw, time);
      pose.pos = cogx::Math::vector3(x, y, z);
      fromRPY(pose.rot, roll, pitch, yaw);
      log(labels[i] + " pose in world " + toString(pose));
      transformInverse(robotPose, pose, pose);
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
      log(labels[i] + " pose " + toString(pose));
    }
    sleepComponent(1000);
  }
}

}


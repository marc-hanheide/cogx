/**
 * @author Andreas Richtsfeld
 * @date April 2011
 * @brief Just receives point clouds and displays them on the TomGine.
 */

#include "ManipulationPlanner.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "cogxmath.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::ManipulationPlanner();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;

static double FIRST_GRIPPER_DISTANCE = 0.05;

/**
 * @brief Configure
 * @param _config Configuration
 */
void ManipulationPlanner::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;


}


/**
 * @brief start component
 */
void ManipulationPlanner::start()
{
    // add change filter for ProtoObject changes
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<ManipulationPlanner>(this, &ManipulationPlanner::newVisualObject));
}


/**
 * @brief runComponent
 */
void ManipulationPlanner::runComponent()
{

}

void ManipulationPlanner::newVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr newobj = getMemoryEntry< VisionData::VisualObject>(_wmc.address);
  Pose3 pose;
  calculateGripperPosition(newobj, pose);
  
}
void calculateGripperPosition(VisionData::VisualObjectPtr obj, Math::Pose3 &pose){
  /* TODO 1. Select which face to approach
              1.1 Eliminate top and bottom surfaces
              1.2 Eliminate two biggest surfaces
              1.3 Eliminate the surface furthest from us
         2.  Select which orientation to approach */ 
  
  bool goodFaces[6] = {1, 1, 1, 1, 1, 1};  
  Pose3 finalPose, firstPose;
  int bestFace;

  // 1.1
  for (unsigned int i = 0; i < obj->model->faces.size(); i++)
  {
    Vector3 v0 = obj->model->vertices[obj->model->faces[i].vertices[0]].pos;
    Vector3 v1 = obj->model->vertices[obj->model->faces[i].vertices[2]].pos;
    Vector3 normal = cross(v0, v1);
    
    if(normal.z < 0.01) // 1cm threshold
    {
      goodFaces[i] = false;
    }
  }
  
  // 1.2
  list<double> area;
  map<double,int> maparea;
  for (unsigned int i = 0; i < obj->model->faces.size(); i++)
  {
    if(goodFaces[i])
    {
       Vector3 v0 = obj->model->vertices[obj->model->faces[i].vertices[0]].pos;
       Vector3 v1 = obj->model->vertices[obj->model->faces[i].vertices[1]].pos;
       Vector3 v2 = obj->model->vertices[obj->model->faces[i].vertices[2]].pos;

       double len0 = length(v0 - v1);
       double len1 = length(v1 - v2);
      
       area.push_back(len0 * len1); 
       maparea.insert(make_pair<double, int>(len0*len1, i));
    }
  }  
  area.sort();
  list<double>::iterator it = area.begin();
  goodFaces[maparea[*it]] = false;
  it++;
  goodFaces[maparea[*it]] = false;

  // 1.3
  Vector3 pos[2];
  int posi = 0;
  int goodOnes[2];
  for (unsigned int i = 0; i < obj->model->faces.size(); i++)
  {
    if(goodFaces[i])
    {
      Vector3 v0 = obj->model->vertices[obj->model->faces[i].vertices[0]].pos;
      Vector3 v1 = obj->model->vertices[obj->model->faces[i].vertices[1]].pos;
      Vector3 v2 = obj->model->vertices[obj->model->faces[i].vertices[2]].pos;
      Vector3 v3 = obj->model->vertices[obj->model->faces[i].vertices[3]].pos;

      pos[posi] = (v0+v1+v2+v3)/4.;
      goodOnes[posi] = i;
      posi++;
    }
  }
  
  // check, which one is nearer to the robot?
  if (length(pos[0]) < length(pos[1]))
  {
    finalPose.pos = pos[0];
    bestFace = goodOnes[0];
  }  
  else
  {
    finalPose.pos = pos[1];
    bestFace = goodOnes[1];
  }
  
  Vector3 dir = finalPose.pos - obj->pose.pos;
  normalise(dir);
  firstPose.pos = finalPose.pos + dir * FIRST_GRIPPER_DISTANCE;
  
  /// TODO TODO TODO TODO 
  
//   firstPose.rot;
}
}










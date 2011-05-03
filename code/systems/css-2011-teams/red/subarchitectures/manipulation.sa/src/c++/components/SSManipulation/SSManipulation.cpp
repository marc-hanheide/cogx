/**
 * @author Andreas Richtsfeld
 * @date April 2011
 * @brief Manipulation for spring school system
 */

#include "SSManipulation.h"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "cogxmath.h"

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::SSManipulation();
  }
}

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;

static double FIRST_GRIPPER_DISTANCE = 0.05;    /// We define the distance of the gripper to the finalposition: 5 cm

/**
 * @brief Configure
 * @param _config Configuration
 */
void SSManipulation::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;
}


/**
 * @brief start component
 */
void SSManipulation::start()
{
  // add change filter for vision commands
  addChangeFilter(createLocalTypeFilter<manipulation::SSManipulationCommand>(cdl::ADD),
    new MemberFunctionChangeReceiver<SSManipulation>(this, &SSManipulation::receivedManipulationCommand));
}


/**
 * @brief runComponent
 */
void SSManipulation::runComponent()
{}

/**
 * @brief receivedManipulationCommand
 */
void SSManipulation::receivedManipulationCommand(const cdl::WorkingMemoryChange & _wmc)
{
  log("new manipulation command received");
  
  manipulation::SSManipulationCommandPtr command = getMemoryEntry<manipulation::SSManipulationCommand>(_wmc.address);
  
  if(command->cmd == manipulation::SSMGRAB)
  {
    cast::cdl::WorkingMemoryAddress addr;                                                                                                                           // = new cast::cdl::WorkingMemoryAddress();
    addr.subarchitecture = "vision.sa";
    addr.id = command->objID;
    VisionData::VisualObjectPtr newobj = getMemoryEntry<VisionData::VisualObject>(addr);
    
    Pose3 pose;
    calculateGripperPosition(newobj, pose);      
    WriteCommandToWM(_wmc.address, 1, true, addr.id);         /// we succeeded
  }
  WriteCommandToWM(_wmc.address, 0, false, "");               /// we failed
}

/**
 * @brief calculateGripperPosition
 */
void SSManipulation::calculateGripperPosition(VisionData::VisualObjectPtr obj, Math::Pose3 &pose)
{
  log("calculate gripper position started.");
  /* TODO 1. Select which face to approach
              1.1 Eliminate top and bottom surfaces
              1.2 Eliminate two biggest surfaces
              1.3 Eliminate the surface furthest from us
         2.  Select which orientation to approach */ 
  
  bool goodFaces[6] = {1, 1, 1, 1, 1, 1};  
  Pose3 finalPose, firstPose;
  int bestFace;

  firstPose.rot = obj->pose.rot;
  finalPose.rot = obj->pose.rot;

  // 1.1
  for (unsigned int i = 0; i < obj->model->faces.size(); i++)
  {
    Vector3 v0 = obj->model->vertices[obj->model->faces[i].vertices[0]].pos;
    Vector3 v1 = obj->model->vertices[obj->model->faces[i].vertices[1]].pos;
    Vector3 v2 = obj->model->vertices[obj->model->faces[i].vertices[2]].pos;
    Vector3 normal = cross(v0-v1, v1-v2);
    normalise(normal);
    double z = fabs(normal.z);
    
    if(z > 0.8) // 1cm threshold
    {
      goodFaces[i] = false;
      log("eliminating surface %d!", i);
    }
  }
  
  // 1.2
  list<double> area;
  map<int,double> maparea;
  for (unsigned int i = 0; i < obj->model->faces.size(); i++)
  {
    if(goodFaces[i])
    {
       log("eliminate 2 biggest areas");
       Vector3 v0 = obj->model->vertices[obj->model->faces[i].vertices[0]].pos;
       Vector3 v1 = obj->model->vertices[obj->model->faces[i].vertices[1]].pos;
       Vector3 v2 = obj->model->vertices[obj->model->faces[i].vertices[2]].pos;

       double len0 = length(v0 - v1);
       double len1 = length(v1 - v2);
       area.push_back(len0 * len1); 
       log("area for surface %d is: %4.2f", i,len0 * len1 );

       maparea.insert(make_pair<int, double>(i, len0*len1));
    }
  }  
 
 // find biggest area in list
 double maxarea = -1;
 int maxindex = -1;
 map<int,double>::iterator biggestit;
 for(map<int,double>::iterator it = maparea.begin(); it != maparea.end(); it++){
   if (it->second > maxarea){
     maxindex = it->first;
     maxarea = it->second;
     biggestit = it;
   }
 }
 
 log("First big surface index: %d with area: %4.2f", maxindex, maxarea);
 goodFaces[maxindex] = false;
 maparea.erase(biggestit);

 log("Now calculating second biggest area");
 maxarea = -1;
 maxindex = -1;
 for(map<int,double>::iterator it = maparea.begin(); it != maparea.end(); it++){
   if (it->second > maxarea){
     maxindex = it->first;
     maxarea = it->second;
   }
 }
 
 log("Second big surface index: %d with area: %4.2f", maxindex, maxarea);
 goodFaces[maxindex] = false;
 maparea.erase(maparea[maxindex]);

  // 1.3
  Vector3 pos[2];
  int posi = 0;
  int goodOnes[2];
  for (unsigned int i = 0; i < obj->model->faces.size(); i++)
  {
    if(goodFaces[i])
    {
      Vector3 v0 = obj->model->vertices[obj->model->faces[i].vertices[0]].pos + obj->pose.pos;
      Vector3 v1 = obj->model->vertices[obj->model->faces[i].vertices[1]].pos + obj->pose.pos;
      Vector3 v2 = obj->model->vertices[obj->model->faces[i].vertices[2]].pos + obj->pose.pos;
      Vector3 v3 = obj->model->vertices[obj->model->faces[i].vertices[3]].pos + obj->pose.pos;

      log("We have a pose!");
      pos[posi] = (v0+v1+v2+v3)/4.;
      log("pos: %4.2f / %4.2f / %4.2f", pos[posi].x, pos[posi].y, pos[posi].z);
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
  
  log("first pose:\n%s", toString(firstPose).c_str());
  log("final pose:\n%s", toString(finalPose).c_str());
  
  Vector3 z_coord; 
  z_coord.x = 0;
  z_coord.y = 0;
  z_coord.z = 1;
  
  Vector3 dir2 = cross(dir, z_coord);

  double col[9];
  col[0] = -dir2.x;
  col[1] = -dir2.y;
  col[2] = -dir2.z;
  col[3] = -dir.x;
  col[4] = -dir.y;
  col[5] = -dir.z;
  col[6] = 0;
  col[7] = 0;
  col[8] = 1;
  setColumn33(firstPose.rot, col);
  
  log("rotation matrix:\n%s", toString(firstPose.rot).c_str());

  VisionData::GripperPosePtr  grPose = new VisionData::GripperPose;
   grPose->initialPose = firstPose;
   grPose->finalPose = finalPose;
 
   Pose3 releasePose = finalPose;
   releasePose.pos.z += 0.10; // just list 10cm
   grPose->releasePose = releasePose;
   
  WriteGripperPositionToWM(grPose);
}

/**
 * @brief WriteGripperPositionToWM
 */
void SSManipulation::WriteGripperPositionToWM(VisionData::GripperPosePtr  grPose)
{
  gripperPoseID = newDataID();
  addToWorkingMemory(gripperPoseID, "manipulation.sa", grPose);
  sleepComponent(200);
  log("Added new gripper position to working memory: %s", gripperPoseID.c_str());
}


/**
 * @brief Send command back
 * cmd 0 ... SSMGRABEND
 */
void SSManipulation::WriteCommandToWM(cast::cdl::WorkingMemoryAddress addr, int cmd, bool succeed, std::string id)
{
  log("write command to wm: %i", cmd);
  
  if (cmd == 0) // Manipulation ended
  {
    manipulation::SSManipulationCommandPtr command = new manipulation::SSManipulationCommand;
    command->cmd = manipulation::SSMGRABEND;
    command->succeed = succeed;
    command->objID = id;    
    command->gripperPoseID = gripperPoseID;
    overwriteWorkingMemory(addr.id, command);
  }
  sleepComponent(200);

  log("wrote command to wm");
}

}










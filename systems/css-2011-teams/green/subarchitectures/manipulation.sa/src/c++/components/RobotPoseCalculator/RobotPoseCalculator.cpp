/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <string>
#include <cassert>
#include <iostream>
#include <cmath>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include "RobotPoseCalculator.h"

#include "Vector3.h"
#include "Matrix33.h"

using namespace std;

const double CONFIDENCE_THRESHOLD = 0.03;

const double MAX_GRASP_WIDTH = 0.05;
const double MAX_JOINT_HEIGHT = 0.8;
const double ROBOT_HAND_OFFSET = 0.2;
const double ROBOT_ARM_X_OFFSET = 0.0;
const double MIN_GRASP_DISTANCE = 0.2;
const double MAX_GRASP_DISTANCE = 0.6;
const double GRASP_DISTANCE = 0.4;
const double INIT_DISTANCE = 1.0;

const int label_count = 1;
const string labels[] = { "cereals1_model", "example-cereals-schokomusli", "cereals-bircher" };

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cogx::RobotPoseCalculator();
  }
}

namespace cogx
{

using namespace cast;
using namespace cast::cdl;
using namespace manipulation::slice;
using namespace NavData;
using namespace VisionData;
using namespace Math;

RobotPoseCalculator::RobotPoseCalculator()
{
    m_x = 0;
    m_y = 0;
    m_theta = 0;
// #ifdef FEAT_VISUALIZATION
//   display.setClientData(this);
// #endif
}

void RobotPoseCalculator::configure(const map<string, string> &_config)
{

}

void RobotPoseCalculator::start()
{
  addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::ADD),
                  new MemberFunctionChangeReceiver<RobotPoseCalculator>(this, &RobotPoseCalculator::receiveVisualObject));
  addChangeFilter(createGlobalTypeFilter<VisualObject>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<RobotPoseCalculator>(this, &RobotPoseCalculator::receiveVisualObject));

  addChangeFilter(createGlobalTypeFilter<NavData::RobotPose2d>(cdl::ADD),
                  new MemberFunctionChangeReceiver<RobotPoseCalculator>(this, &RobotPoseCalculator::receiveRobotPose));

  addChangeFilter(createGlobalTypeFilter<NavData::RobotPose2d>(cdl::OVERWRITE),
                  new MemberFunctionChangeReceiver<RobotPoseCalculator>(this, &RobotPoseCalculator::receiveRobotPose));

}

void RobotPoseCalculator::destroy()
{
}

void RobotPoseCalculator::receiveRobotPose(const cdl::WorkingMemoryChange &_wmc) {
  RobotPose2dPtr pose = getMemoryEntry<RobotPose2d>(_wmc.address);
  m_x = pose->x;
  m_y = pose->y;
  m_theta = pose->theta;
}

void RobotPoseCalculator::receiveVisualObject(const cdl::WorkingMemoryChange &_wmc)
{
  log("received VisualObject");
  VisualObjectPtr object = getMemoryEntry<VisualObject>(_wmc.address);

  string label = "";
  for (size_t i = 0; i < object->identLabels.size(); ++i) {
      if (object->identDistrib[i] > CONFIDENCE_THRESHOLD) {
          label = object->identLabels[i];
          break;
      }
  }

  bool acceptLabel = false;
  for (int i = 0; i < label_count; ++i) {
      if (label == labels[i]) {
          acceptLabel = true;
          break;
      }
  }
  if (!acceptLabel) 
      return;

  // Delete poses
  vector<CASTData<ManipulationPose> > poses;
  getMemoryEntriesWithData(poses);
  for(vector<CASTData<ManipulationPose> >::iterator it = poses.begin(); it != poses.end(); ++it) {
      if (it->getData()->label == label) {
          deleteFromWorkingMemory(it->getID());
      }
  }

  Vector3 pos = object->pose.pos;
  Matrix33 rot = object->pose.rot;

  if (isZero(getColumn(rot, 0))) {
      return;
  }

  cout << "robot position: " << m_x << " " << m_y << " / " << m_theta << endl;

  Vector3 dim = getObjectDimensions(*object);
  int short_axis = 0;
  int a1;
  int a2;
  if (dim.x <= dim.y) {
      if (dim.x <= dim.z)
          short_axis = 0; //x
      else
          short_axis = 2; //z
  }
  else {
      if (dim.y <= dim.z)
          short_axis = 1; //y
      else
          short_axis = 2; //z
  }
  if (short_axis == 0) {
      a1 = 1;
      a2 = 2;
  }
  else if (short_axis == 1) {
      a1 = 0;
      a2 = 2;
  }
  else if (short_axis == 2) {
      a1 = 0;
      a2 = 1;
  }

  std::cout << "center:" << pos << endl;
  std::cout << "rotation:" << getColumn(rot,0) << " " << getColumn(rot,1) << " " << getColumn(rot,2) << endl;
  std::cout << "dimensions:" << dim << endl;
  
  //Grasp vector is orthogonal to object x-axis
  Vector3 grasp_dir = cross(vector3(0.0, 0.0, 1.0), getColumn(rot, short_axis));
  std::cout << "grasp direction:" << grasp_dir << endl;
  vector<Vector3> grasp_offsets;
  grasp_offsets.push_back(getColumn(rot, a1) * get(dim)[a1]/2);
  grasp_offsets.push_back(-getColumn(rot, a1) * get(dim)[a1]/2);
  grasp_offsets.push_back(getColumn(rot, a2) * get(dim)[a2]/2);
  grasp_offsets.push_back(-getColumn(rot, a2) * get(dim)[a2]/2);

  vector<ManipulationPosePtr> results;
  for (vector<Vector3>::iterator it = grasp_offsets.begin(); it != grasp_offsets.end(); ++it) {
      checkGraspPosition(pos, *it, grasp_dir, results);
  }
  for (vector<ManipulationPosePtr>::iterator it = results.begin(); it != results.end(); ++it) {
      (*it)->label = object->identLabels[0];
      Matrix33 inv_rot;
      assert(inverse(rot, inv_rot));
      (*it)->offset = inv_rot * (*it)->offset;
      addToWorkingMemory(newDataID(), *it);
  }
}

void RobotPoseCalculator::checkGraspPosition(const Vector3& pos, const Vector3& dir, const Vector3& grasp_dir, std::vector<ManipulationPosePtr>& results) {
    Vector3 grasp_pos = pos + dir;
    Vector3 norm_dir = dir / norm(dir);
    Vector3 joint_pos = grasp_pos + norm_dir * ROBOT_HAND_OFFSET; // position of the robot hand
    std::cout << "\nnormal:" << norm_dir << endl;
    std::cout << "grasp pos:" << grasp_pos << endl;
    std::cout << "joint pos:" << joint_pos << endl;

    if (grasp_pos.z < 0.2) { 
        cout << "grasp pos too low" << endl;
        return;
    }
    if (joint_pos.z > MAX_JOINT_HEIGHT) { // grasping position is too high
        cout << "grasp pos too high" << endl;
        return;
    }
    if (norm_dir.z < -0.5) { // normal is pointing down
        cout << "pointing down" << endl;
        return;
    }
    if (norm_dir.z > 0.5) { // normal is pointing up
        cout << "pointing up" << endl;
        return;
    }
    Vector3 xy_pos = vector3(grasp_pos.x, grasp_pos.y, 0.0);
    Vector3 im_pos;
    Vector3 base_pos;
    double theta = atan2(-norm_dir.y, -norm_dir.x);
    // if (-norm_dir.y < 0)
    //     theta = -theta;

    double grasp_distance = GRASP_DISTANCE + ROBOT_ARM_X_OFFSET;
    if (dot(grasp_dir, norm_dir) > 0) {
        im_pos = xy_pos + grasp_dir * INIT_DISTANCE;
        base_pos = xy_pos + grasp_dir * grasp_distance;
    }
    else {
        im_pos = xy_pos - grasp_dir * INIT_DISTANCE;
        base_pos = xy_pos - grasp_dir * grasp_distance;
    }
    base_pos.z = theta;
    im_pos.z = theta;

    ManipulationPosePtr p = new ManipulationPose;
    Vector3 global_base = toGlobal(base_pos);
    std::cout << "base pose:" << base_pos << endl;
    std::cout << "global:" << global_base << endl;
    cout << "theta: " <<  global_base.z / M_PI * 180 << endl;
    std::cout << "init pose:" << im_pos << " / " << toGlobal(im_pos) <<  endl;
    p->robotPose = global_base;
    p->offset = dir;

    cout << "xypos = " << xy_pos << "  <xypos, -norm_dir> = " <<  dot(xy_pos/norm(xy_pos), -norm_dir) << "  |xypos| = " << norm(xy_pos) << endl;
    if (dot(xy_pos/norm(xy_pos), -norm_dir) > 0.95 && norm(xy_pos) <= MAX_GRASP_DISTANCE && norm(xy_pos) >= MIN_GRASP_DISTANCE) {
        std::cout << "object is graspable!" << endl;
        Vector3 pos1 = grasp_pos + norm_dir * 0.1;
        std::cout << "move to " << pos1 << " first" << endl;
        std::cout << "then to " << grasp_pos << endl;

        p->distance = 0.0;
    }
    else {
        p->distance = dist(global_base, vector3(m_x, m_y, 0.0));
    }
    
    // p.graspPoint = grasp_pos;
    // p.graspPointAbs = toGlobal(grasp_pos);
    // computePoseDistance(p);
    // std::cout << "angle-dist: " << p.angle << ", movement dist:" << p.dist << endl;
    
    results.push_back(p);
}

void RobotPoseCalculator::computePoseDistance(ManipulationPose& pose) {
    // Vector3 path = vector3(pose.robotPose.x, pose.robotPose.y, 0.0)  - vector3(m_x, m_y, 0.0);
    // Vector3 curr = vector3(cos(m_theta), sin(m_theta), 0.0);
    // Vector3 goal = vector3(cos(pose.robotPose.z), sin(pose.robotPose.z), 0.0);
    // cout << path << curr << goal << endl;
    // pose.dist = norm(path);
    // normalise(path);
    // pose.angle = acos(dot(path, curr)) + acos(dot(path, goal));
}


Vector3 RobotPoseCalculator::toGlobal(const Vector3& pos) {
    // z = theta
    Vector3 r;
    r.x = m_x + pos.x * cos(m_theta) - pos.y * sin(m_theta);
    r.y = m_y + pos.x * sin(m_theta) + pos.y * cos(m_theta);
    r.z = pos.z + m_theta;
    if (r.z > M_PI)
        r.z -= 2*M_PI;
    return r;
}

Vector3 RobotPoseCalculator::getObjectDimensions(const VisualObject& object) {
    Vector3 minPos = vector3(DBL_MAX, DBL_MAX, DBL_MAX);
    Vector3 maxPos = vector3(0, 0, 0);

    VertexSeq::iterator it = object.model->vertices.begin();
    for (; it != object.model->vertices.end(); ++it) {
        minPos.x = min(it->pos.x, minPos.x);
        minPos.y = min(it->pos.y, minPos.y);
        minPos.z = min(it->pos.z, minPos.z);
        maxPos.x = max(it->pos.x, maxPos.x);
        maxPos.y = max(it->pos.y, maxPos.y);
        maxPos.z = max(it->pos.z, maxPos.z);
    }
    return maxPos - minPos;
}


}

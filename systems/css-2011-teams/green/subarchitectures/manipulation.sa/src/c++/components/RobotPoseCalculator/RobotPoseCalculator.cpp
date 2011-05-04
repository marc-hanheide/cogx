/**
 * @author Michael Zillich
 * @date April 2011
*/

#include <string>
#include <sstream>
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
const double GRASP_DISTANCE = 0.3;
const double INIT_DISTANCE = 1.0;

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
    map<string,string>::const_iterator it;
    istringstream labeliss;

    if((it = _config.find("--labels")) != _config.end()){
		labeliss.str(it->second);
	} else {
		throw runtime_error(exceptionMessage(__HERE__, "No labels given"));
	}

    string label;
	while(labeliss >> label) {
        m_labels.insert(label);
	}
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
          if (m_labels.find(object->identLabels[i]) != m_labels.end()) {
              label = object->identLabels[i];
              log("object identified: " + object->identLabels[i]);
              break;
          } else
              log("unknown label: " + object->identLabels[i]);
      }
  }

  if (label.empty()) {
      log("object is unknown, aborting");
      return;
  }

  Vector3 pos = object->pose.pos;
  Matrix33 rot = object->pose.rot;

  if (isZero(getColumn(rot, 0))) {
      log("rotation matrix is zero, aborting");
      return;
  }

  ostringstream s;
  s << "robot position: " << m_x << " " << m_y << " / " << m_theta;
  log(s.str());

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

  s.str("");
  s << "center:" << pos << endl;
  s << "rotation:" << getColumn(rot,0) << " " << getColumn(rot,1) << " " << getColumn(rot,2) << endl;
  s << "dimensions:" << dim << endl;
  log(s.str());

  if (pos.z < 0.3 || pos.z > 0.8) {
      log("what is this? I'm hallucinating (or the object is lying on the floor).");
      return;
  }

  // Delete poses
  vector<CASTData<ManipulationPose> > poses;
  getMemoryEntriesWithData(poses);
  for(vector<CASTData<ManipulationPose> >::iterator it = poses.begin(); it != poses.end(); ++it) {
      if (it->getData()->label == label) {
          deleteFromWorkingMemory(it->getID());
      }
  }

  if (abs(getColumn(rot, short_axis).z) > 0.8) {
      log("object is lying on the side, no grasping is possible");
      return;
  }
  
  //Grasp vector is orthogonal to object short-axis
  Vector3 grasp_dir = cross(vector3(0.0, 0.0, 1.0), getColumn(rot, short_axis));
  log("grasp direction:" + toString(grasp_dir));
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
      (*it)->label = label;
      Matrix33 inv_rot;
      assert(inverse(rot, inv_rot));
      (*it)->offset = inv_rot * (*it)->offset;
      string id = newDataID();
      (*it)->id = id;
      addToWorkingMemory(id, *it);
      s.str("");
      s << "added pose for " << label << " with distance " << (*it)->distance << ": " << id;
      log(s.str());
  }
}

void RobotPoseCalculator::checkGraspPosition(const Vector3& pos, const Vector3& dir, const Vector3& grasp_dir, std::vector<ManipulationPosePtr>& results) {
    Vector3 grasp_pos = pos + dir;
    Vector3 norm_dir = dir / norm(dir);
    Vector3 joint_pos = grasp_pos + norm_dir * ROBOT_HAND_OFFSET; // position of the robot hand
    ostringstream s;
    s << "\nnormal:" << norm_dir << endl;
    s << "grasp pos:" << grasp_pos << endl;
    s << "joint pos:" << joint_pos << endl;
    log(s.str());

    if (grasp_pos.z < 0.2) { 
        log("grasp pos too low");
        return;
    }
    if (joint_pos.z > MAX_JOINT_HEIGHT) { // grasping position is too high
        log("grasp pos too high");
        return;
    }
    if (norm_dir.z < -0.5) { // normal is pointing down
        log("pointing down");
        return;
    }
    if (norm_dir.z > 0.5) { // normal is pointing up
        log("pointing up");
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
    s.str("");
    s << "base pose:" << base_pos << endl;
    s << "global:" << global_base << endl;
    s << "theta: " <<  global_base.z / M_PI * 180 << endl;
    s << "init pose:" << im_pos << " / " << toGlobal(im_pos) <<  endl;
    log(s.str());

    p->robotPose = global_base;
    p->offset = dir;

    s.str("");
    s << "xypos = " << xy_pos << "  <xypos, -norm_dir> = " <<  dot(xy_pos/norm(xy_pos), -norm_dir) << "  |xypos| = " << norm(xy_pos);
    log(s.str());
    if (dot(xy_pos/norm(xy_pos), -norm_dir) > 0.9 && norm(xy_pos) <= MAX_GRASP_DISTANCE && norm(xy_pos) >= MIN_GRASP_DISTANCE) {
        s.str("");
        s << "object is graspable!" << endl;
        Vector3 pos1 = grasp_pos + norm_dir * 0.1;
        s << "move to " << pos1 << " first" << endl;
        s << "then to " << grasp_pos << endl;
        log(s.str());
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

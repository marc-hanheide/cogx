/**
 * @author Team RED
 * @date 2011
 * @brief Tools for Manipulation Planner
 */

#ifndef MANIPULATION_PLANNER_H
#define MANIPULATION_PLANNER_H

#include "VisionData.hpp"
#include "Math.h"

namespace cast
{
  
  void calculateGripperPosition(VisionData::VisualObject &obj, cogx::Math::Pose3 &gripperPose);
  
}

#endif


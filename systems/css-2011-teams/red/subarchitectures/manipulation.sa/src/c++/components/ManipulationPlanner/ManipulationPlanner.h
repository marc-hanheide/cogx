/**
 * @author Team RED
 * @date 2011
 * @brief 
 */

#ifndef MANIPULATION_PLANNER_H
#define MANIPULATION_PLANNER_H

#include <cast/architecture/ManagedComponent.hpp>
#include "Tools.h"
#include <VisionData.hpp>
#include "Math.hpp"


namespace cast
{

class ManipulationPlanner : public ManagedComponent
{
private:
  void calculateGripperPosition(VisionData::VisualObjectPtr obj, cogx::Math::Pose3 &pose);
protected:

  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

public:
  ManipulationPlanner() {}
  virtual ~ManipulationPlanner() {}
};

}

#endif




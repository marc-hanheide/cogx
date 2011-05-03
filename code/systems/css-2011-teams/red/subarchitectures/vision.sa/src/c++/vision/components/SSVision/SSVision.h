/**
 * @author Team RED
 * @date 2011
 * @brief Vision system for the spring school system
 */

#ifndef SS_VISION_H
#define SS_VISION_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include "Math.hpp"


namespace cast
{

class SSVision : public ManagedComponent
{
private:
//   void calculateGripperPosition(VisionData::VisualObjectPtr obj, cogx::Math::Pose3 &pose);
//   void WriteGripperPositionToWM(VisionData::GripperPosePtr  grPose);

protected:

  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

//   void newVisualObject(const cdl::WorkingMemoryChange & _wmc);

public:
  SSVision() {}
  virtual ~SSVision() {}
};

}

#endif




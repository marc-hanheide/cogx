/**
 * @author Team RED
 * @date 2011
 * @brief 
 */

#ifndef SS_MANIPULATION_H
#define SS_MANIPULATION_H

#include <cast/architecture/ManagedComponent.hpp>
#include <VisionData.hpp>
#include <manipulation.hpp>
#include "Math.hpp"


namespace cast
{

class SSManipulation : public ManagedComponent
{
private:
  std::string gripperPoseID;      // The gripper pose id of the working memory
    
  void calculateGripperPosition(VisionData::VisualObjectPtr obj, cogx::Math::Pose3 &pose);
  void WriteGripperPositionToWM(VisionData::GripperPosePtr  grPose);
  void WriteCommandToWM(cast::cdl::WorkingMemoryAddress addr, int cmd, bool succeed, std::string id);

protected:
  virtual void configure(const std::map<std::string,std::string> & _config);
  virtual void start();
  virtual void runComponent();

  void receivedManipulationCommand(const cdl::WorkingMemoryChange & _wmc);

public:
  SSManipulation() {}
  virtual ~SSManipulation() {}
};

}

#endif




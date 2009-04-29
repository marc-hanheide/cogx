//
// = FILENAME
//    OdomLocalizer.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef OdomLocalizer_hpp
#define OdomLocalizer_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <OdometryReceiver.hpp>
#include <NavData.hpp>
#include <cast/core/CASTTimer.hpp>

// Cure include
#include <Transformation/Pose3D.hh>

namespace navsa {

/**
 * This components turns Odometry readings into NavData::RobotPose2d
 * objects in the working memory. This provides the means to bypass
 * SLAM/Localization and instead use odometry as the reference
 * position in the system. This is handy for testing and many
 * applications where you could not care less about drift and other
 * things that affect odometry over long time. This component will write a 
 * file called robotpose.ccf so that you can start from the same position
 * in an easy way. This is the same functionality as the one in SlamProcess.
 *
 * @param --interval time between writes to WM [s]. This is used
 *    to configure the push connection for odometry so that this comes at
 *    the desired rate
 * @param --robotpose-file path to the robot pose file. This allows you to 
 * specify where the robot should start in world coordinate. After that 
 * odometry will be used to update the position, ie dead reckoning
 * @param --no-file-output with this option this component will not write
 * the robotpose.ccf file.
 *
 * @author Patric Jensfelt
 *
 */
class OdomLocalizer : public cast::ManagedComponent,
                        public OdometryReceiver
{
public:
  OdomLocalizer();
  virtual ~OdomLocalizer();
  
  virtual void runComponent();
  void receiveOdometry(const Robotbase::Odometry &odom);

 protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  virtual void stop();

  void updateRobotPoseInWM();

  void writeRobotPoseFile();

 private:
  double m_Interval;
  std::string m_RobotPoseIdString;

  Cure::Pose3D m_Pose;

  cast::CASTTimer m_WriteTimer;
  double m_WriteInterval;
  bool m_DontWriteFiles;

  bool m_GotFirstOdom;
  Cure::Pose3D m_LastOdom;
};

}; // namespace navsa

#endif // OdomLocalizer_hpp

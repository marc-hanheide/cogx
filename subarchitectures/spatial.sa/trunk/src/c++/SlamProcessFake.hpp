//
// = FILENAME
//    SlamProcessFake.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Nikolaus Demmel (based on SlamProcess by Chandana Paul, Patric Jensfelt)
//
// = COPYRIGHT
//    Copyright (c) 2012 Nikolaus Demmel
//                  2009 Patric Jensfelt
//                  2007 Chandana Paul, Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef SlamProcessFake_hpp
#define SlamProcessFake_hpp

//#include <peekabot.hh>
//#include <peekabot/Types.hh>

#include <cast/architecture/ManagedComponent.hpp>
#include <castutils/OptionParserMixin.hpp>
#include <OdometryReceiver.hpp>
#include <NavData.hpp>
#include <Robotbase.hpp>
#include <cast/core/CASTTimer.hpp>

// TODO: remove unneeded

// Cure stuff
#include <Map/PoseProvider.hh>
#include <SensorData/LaserScan2d.hh>
#include <Map/WrappedSLAM.hh>
#include <Map/WrappedLocalization.hh>
#include <Transformation/Pose3D.hh>
#include <AddressBank/ConfigFileReader.hh>
#include <Utils/RoboLookProxy.h>
#include <Sensory/RANSACSegmentor.hh>
#include <Sensory/LsqLineFitter.hh>

namespace navsa {

/**
 * This is a CAST managed process that runs fake SLAM which simply passes on the
 * odometry input. It can still load a metric map in order to write it to
 * working memory. Combine this with RobotbaseServerFake to get a constant 0
 * pose.
 *
 * @author Nikolaus Demmel
 *
 * Command line options:
 *
 * @param -m optional map file. If the file exist it will be loaded and the
 *   metric map will be written to working memory.
 */
  class SlamProcessFake: 
      public castutils::OptionParserMixinBaseclass<cast::ManagedComponent>,
      public OdometryReceiver
{
public:
  SlamProcessFake();
  virtual ~SlamProcessFake();

  void receiveOdometry(const Robotbase::Odometry &odom);

protected:
  virtual void runComponent();
  virtual void configure(const std::map<std::string, std::string> &config);

  void updateRobotPoseInWM(const NavData::RobotPose2dPtr &pose);
  void writeLineMapToWorkingMemory(Cure::FeatureMap &fm);

  void connectPeekabot();

private:

  // The string that identifies the robotpose in the working memory
  std::string _robotPoseIdString;

  // The string that identifies the map in the working memory
  std::string _lineMapIdString;

  // The name of the map file we read from / write to
  std::string _mapFilename;

  // The config file for loading the metric map
  std::string _configFilename;

};

}; // namespace navsa

#endif

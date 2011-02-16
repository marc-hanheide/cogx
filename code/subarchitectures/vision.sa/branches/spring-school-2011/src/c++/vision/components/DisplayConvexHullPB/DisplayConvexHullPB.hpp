#ifndef DisplayConvexHullPB_hpp
#define DisplayConvexHullPB_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <string>

#include <peekabot.hh>
#include <peekabot/Types.hh>
#include <VisionData.hpp>
#include <NavData.hpp>
#include <SensorData/SensorPose.hh>
#include <Transformation/Transformation3D.hh>
#include <PTZ.hpp>

class DisplayConvexHullPB : public cast::ManagedComponent 
{
public:
  DisplayConvexHullPB();
  virtual ~DisplayConvexHullPB();
  
  virtual void runComponent();
  virtual void start();
  
protected:

  virtual void configure(const std::map<std::string, std::string>& _config);

private:
  std::vector<double> previouscenter;
  void newConvexHull(const cast::cdl::WorkingMemoryChange &objID);
  void robotPoseChanged(const cast::cdl::WorkingMemoryChange &objID);
  Cure::Transformation3D getCameraToWorldTransform();
  void connectPeekabot();
  int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry

  double m_FovH; // horisontal fov in degs
  double m_FovV; // vertical fov in degs

  NavData::RobotPose2dPtr lastRobotPose;

  IceUtil::Mutex m_Mutex;
  ptz::PTZInterfacePrx m_ptzInterface;

  std::string m_PbHost;
  int m_PbPort;
  std::string m_PTZServer;

  peekabot::PeekabotClient m_PeekabotClient;
  peekabot::ObjectProxy m_ProxyRoot;

  Cure::SensorPose m_CameraPoseR;
};

#endif // DisplayConvexHullPB_hpp

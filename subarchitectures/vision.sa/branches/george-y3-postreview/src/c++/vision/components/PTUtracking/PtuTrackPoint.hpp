
#ifndef PTU_TRACK_POINT_HPP
#define PTU_TRACK_POINT_HPP

#include <stdexcept>
#include <vector>
#include <cast/architecture/ManagedComponent.hpp>
#include <Math.hpp>
#include <PTZ.hpp>
#include <NavData.hpp>
#include <VisionData.hpp>

#include <Transformation/Pose3D.hh>


class PtuTrackPoint : public cast::ManagedComponent
{
private:
  ptz::PTZInterfacePrx m_PTUServer;
  double pan;
  double tilt;
  
  //position of the robot in world coordinate frame
  Cure::Pose3D cp;

  //target point
  double x;
  double y; 
  double z;
  
  bool dontTrackSOIs;
  bool trackFirstSOI;
  bool waitingForFirstSOI;
  const char *firstSOIid;  

  void MovePanTilt(double pan, double tilt, double tolerance);
  void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
  void trackSOI(const cast::cdl::WorkingMemoryChange &objID);
  void trackTargetPoint(Cure::Pose3D cp);
  
protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  virtual void start();
  virtual void runComponent();

public:
  PtuTrackPoint() : pan(0), tilt(0), x(0), y(0), z(0), waitingForFirstSOI(true) {}
  virtual ~PtuTrackPoint() {}
  
  void setTargetPoint(double x, double y, double z);
};

#endif


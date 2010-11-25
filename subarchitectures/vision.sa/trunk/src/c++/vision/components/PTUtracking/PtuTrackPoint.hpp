
#ifndef PTU_TRACK_POINT_HPP
#define PTU_TRACK_POINT_HPP



#include <stdexcept>
#include <vector>
#include <cast/architecture/ManagedComponent.hpp>
#include <Math.hpp>
#include <PTZ.hpp>
#include <NavData.hpp>

#include <Transformation/Pose3D.hh>


class PtuTrackPoint : public cast::ManagedComponent
{
private:
  ptz::PTZInterfacePrx m_PTUServer;
  double pan;
  double tilt;
  
  //target point
  double x;
  double y; 
  double z;
  
  //variables needed for calculation of pan and tilt angles in trackTargetPoint
  double T_R0x, T_R0y; 
  double T_Rx, T_Ry;
  double phi_R, phi_R_abs;
  double T_R_abs, TD, sin_alpha;
  double T_PD_abs;
  double c, gamma, beta, gamma2;
  
  
  void MovePanTilt(double pan, double tilt, double tolerance);
  void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
  void trackTargetPoint(Cure::Pose3D cp);
  
protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  virtual void start();
  virtual void runComponent();

public:
  PtuTrackPoint() : pan(0), tilt(0), x(0), y(0), z(0) {}
  virtual ~PtuTrackPoint() {}
  
  void setTargetPoint(double x, double y, double z);
};

#endif


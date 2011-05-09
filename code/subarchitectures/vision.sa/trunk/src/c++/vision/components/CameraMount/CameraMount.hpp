/**
 * Provides 3D poses of cameras.
 * Poses can change if cameras are mounted on pan-tilt unit or arm.
 * NOTE: right now only works with pan-tilt unit (PTZ server).
 *
 * Michael Zillich
 * Oct 2009
 */

#ifndef CAMERA_MOUNT_HPP
#define CAMERA_MOUNT_HPP

#include <stdexcept>
#include <vector>
#include <cast/architecture/ManagedComponent.hpp>
#include <Math.hpp>
#include <PTZ.hpp>

class CameraMount : public cast::ManagedComponent
{
private:
  /**
   * Tolerance for deciding whther we reached a fixed position, in rad.
   * 0.015 rad = a bit less than 1 degree
   */
  static double FIXED_POSITION_TOLERANCE;
  /**
   * ICE inteface to PTU server
   */
  ptz::PTZInterfacePrx m_PTUServer;
  /**
   * IDs of cameras (0, 1 etc.)
   */
  std::vector<int> camIds;
  /**
   * WM IDs of camera poses.
   */
  std::vector<std::string> camWMIds;
  /**
   * Pose of PTU base w.r.t. to robot ego.
   */
  cogx::Math::Pose3 ptBasePose;
  /**
   * Pose of PTU pan joint w.r.t. base.
   */
  cogx::Math::Pose3 ptPanPose;
  /**
   * Pose of PTU tilt joint w.r.t. pan joint.
   */
  cogx::Math::Pose3 ptTiltPose;
  /**
   * Poses of cameras w.r.t. tilt joint.
   */
  std::vector<cogx::Math::Pose3> camPoses;
  /**
   * Poses of cameras w.r.t. robot ego, for fixed pan/tilt angle.
   */
  std::vector<cogx::Math::Pose3> camFixedPoses;
  /**
   * If true, ask the PTZ server regularly for pan/tilt angles to update camera poses.
   */
  bool usePTZ;
  /**
   * Fixed pan and tilt angle, for which we have precise calibration.
   */
  ptz::PTZReading fixedPanTilt;

  /**
   * for given pan and tilt angles, calculate poses of cameras w.r.t. robot ego
   * system
   */
  void calculatePoses(ptz::PTZReading &ptz, std::vector<cogx::Math::Pose3> &camPosesToEgo);

protected:
  virtual void configure(const std::map<std::string, std::string>& _config) throw(std::runtime_error);
  virtual void start();
  virtual void runComponent();

public:
  CameraMount();
  virtual ~CameraMount() {}
};

#endif


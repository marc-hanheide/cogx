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
  ptz::PTZInterfacePrx m_PTUServer;
  /**
   * Pose of pan-tilt head w.r.t. to robot ego, when all angles are zero.
   */
  cogx::Math::Pose3 ptZeroPose;
  /**
   * Poses of cameras w.r.t. to pan-tilt head.
   */
  std::vector<cogx::Math::Pose3> camPoses;
  /**
   * IDs of cameras (0, 1 etc.)
   */
  std::vector<int> camIds;
  /**
   * WM IDs of camera poses.
   */
  std::vector<std::string> camWMIds;

  /**
   * for given pan and tilt angles, calculate poses of cameras w.r.t. robot ego
   * system
   */
  void calculatePoses(ptz::PTZReading &ptz, std::vector<cogx::Math::Pose3> &poses);

protected:
  virtual void configure(const std::map<std::string, std::string>& _config)
    throw(std::runtime_error);
  virtual void start();
  virtual void runComponent();

public:
  CameraMount() {}
  virtual ~CameraMount() {}
};

#endif


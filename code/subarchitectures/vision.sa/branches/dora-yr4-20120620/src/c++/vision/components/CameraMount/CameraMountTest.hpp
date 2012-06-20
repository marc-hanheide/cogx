/**
 * Test camera mount component.
 * Set some pan tilt angles to the pan tilt head and receive camera poses.
 *
 * Michael Zillich
 * Oct 2009
 */

#ifndef CAMERA_MOUNT_TEST_HPP
#define CAMERA_MOUNT_TEST_HPP

#include <stdexcept>
#include <vector>
#include <string>
#include <cast/architecture/ManagedComponent.hpp>
#include <Math.hpp>
#include <PTZ.hpp>

class CameraMountTest : public cast::ManagedComponent
{
private:
  ptz::PTZInterfacePrx m_PTUServer;
  double pan;
  double tilt;
  std::string ptzServerComponent;

  void MovePanTilt(double pan, double tilt, double tolerance);

protected:
  virtual void configure(const std::map<std::string, std::string>& _config);
  virtual void start();
  virtual void runComponent();

public:
  CameraMountTest() : pan(0), tilt(0) {}
  virtual ~CameraMountTest() {}
};

#endif


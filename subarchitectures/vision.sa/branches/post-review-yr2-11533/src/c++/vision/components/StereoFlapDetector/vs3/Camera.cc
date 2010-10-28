/**
 * $Id: Camera.cc,v 1.1 2006/02/06 16:26:21 mz Exp mxz $
 */

#include <fstream> 
#include "Config.hh"
#include "Camera.hh"

namespace ACIN
{

Camera::Camera()
{
  fx = fx = 1.;
  u0 = v0 = 0.;
  k1 = k2 = p1 = p2 = 0.;
  pose = Pose3::ZERO;
}

void Camera::ReadCalibration(const string &calibfile)
{
  Config cfg(calibfile);
  fx = cfg.GetDouble("fx");
  fy = cfg.GetDouble("fy");
  u0 = cfg.GetDouble("u0");
  v0 = cfg.GetDouble("v0");
  k1 = cfg.GetDouble("k1");
  k2 = cfg.GetDouble("k2");
  p1 = cfg.GetDouble("p1");
  p2 = cfg.GetDouble("p2");
}

void Camera::ReadPose(const string &posefile)
{
  ifstream p(posefile.c_str());
  p >> pose;
}

void Camera::Read(const string &calibfile, const string &posefile)
{
  ReadCalibration(calibfile);
  ReadPose(posefile);
}

}


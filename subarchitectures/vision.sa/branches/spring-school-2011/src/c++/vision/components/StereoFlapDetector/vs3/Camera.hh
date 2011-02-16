/**
 * $Id: Camera.hh,v 1.2 2006/11/24 13:47:03 mxz Exp mxz $
 */

#ifndef Z_CAMERA_HH
#define Z_CAMERA_HH

#include "Namespace.hh"
#include "Vector2.hh"
#include "Pose3.hh"

namespace ACIN
{

/**
 * Camera class.
 * For some theory:
 * Sonka, Hlavac, Boyle: Image Processing, Analysy and Machine Vision
 *   2nd ed., chapter 9.2 Geometry for 3D Vision, p. 448.
 *   Note however that we use positive entries in the camera calibration matrix
 *   (i.e. we place the image plane conveniently in front of the focal point).
 * Trucco, Verri: Introductory Techniques for 3D Computer Vision
 *   chapter 2.4 Camera parameters, p. 34
 *   Note again, we use positive entries.
 * Intel OpenCV library manual (OpenCVMan.pdf), chapter 6 3D Reconstruction,
 *   p. 6-1
 *
 * We use the following notation:
 * extrinsic parameters, pose:
 *   x_w = R*x_c + t   R .. rotation matrix, t .. translation vector
 *                     x_w .. world coords, x_c .. camera coords
 * projection to image plane:
 *   u = A*x_c         A .. 3x3 camera matrix, u .. homogenous pixel coords
 *
 *       | f_x  0  u_0 |   f_x = f/s_x where f .. focal length in mm
 *   A = |  0  f_y v_0 |                     s_x .. pixel size in mm/pix
 *       |  0   0   1  |   u_0, v_0 .. principal point in pix
 *
 *   u = U/W    U, V, W .. homogenous pixel coords
 *   v = U/W    u, v .. pixel coords
 *   thus:
 *   u = f_x*x_c/z_c + u_0
 *   v = f_y*y_c/z_c + v_0
 * distortion:  u', v* is the distored pixel point
 * (note that the higher order factor k2 and the tangential distortion factors
 *  p1, p2 are not used)
 *   u' = u + (u - u_0)*k1*r^2   r^2 = u^2 + v^2
 *   v' = v + (v - v_0)*k1*r^2
 *
 */
class Camera
{
public:
  // Instrinsic parameters:
  /// entries of the camera matrix
  double fx, fy, u0, v0;
  /// radial distortion parameters
  double k1, k2;
  /// tangential distortion parameters
  double p1, p2;
  /// extrinsic parameters: 3D pose
  Pose3 pose;

public:
  Camera();
  void Read(const string &calibfile, const string &posefile);
  void ReadCalibration(const string &calibfile);
  void ReadPose(const string &posefile);
  void DistortPoint(int u, int v, int &ud, int &vd) const;
  void DistortPoint(double u, double v, double &ud, double &vd) const;
  Vector2 ProjectPoint(const Vector3 &w);
};

}

#include "Camera.ic"

#endif



#ifndef STEREO_CAMERA_HH
#define STEREO_CAMERA_HH

#include "Namespace.hh"
//#include "Pose3.hh"
//#include "Math.hh"

namespace Z
{

#define LEFT 0
#define RIGHT 1

/**
 * | u |     | X |
 * | v | = P | Y |
 * | w |     | Z |
 *           | 1 |
 * u/w and v/w idealised image coordinates
 * projection matrix:
 *     | fx  0  cx  -fx tx |
 * P = | 0  fy  cy     0   |
 *     | 0   0   0     0   |
 * where element -fx tx only for right image, 0 for left image
 *
 * | X |     | u |
 * | Y | = Q | v |
 * | Z |     | d |
 * | W |     | 1 |
 * 3D point in left camera coordinages X/W, Y/W, Z/W
 * reprojection matrix:
 *     | 1  0   0   -cx            |
 * Q = | 0  1   0   -cy            |
 *     | 0  0   0    fx            |
 *     | 0  0 -1/tx (cx - cx_r)/tx |
 * where cx is for left image and cx_r for right image
 */
class StereoCamera
{
public:
  class MonoParam
  {
  public:
    int width, height;
    double fx, fy, cx, cy;
    double k1, k2, k3, t1, t2;
    double proj[3][4];
    double rect[3][3];
    MonoParam()
    {
      width = height = 0;
      fx = fy = cx = cy = 0.;
      k1 = k2 = k3 = t1 = t2 = 0.;
      proj[0][0] = proj[0][1] = proj[0][2] = proj[0][3] = 0.;
      proj[1][0] = proj[1][1] = proj[1][2] = proj[1][3] = 0.;
      proj[2][0] = proj[2][1] = proj[2][2] = proj[2][3] = 0.;
      rect[0][0] = rect[0][1] = rect[0][2] = 0.;
      rect[1][0] = rect[1][1] = rect[1][2] = 0.;
      rect[2][0] = rect[2][1] = rect[2][2] = 0.;
    }
  }; 
  /// parameters specific to LEFT and RIGHT camera
  MonoParam cam[2];
  double maxDistortion;
  /// pose of left camera
  //Pose3 pose;

public:
  StereoCamera();
  void ReadSVSCalib(const string &calibfile);
  void ProjectPoint(double X, double Y, double Z, double &u, double &v,
      int side);
  void ReconstructPoint(double u, double v, double d, double &X, double &Y,
      double &Z);
  void DistortPoint(double u, double d, double &ud, double &vd, int side);
  bool UndistortPoint(double ud, double vd, double &u, double &v,int side);
  void SetMaxDistortion(double err=.5){maxDistortion=err;}
  void RectifyPoint(double u, double v, double &ur, double &vr, int side);
};

}

#endif


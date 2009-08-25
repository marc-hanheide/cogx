/**
 * @author Michael Zillich
 * @date February 2009
 */

#ifndef CAMERA_PARAMETERS_H
#define CAMERA_PARAMETERS_H

#include <string>
#include <cogxmath.h>
#include "Video.hpp"

namespace Video
{

using namespace std;
using namespace cogx::Math;

/**
 * Initialise camera parameters to meaningful default values.
 * (focal length to 1, pose to identity, everything else to 0)
 */
void initCameraParameters(CameraParameters &cam);

/**
 * Load camera parameters from config file.
 * The file contains internal and external (pose) parameters.
 */
void loadCameraParameters(CameraParameters &cam, const string &configfile);

/**
 * Save camera parameters to a config file.
 * The file contains internal and external (pose) parameters.
 */
void saveCameraParameters(const CameraParameters &cam, const string &configfile);

void distortPoint(const CameraParameters &cam, double u, double v,
    double &ud, double &vd);

inline void distortPoint(const CameraParameters &cam, int u, int v,
    int &ud, int &vd)
{
  double udf, vdf;
  distortPoint(cam, (double)u, (double)v, udf, vdf);
  ud = (int)roundf((float)udf);
  vd = (int)roundf((float)vdf);
}

inline void distortPoint(const CameraParameters &cam, const Vector2 &u,
    Vector2 &ud)
{
  distortPoint(cam, u.x, u.y, ud.x, ud.y);
}

/**
 * Undistort point.
 * Uses an iterative method for the inverse mapping (borrowed from Markus
 * Bader).
 * @return  return true if the method converged, false otherwise
 */
bool undistortPoint(const CameraParameters &cam, double ud, double vd,
    double &u, double &v);

inline bool undistortPoint(const CameraParameters &cam, int ud, int vd,
    int &u, int &v)
{
  double uf, vf;
  bool ret = undistortPoint(cam, (double)ud, (double)vd, uf, vf);
  u = (int)roundf((float)uf);
  v = (int)roundf((float)vf);
  return ret;
}

inline bool undistortPoint(const CameraParameters &cam, const Vector2 &ud,
    Vector2 &u)
{
  return undistortPoint(cam, ud.x, ud.y, u.x, u.y);
}

/**
 * Project a point given in world co-ordinates to image pixel co-ordinates.
 */
inline Vector2 projectPoint(const CameraParameters &cam, const Vector3 &w)
{
  Vector3 p = transformInverse(cam.pose, w);
  return vector2(cam.fx*p.x/p.z + cam.cx, cam.fy*p.y/p.z + cam.cy);
}

/**
 * Given image pixel coordinates (x,y) return the according normalised view
 * ray in camera local co-ordinates.
 */
inline Vector3 localRay(const CameraParameters &cam, const Vector2 &u)
{
  Vector3 r = vector3(u.x - cam.cx, (u.y - cam.cy)*cam.fx/cam.fy, cam.fx);
  normalise(r);
  return r;
}

/**
 * Given image pixel coordinates (x,y) return the according normalised view
 * ray in world co-ordinates.
 */
inline Vector3 globalRay(const CameraParameters &cam, const Vector2 &u)
{
  return transformDirection(cam.pose, localRay(cam, u));
}

}

#endif

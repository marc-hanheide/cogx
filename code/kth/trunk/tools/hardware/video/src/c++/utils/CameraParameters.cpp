/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <fstream>
#include <sstream>
#include "CDataFile.h"
#include "CameraParameters.h"

namespace Video
{

using namespace std;
using namespace cogx::Math;

static const char *calib_header =
"# Intrinsic parameters for pinhole camera model.\n"
"# The pinhole camera model is as follows:\n"
"#\n"
"# We use the following notation:\n"
"# Matrices (R, A) are upper case. Vectors (t, u) are lower case, homogenous\n"
"# vectors (U) are upper case.\n"
"#\n"
"# We have the following coordinate systems:\n"
"# world: 3D coordinate system, units are [m], origin and orientation arbitrary\n"
"# camera: 3D coordinate system, units are [m], origin in camera center of\n"
"#         projection, x points to the 'right', y points 'down' and z points\n"
"#         'away', all w.r.t. the world coordinate system\n"
"# image plane: 2D coordinate system, units are [m], origin at the intersection\n"
"#              of principal ray and image plane, x points to the 'right' and y\n"
"#              points 'down'\n"
"# image pixel: 2D coordinate system, units are [pixel], origin is in upper left\n"
"#              corner of the image, x points 'right' and y points 'down'\n"
"#\n"
"# w  = [w_x w_y w_z]  .. point in world coords\n"
"# p  = [p_x p_y p_z]  .. point in camera coords\n"
"# U  = [U_X U_Y U_W]  .. point in homogeneous image pixel coords\n"
"# u  = [u_x u_y]      .. ideal point in image pixel coords\n"
"# u' = [u_x' u_y']    .. distorted point in image pixel coords\n"
"#\n"
"# extrinsic parameters, pose:\n"
"#   w = R*p + t   R .. rotation matrix, t .. translation vector\n"
"#                 Note that R,t is the pose of the camera w.r.t. the world\n"
"#                 origin. Some references use the inverse, i.e. p = R*w + t,\n"
"#                 where R,t is now the pose of the world w.r.t. to the camera.\n"
"# thus\n"
"#   p = R^T*(w - t)\n"
"#\n"
"# projection to image:\n"
"#   U = A*p       A .. 3x3 camera matrix\n"
"#\n"
"#       | f_x  0  c_x |   f_x = f/s_x where f .. focal length in mm\n"
"#   A = |  0  f_y c_y |                     s_x .. pixel size in mm/pix\n"
"#       |  0   0   1  |   c_x, c_y .. camera principal point in pix\n"
"#\n"
"#   u_x = U_X/U_W\n"
"#   u_x = U_Y/U_W\n"
"# i.e. written out:\n"
"#   u_x = f_x*p_x/p_z + c_x\n"
"#   u_y = f_y*p_y/p_z + c_y\n"
"#\n"
"# distortion:\n"
"# with\n"
"#   x = p_x/p_z\n"
"#   y = p_y/p_z\n"
"#   r^2 = x^2 + y^2\n"
"#\n"
"#   x' = x*(1 + k1*r^2 + k2*r^4) + 2*p1*x*y + p2*(r^2 + 2*x^2)\n"
"#   y' = y*(1 + k1*r^2 + k2*r^4) + p1*(r2 + 2*y^2) + 2*p2*x*y\n"
"#\n"
"#   u_x' = f_x*x' + c_x\n"
"#   u_y' = f_x*y' + c_y\n"
"#\n"
"# Note that only the ratios focal length to pixel size can be determined.\n"
"# Given a (known) nominal focal length f (e.g. from camera data sheet) we\n"
"# can calculate:\n"
"# s_x = f/f_x .. horizontal pixel size [mm/pix]\n"
"# s_y = f/f_y .. vertical pixel size [mm/pix]\n"
"# r = s_y/s_s .. pixel aspect ratio\n"
"\n";

static const char *pose_header =
"# pose given as [x y z] [a b c]\n"
"#  x, y, z  position in [m]\n"
"#  a, b, c  rotation vector, direction is axis, length is angle in [rad]\n"
"\n";

void initCameraParameters(CameraParameters &cam)
{
  cam.width = cam.height = 0;
  cam.fx = cam.fy = 1.;
  cam.cx = cam.cy = 0.;
  cam.k1 = cam.k2 = cam.k3 = cam.p1 = cam.p2 = 0.;
  setIdentity(cam.pose);
}

void loadCameraParameters(CameraParameters &cam, const string &configfile)
{
  // first make sure all values are set to meaningful defaults
  // note that some values might not be set from the config file
  initCameraParameters(cam);

  CDataFile cfg(configfile);
  cam.width = cfg.GetFloat("w");
  cam.height = cfg.GetFloat("h");
  cam.fx = cfg.GetFloat("fx");
  cam.fy = cfg.GetFloat("fy");
  cam.cx = cfg.GetFloat("cx");
  cam.cy = cfg.GetFloat("cy");
  cam.k1 = cfg.GetFloat("k1");
  cam.k2 = cfg.GetFloat("k2");
  cam.k3 = cfg.GetFloat("k3");
  cam.p1 = cfg.GetFloat("p1");
  cam.p2 = cfg.GetFloat("p2");
  istringstream istr(cfg.GetString("pose"));
  if(!istr.str().empty())
    istr >> cam.pose;
}

void saveCameraParameters(const CameraParameters &cam, const string &configfile)
{
  ofstream ostr(configfile.c_str());
  // nominal focal length in mm, we don't know it, so just set to 1
  double f = 1.;
  // pixel size in mm/pix
  double sx = f/cam.fx, sy = f/cam.fy;
  // aspect ratio
  double r = cam.fy/cam.fx;
  time_t t = time(NULL);

  // file header including creation date
  ostr << "# camera parameter file generated " << ctime(&t) << endl;
  ostr << calib_header;

  ostr << "# focal lengths in [pix]\n";
  ostr << "fx = " << cam.fx << endl;
  ostr << "fy = " << cam.fy << endl;
  ostr << "# principal point [pix]\n";
  ostr << "cx = " << cam.cx << endl;
  ostr << "cy = " << cam.cy << endl;

  ostr << "\n# (nominal) focal length [mm]\n";
  ostr << "f  = " << f << endl;
  ostr << "# pixel size [mm/pix]\n";
  ostr << "sx = " << sx << endl;
  ostr << "sy = " << sy << endl;
  ostr << "# aspect ratio: pixel size y/x\n";
  ostr << "r  = " << r << endl;

  ostr << "# image width and height\n";
  ostr << "width  = " << cam.width << endl;
  ostr << "height = " << cam.height << endl;

  ostr << "\n# distortion parameters\n";
  ostr << "k1 = " << cam.k1 << endl;
  ostr << "k2 = " << cam.k2 << endl;
  ostr << "k3 = " << cam.k3 << endl;
  ostr << "p1 = " << cam.p1 << endl;
  ostr << "p2 = " << cam.p2 << endl;
  ostr << endl;

  // some header for the pose
  ostr << endl << endl << pose_header;
  ostr << "pose = " << cam.pose;
}

void distortPoint(const CameraParameters &cam, double u, double v,
    double &ud, double &vd)
{
  double x = (u - cam.cx)/cam.fx;
  double y = (v - cam.cy)/cam.fy;
  double x2 = x*x;
  double y2 = y*y;
  double two_xy = 2.*x*y;
  double r2 = x2 + y2;
  double r4 = r2*r2;
  double r6 = r4*r2;
  double t = (1. + cam.k1*r2 + cam.k2*r4 + cam.k3*r6);
  double xd = x*t + two_xy*cam.p1 + cam.p2*(r2 + 2.*x2);
  double yd = y*t + two_xy*cam.p2 + cam.p1*(r2 + 2.*y2);
  ud = xd*cam.fx + cam.cx;
  vd = yd*cam.fy + cam.cy;
}

bool undistortPoint(const CameraParameters &cam, double ud, double vd,
    double &u, double &v)
{
  const int MAX_ITER = 100;
  const double eps = 0.25;
  double error = REAL_MAX;
  double dx = 0, dy = 0;
  double currentx = 0, currenty = 0;

  u = ud;
  v = vd;
  for(int i = 0; i < MAX_ITER && error > eps; i++)
  {
    distortPoint(cam, u, v, currentx, currenty);
    dx = ud - currentx;
    dy = vd - currenty;
    u += dx;
    v += dy;
    error = sqr(dx) + sqr(dy);
  }
  clamp(u, 0., (double)cam.width - 1.);
  clamp(v, 0., (double)cam.height - 1.);
  return error <= eps;
}

}


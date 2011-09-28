/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <fstream>
#include <sstream>
#include <opencv/cv.h>
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
  cam.id = -1;
  cam.width = cam.height = 0;
  cam.fx = cam.fy = 1.;
  cam.cx = cam.cy = 0.;
  cam.k1 = cam.k2 = cam.k3 = cam.p1 = cam.p2 = 0.;
  setIdentity(cam.pose);
  cam.time.s = cam.time.us = 0;
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

void loadCameraParametersXML(CameraParameters &cam, const string &configfile)
  throw(runtime_error)
{
  // first make sure all values are set to meaningful defaults
  // note that some values might not be set from the config file
  initCameraParameters(cam);

  cv::FileStorage calibFile(configfile, cv::FileStorage::READ);
  if(calibFile.isOpened())
  {
    CvMat *intr = (CvMat*)calibFile["intrinsic"].readObj();
    CvMat *dist = (CvMat*)calibFile["distortion"].readObj();
    CvMat *imgsize = (CvMat*)calibFile["imgsize"].readObj();
    CvMat *tvec = (CvMat*)calibFile["tvec"].readObj();
    CvMat *rmat = (CvMat*)calibFile["rmat"].readObj();

    cam.width = (int)cvGetReal1D(imgsize, 0);
    cam.height = (int)cvGetReal1D(imgsize, 1);
    cam.fx = cvGetReal2D(intr, 0, 0);
    cam.fy = cvGetReal2D(intr, 1, 1);
    cam.cx = cvGetReal2D(intr, 0, 2);
    cam.cy = cvGetReal2D(intr, 1, 2);
    cam.k1 = cvGetReal1D(dist, 0);
    cam.k2 = cvGetReal1D(dist, 1);
    cam.p1 = cvGetReal1D(dist, 2);
    cam.p2 = cvGetReal1D(dist, 3);
    if(dist->rows >= 5)
      cam.k3 = cvGetReal1D(dist, 4);
    else
      cam.k3 = 0.;

    // these are optional
    if(tvec != 0 && rmat != 0)
    {
      cam.pose.pos = vector3(cvGetReal1D(tvec, 0), cvGetReal1D(tvec, 1), cvGetReal1D(tvec, 2));
      cam.pose.rot.m00 = cvGetReal2D(rmat, 0, 0);
      cam.pose.rot.m01 = cvGetReal2D(rmat, 0, 1);
      cam.pose.rot.m02 = cvGetReal2D(rmat, 0, 2);
      cam.pose.rot.m10 = cvGetReal2D(rmat, 1, 0);
      cam.pose.rot.m11 = cvGetReal2D(rmat, 1, 1);
      cam.pose.rot.m12 = cvGetReal2D(rmat, 1, 2);
      cam.pose.rot.m20 = cvGetReal2D(rmat, 2, 0);
      cam.pose.rot.m21 = cvGetReal2D(rmat, 2, 1);
      cam.pose.rot.m22 = cvGetReal2D(rmat, 2, 2);
    }
  }
  else
  {
    throw runtime_error(exceptionMessage(__HERE__, "failed to read calibration file '%s'", configfile.c_str()));
  }
}

void loadCameraParametersFromSVSCalib(CameraParameters &cam,
    const string &configfile, int side)
{
  // first make sure all values are set to meaningful defaults
  // note that some values might not be set from the config file
  initCameraParameters(cam);

  CDataFile file(configfile);

  const char *side_str = side == LEFT ? "left camera" : "right camera";
  cam.width = file.GetInt("pwidth", side_str);
  cam.height = file.GetInt("pheight", side_str);
  cam.fx = file.GetFloat("f", side_str);
  cam.fy = file.GetFloat("fy", side_str);
  cam.cx = file.GetFloat("Cx", side_str);
  cam.cy = file.GetFloat("Cy", side_str);
  cam.k1 = file.GetFloat("kappa1", side_str);
  cam.k2 = file.GetFloat("kappa2", side_str);
  cam.k3 = file.GetFloat("kappa3", side_str);
  cam.p1 = file.GetFloat("tau1", side_str);
  cam.p2 = file.GetFloat("tau2", side_str);

  Pose3 global_pose, right_pose;
  Vector3 r;

  // read poses
  // We have:
  // - pose of the whole stereo rig, we call that the global rig pose
  // - pose of the left camera relative to the rig, we assume this to be
  //   identity (as is quite common), so global rig pose and global left camera
  //   pose are the same
  // - global pose of left camera, same as global rig pose (see above)
  //   This is called "global" in the calibration file.
  // - pose of the right camera relative to the rig.
  //   This is called "external" in the calibration file (NOTE: Actually it is
  //   the inverse of "external"!)
  // - global pose of the right camera, i.e. global rig pose + relative right
  //   pose

  global_pose.pos.x = file.GetFloat("GTx", "global");
  global_pose.pos.y = file.GetFloat("GTy", "global");
  global_pose.pos.z = file.GetFloat("GTz", "global");
  // SVS use mm, we use m
  global_pose.pos /= 1000.;
  r.x = file.GetFloat("GRx", "global");
  r.y = file.GetFloat("GRy", "global");
  r.z = file.GetFloat("GRz", "global");
  fromRotVector(global_pose.rot, r);

  if(side == LEFT)
  {
    cam.pose = global_pose;
  }
  else  // side == RIGHT
  {
    right_pose.pos.x = file.GetFloat("Tx", "external");
    right_pose.pos.y = file.GetFloat("Ty", "external");
    right_pose.pos.z = file.GetFloat("Tz", "external");
    // SVS use mm, we use m
    right_pose.pos /= 1000.;
    r.x = file.GetFloat("Rx", "external");
    r.y = file.GetFloat("Ry", "external");
    r.z = file.GetFloat("Rz", "external");
    fromRotVector(right_pose.rot, r);
    // "external" is the pose of the left w.r.t. right camera, so need to
    // invert:
    inverse(right_pose, right_pose);
    // now get from relative right pose to global right pose
    transform(global_pose, right_pose, cam.pose);
  }
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

ostream& operator<<(ostream &os, const CameraParameters &cam)
{
  os << "w/h: "  << cam.width << " / " << cam.height
     << "  fx/fy: " << cam.fx << " / " << cam.fy
     << "  cx/cy: " << cam.cx << " / " << cam.cy
     << "  dist: " << cam.k1 << " " << cam.k2 << " " << cam.k3
     << " " << cam.p1 << " " << cam.p2
     << " pose: " << cam.pose;
  return os;
}

void changeImageSize(CameraParameters &cam, int newWidth, int newHeight)
{
  //assert(newWidth > 0);
  //assert(newHeight > 0);
  // adjust camera parameters for scaled images
  double sx = (double)newWidth/(double)cam.width;
  double sy = (double)newHeight/(double)cam.height;
  cam.width = newWidth;
  cam.height = newHeight;
  cam.fx *= sx;
  cam.cx *= sx;
  cam.fy *= sy;
  cam.cy *= sy;
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

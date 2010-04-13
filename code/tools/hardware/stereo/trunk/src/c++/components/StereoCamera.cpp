/**
 * @author Michael Zillich
 *
 * @version $Id: Camera.h,v 1.2 2009/01/09 17:21:28 mz Exp mz $
 */

#include <cassert>
#include <cmath>
#include <cfloat>
#include <sstream>
#include <iostream>
#include <opencv/cvaux.h>
#include "cogxmath.h"
#include "CDataFile.h"
#include "VideoUtils.h"
#include "StereoCamera.h"

namespace cast
{

using namespace std;
using namespace cogx;
using namespace cogx::Math;
using namespace Video;

static void ReadMat33(istream &s, double M[3][3])
{
  s >> M[0][0] >> M[0][1] >> M[0][2]
    >> M[1][0] >> M[1][1] >> M[1][2]
    >> M[2][0] >> M[2][1] >> M[2][2];
}

static void ReadMat34(istream &s, double M[3][4])
{
  s >> M[0][0] >> M[0][1] >> M[0][2] >> M[0][3]
    >> M[1][0] >> M[1][1] >> M[1][2] >> M[1][3]
    >> M[2][0] >> M[2][1] >> M[2][2] >> M[2][3];
}

StereoCamera::StereoCamera()
{
  maxDistortion = .5;
  mapx[LEFT] = mapx[RIGHT] = 0;
  mapy[LEFT] = mapy[RIGHT] = 0;
  sx = sy = 1.;
  inImgSize.width = 0;
  inImgSize.height = 0;
}

StereoCamera::~StereoCamera()
{
  cvReleaseImage(&mapx[LEFT]);
  cvReleaseImage(&mapx[RIGHT]);
  cvReleaseImage(&mapy[LEFT]);
  cvReleaseImage(&mapy[RIGHT]);
}

/**
 * Read calibration from a file generated by SVS.
 * NOTE: the 'frame' parameter in the SVS calibration file is ignored. It is
 * typically 1 anyway.
 */
void StereoCamera::ReadSVSCalib(const string &calibfile)
{
  CDataFile file(calibfile);

  for(int side = LEFT; side <= RIGHT; side++)
  {
    const char *side_str = side == LEFT ? "left camera" : "right camera";
    cam[side].width = file.GetInt("pwidth", side_str);
    cam[side].height = file.GetInt("pheight", side_str);
    cam[side].fx = file.GetFloat("f", side_str);
    cam[side].fy = file.GetFloat("fy", side_str);
    cam[side].cx = file.GetFloat("Cx", side_str);
    cam[side].cy = file.GetFloat("Cy", side_str);
    cam[side].k1 = file.GetFloat("kappa1", side_str);
    cam[side].k2 = file.GetFloat("kappa2", side_str);
    cam[side].k3 = file.GetFloat("kappa3", side_str);
    cam[side].t1 = file.GetFloat("tau1", side_str);
    cam[side].t2 = file.GetFloat("tau2", side_str);

    string str1 = file.GetString("proj", side_str);
    istringstream sstr1(str1);
    ReadMat34(sstr1, cam[side].proj);

    string str2 = file.GetString("rect", side_str);
    istringstream sstr2(str2);
    ReadMat33(sstr2, cam[side].rect);
  }

  // read poses
  // Note: an ideal camera is the camera associated with the ideal, i.e. rectified
  // image
  // We have:
  // - pose of the whole stereo rig, we call that the global rig pose
  //   (this->pose)
  // - pose of the left ideal camera relative to the rig (cam[LEFT]->pose), we assume
  //   this to be identity (as is quite common), so global rig pose and global
  //   ideal left camera pose are the same
  // - global pose of left ideal camera, same as global rig pose (see above).
  //   This is called "global" in the calibration file.
  // - pose of the right ideal camera relative to the rig (cam[RIGHT]->pose).
  //   This is stored implicitly in the right projection matrix Pr:
  //   relative right ideal pose is identity except x = -Pr(0,3)/Pr(0,0)
  //   (Note: What is called "external" in the calibration file (NOTE: Actually it is
  //   the inverse of "external"!) is the pose of the real (not ideal) right camera.)
  // - global pose of the right ideal camera, i.e. global rig pose + relative ideal
  //   right pose

  Vector3 r;
  pose.pos.x = file.GetFloat("GTx", "global");
  pose.pos.y = file.GetFloat("GTy", "global");
  pose.pos.z = file.GetFloat("GTz", "global");
  // SVS uses mm, we use m
  pose.pos /= 1000.;
  r.x = file.GetFloat("GRx", "global");
  r.y = file.GetFloat("GRy", "global");
  r.z = file.GetFloat("GRz", "global");
  fromRotVector(pose.rot, r);

  setIdentity(cam[LEFT].pose);

  setIdentity(cam[RIGHT].pose);
  cam[RIGHT].pose.pos.x = -cam[RIGHT].proj[0][3]/cam[RIGHT].proj[0][0];
  // SVS uses mm, we use m
  cam[RIGHT].pose.pos /= 1000.;

  /* Note: the following would get the real (not ideal) right camera pose
  cam[RIGHT].pose.pos.x = file.GetFloat("Tx", "external");
  cam[RIGHT].pose.pos.y = file.GetFloat("Ty", "external");
  cam[RIGHT].pose.pos.z = file.GetFloat("Tz", "external");
  // SVS use mm, we use m
  cam[RIGHT].pose.pos /= 1000.;
  r.x = file.GetFloat("Rx", "external");
  r.y = file.GetFloat("Ry", "external");
  r.z = file.GetFloat("Rz", "external");
  fromRotVector(cam[RIGHT].pose.rot, r);
  // "external" is the pose of the left w.r.t. right camera, so need to
  // invert:
  inverse(cam[RIGHT].pose, cam[RIGHT].pose);*/

  assert(cam[LEFT].width == cam[RIGHT].width);
  assert(cam[LEFT].height == cam[RIGHT].height);

  inImgSize.width = cam[LEFT].width;
  inImgSize.height = cam[LEFT].height;
  sx = 1.;
  sy = 1.;
}

/**
 * Point (X,Y,Z) is given in coord sys of left camera.
 */
void StereoCamera::ProjectPoint(double X, double Y, double Z,
    double &u, double &v, int side)
{
  assert(Z != 0.);
  // get from m to mm
  X *= 1000.;
  Y *= 1000.;
  Z *= 1000.;
  u = sx*cam[side].proj[0][0]*X + sx*cam[side].proj[0][2]*Z +
      sx*cam[side].proj[0][3];
  v = sy*cam[side].proj[1][1]*Y + sy*cam[side].proj[1][2]*Z;
  // w = Z;
  u /= Z;
  v /= Z;
}

/**
 * Given a point in the left image and its disparity, return the reconstructed
 * 3D point.
 */
void StereoCamera::ReconstructPoint(double u, double v, double d, double &X, double &Y, double &Z)
{
  // NOTE: actually tx = -proj[0][3]/proj[0][0] because:
  // proj[0][3] = -fx*tx  (where fx = proj[0][0])
  // but there seems to be an error in the SVS calib file:
  //   [external]
  //   Tx = -202.797
  //   [right camera]
  //   proj = 640 ... -1.297899e+05  (this should be positive!)
  // This should be further investigated!!!
  double tx = cam[RIGHT].proj[0][3]/cam[RIGHT].proj[0][0];
  X = u - sx*cam[LEFT].proj[0][2];
  Y = v - sy*cam[LEFT].proj[1][2];
  Z = sx*cam[LEFT].proj[0][0];
  double W = -d/tx + sx*(cam[LEFT].proj[0][2] - cam[RIGHT].proj[0][2])/tx;
  // SVS calibration uses mm, we want m -> divide by 1000
  W *= 1000.;
  X /= W;
  Y /= W;
  Z /= W;
}


void StereoCamera::DistortNormalisedPoint(double x, double y,
    double &xd, double &yd, int side)
{
  double x2 = x*x;
  double y2 = y*y;
  double r2 = x2 + y2;
  double r4 = r2*r2;
  double r6 = r4*r2;
  double t = (1. + cam[side].k1*r2 + cam[side].k2*r4 + cam[side].k3*r6);
  xd = x*t + 2.*cam[side].t1*x*y + cam[side].t2*(r2 + 2.*x2);
  yd = y*t + 2.*cam[side].t2*x*y + cam[side].t1*(r2 + 2.*y2);
}

void StereoCamera::DistortPoint(double u, double v, double &ud, double &vd,
    int side)
{
  double x = (u - sx*cam[side].cx)/(sx*cam[side].fx);
  double y = (v - sy*cam[side].cy)/(sy*cam[side].fy);
  double xd, yd;
  DistortNormalisedPoint(x, y, xd, yd, side);
  ud = xd*sx*cam[side].fx + sx*cam[side].cx;
  vd = yd*sy*cam[side].fy + sy*cam[side].cy;
}

/**
 * gradient based method for undistortion
 */
bool StereoCamera::UndistortPoint(double ud, double vd, double &u, double &v,
    int side)
{
  const unsigned MAX_ITER = 100;
  double error = DBL_MAX;
  double gradx = 0, grady = 0;
  double currentx, currenty;
  unsigned z;
  u = ud;
  v = vd;

  for(z = 0; z < MAX_ITER && error > maxDistortion; z++)
  {
    u += gradx;
    v += grady;
    DistortPoint(u, v, currentx, currenty, side);
    gradx = ud - currentx;
    grady = vd - currenty;
    error = gradx*gradx + grady*grady;
  }
  if(u < 0)
    u = 0;
  if(u >= inImgSize.width)
    u = inImgSize.width - 1;
  if(v < 0)
    v = 0;
  if(v >= inImgSize.height)
    v = inImgSize.height - 1;
  if(z < MAX_ITER)
    return true;
  return false;
}

void StereoCamera::RectifyPoint(double ud, double vd, double &ur, double &vr,
    int side)
{
  double u, v;
  UndistortPoint(ud, vd, u, v, side);
  double x = (u - sx*cam[side].cx)/(sx*cam[side].fx);
  double y = (v - sy*cam[side].cy)/(sy*cam[side].fy);
  double xr = cam[side].rect[0][0]*x + cam[side].rect[0][1]*y +
    cam[side].rect[0][2];
  double yr = cam[side].rect[1][0]*x + cam[side].rect[1][1]*y +
    cam[side].rect[1][2];
  double wr = cam[side].rect[2][0]*x + cam[side].rect[2][1]*y +
    cam[side].rect[2][2];
  xr /= wr;
  yr /= wr;
  ur = xr*sx*cam[side].proj[0][0] + sx*cam[side].proj[0][2];
  vr = yr*sy*cam[side].proj[1][1] + sy*cam[side].proj[1][2];
}

void StereoCamera::UnrectifyPointFast(double ur, double vr, double &ud, double &vd,
    int side)
{
  int ui = (int)floor(ur), vi =  (int)floor(vr);
  if(ui >= 0 && ui < inImgSize.width && vi >= 0 && vi < inImgSize.height)
  {
    ud = *(float*)cvAccessImageData(mapx[side], ui, vi);
    vd = *(float*)cvAccessImageData(mapy[side], ui, vi);
  }
  else
  {
    ud = vd = 0.;
  }
}

void StereoCamera::SetupImageRectification()
{
  for(int side = LEFT; side <= RIGHT; side++)
  {
    CvMat R = cvMat(3, 3, CV_64FC1, cam[side].rect);
    // inverse of rectification matrix: xu = R_i * xi (with xu and xi
    // homogeneous)
    double r_i[3][3];
    CvMat R_i = cvMat(3, 3, CV_64FC1, r_i);
    cvInvert(&R, &R_i);
    // make sure this function is only called once
    assert(mapx[side] == 0);
    mapx[side] = cvCreateImage(inImgSize, IPL_DEPTH_32F, 1);
    mapy[side] = cvCreateImage(inImgSize, IPL_DEPTH_32F, 1);
    //cvSetZero(mapx[side]);
    //cvSetZero(mapy[side]);
    // iterate over ideal image coords
    for(int vi = 0; vi < inImgSize.height; vi++)
    {
      for(int ui = 0; ui < inImgSize.width; ui++)
      {
        // ideal normalised coords
        double xi = (ui - sx*cam[side].proj[0][2])/(sx*cam[side].proj[0][0]);
        double yi = (vi - sy*cam[side].proj[1][2])/(sy*cam[side].proj[1][1]);
        // undistorted normalised coords
        double xu = r_i[0][0]*xi + r_i[0][1]*yi + r_i[0][2];
        double yu = r_i[1][0]*xi + r_i[1][1]*yi + r_i[1][2];
        double wu = r_i[2][0]*xi + r_i[2][1]*yi + r_i[2][2];
        assert(!iszero(wu));
        xu /= wu;
        yu /= wu;
        // distorted normalised coords
        double xd, yd;
        DistortNormalisedPoint(xu, yu, xd, yd, side);
        // distorted image coords
        double ud = xd*sx*cam[side].fx + sx*cam[side].cx;
        double vd = yd*sy*cam[side].fy + sy*cam[side].cy;
        *(float*)cvAccessImageData(mapx[side], ui, vi) = ud;
        *(float*)cvAccessImageData(mapy[side], ui, vi) = vd;
      }
    }
    //cvDeleteMoire(mapx[side]);
    //cvDeleteMoire(mapy[side]);
  }
}

void StereoCamera::RectifyImage(const IplImage *src, IplImage *dst, int side)
{
  assert(src != 0 && dst != 0);
  cvRemap(src, dst, mapx[side], mapy[side],
          CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll(0));
}

void StereoCamera::DisparityImage(const IplImage *left, const IplImage *right,
    IplImage *disp)
{
  const int max_disparity = 64;
  assert(left != 0 && right != 0 && disp != 0);
  cvFindStereoCorrespondence(left, right,
                             CV_DISPARITY_BIRCHFIELD,  // mode
                             disp,
                             max_disparity,
                             15, 3, 6, 8, 15 );
                             //25,  // param1
                             //5,   // param2
                             //12,  // param3
                             //15,  // param4
                             //25   // param5
                             //);

}

void StereoCamera::SetInputImageSize(CvSize size)
{
  inImgSize = size;
  // NOTE: I assume that of course left and right camera have the same
  // width/height
  sx = (double)inImgSize.width/(double)cam[LEFT].width;
  sy = (double)inImgSize.height/(double)cam[LEFT].height;
  cout << "StereoCamera: sx sy " << sx << " " << sy << endl;
}

}


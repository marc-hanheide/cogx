/**
 * @author Michael Zillich
 *
 * @version $Id: Camera.h,v 1.2 2009/01/09 17:21:28 mz Exp mz $
 */

#ifndef STEREO_CAMERA_H
#define STEREO_CAMERA_H

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cogxmath.h>

namespace cast
{

using namespace std;

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
  enum MatchingAlgorithm {BLOCK_MATCH, SEMI_GLOBAL_BLOCK_MATCH, GRAPH_CUT};

  class MonoParam
  {
  public:
    int width, height;
    double fx, fy, cx, cy;
    double k1, k2, k3, t1, t2;
    double proj[3][4];
    double rect[3][3];
    cogx::Math::Pose3 pose;
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
  cogx::Math::Pose3 pose;
  /// remaping images for simultaneous undistortion and rectification
  IplImage *mapx[2], *mapy[2];
  double maxDistortion;
  // size of input images, can differ from size of calibration images!
  CvSize inImgSize;
  // scaling parameters in case input image size differs from calibration image
  // size
  double sx;
  double sy;
  MatchingAlgorithm matchAlgorithm;
  CvStereoBMState *stereo_bm_state;
  cv::StereoBM *stereoBM;
  cv::StereoSGBM *stereoSGBM;

public:
  StereoCamera();
  ~StereoCamera();
  void ReadFromXML(const string &filename, int side, bool usePose);
  bool ReadSVSCalib(const std::string &calibfile);
  void ProjectPoint(double X, double Y, double Z, double &u, double &v, int side, int imgWidth = 0);
  bool ReconstructPoint(double u, double v, double d, double &X, double &Y, double &Z);
  void DistortNormalisedPoint(double u, double d, double &ud, double &vd, int side);
  void DistortPoint(double u, double d, double &ud, double &vd, int side);
  bool UndistortPoint(double ud, double vd, double &u, double &v,int side);
  void SetMaxDistortion(double err = .5) {maxDistortion = err;}
  void RectifyPoint(double ud, double vd, double &ur, double &vr, int side);
  void UnrectifyPointFast(double ur, double vr, double &ud, double &vd, int side);
  void SetupImageRectification();
  void RectifyImage(const IplImage *src, IplImage *dst, int side);
  void SetDisparityRange(int minDisp, int maxDisp);
  void SetInputImageSize(CvSize size);
  void SetMatchingAlgoritm(MatchingAlgorithm algo);
  void CalculateDisparity(const IplImage *left, const IplImage *right, IplImage *disp);
  cv::Mat GetIntrinsic(unsigned side);
};

}

#endif


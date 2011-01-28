/**
 * @file StereoCamera.hh
 * @author Michael Zillich
 * @date 2009
 * @version 0.1
 * @brief StereoCamera calculation class
 */

#ifndef STEREO_CAMERA_HH
#define STEREO_CAMERA_HH

#include "Vector.hh"
#include "Math.hh"
#include <opencv/cv.h>
#include <opencv/cv.hpp>

namespace Z
{

// #define LEFT 0
// #define RIGHT 1

/**
 * @brief Camera class for stereo camera calculations.
 *
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
  
  /**
    * @brief Mono parameter class for a single camera.
    */
  class MonoParam
  {
  public:
    int width, height;			///< Width and heigt of the image in pixels.
    double fx, fy, cx, cy;		///< focal length and principal points
    double k1, k2, k3, t1, t2;		///< distortion parameters
    double proj[3][4];			///< projection matrix
    double rect[3][3];			///< rectification matrix
    Pose3 pose;				///< TODO ???

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
    } ///< Constructor of class MonoParam.
  }; 

  MonoParam cam[2];			///< parameters specific to LEFT and RIGHT camera
  Pose3 pose;
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
  bool ReadSVSCalib(const string &calibfile);
  void ProjectPoint(double X, double Y, double Z, double &u, double &v, int side);
  void ReconstructPoint(double u, double v, double d, double &X, double &Y, double &Z);
  void DistortNormalisedPoint(double u, double d, double &ud, double &vd, int side);
  void DistortPoint(double u, double d, double &ud, double &vd, int side);
  bool UndistortPoint(double ud, double vd, double &u, double &v,int side);
  void SetMaxDistortion(double err=.5){maxDistortion=err;}
  void RectifyPoint(double u, double v, double &ur, double &vr, int side);
  void UnrectifyPointFast(double ur, double vr, double &ud, double &vd, int side);
  void SetupImageRectification();
  void RectifyImage(const IplImage *src, IplImage *dst, int side);
  void SetDisparityRange(int minDisp, int maxDisp);
  void SetInputImageSize(CvSize size);
  void SetMatchingAlgoritm(MatchingAlgorithm algo);
  void CalculateDisparity(const IplImage *left, const IplImage *right, IplImage *disp);
};



/**
 * Access raw image data at position (x,y), where x and y are not checked for valid range.
 * const version.
 */
inline const char* cvAccessImageData(const IplImage *img, int x, int y)
{
  return img->imageData + y*img->widthStep + x*img->nChannels*img->depth/8;
}

/**
 * similar to isnan() or isinf(), check a float for being zero
 * Note: This is safer than testing for == REAL_ZERO as +0.0 and -0.0 are
 * distinct values.
 * Note 2: It seems however that still +0.0 == -0.0 is true.
 * Note 3: Anyway.
 */
inline bool iszero(float a)
{
  return std::fpclassify(a) == FP_ZERO;
}
}

#endif


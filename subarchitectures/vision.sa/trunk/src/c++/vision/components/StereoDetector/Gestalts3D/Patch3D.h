/**
 * @file Patch3D.h
 * @author Andreas Richtsfeld
 * @date June 2011
 * @version 0.1
 * @brief Base class for plane patches in 3D.
 */

#ifndef Z_PATCH3D_HH
#define Z_PATCH3D_HH

#include <vector>
#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"
// #include "VisionUtils.h"
#include "ColorHistogram.h"

#include "highgui.h"
// #include <opencv2/highgui/highgui.hpp>

namespace Z
{

/**
 * @brief Class Patch3D
 */
class Patch3D : public Gestalt3D
{
private:
 cv::Point3f normal;                      ///< Normal of the plane
 cv::Point3f center3D;                    ///< Center point of the 3D patch
 std::vector<cv::Vec4f> points;           ///< All (projected) points of the plane patch
 std::vector<cv::Vec4f> hull_points;      ///< Hull points of the plane
 
 unsigned oLabel;                         ///< Object label for learning objects
 
 void CalculatePlaneNormal();

public:
 ColorHistogram *hist;                    ///< ColorHistogram of patch

public:
//   Patch3D(cv::Mat_<cv::Vec4f> _p, cv::Mat_<cv::Vec4f> _h_p);
  Patch3D(std::vector<cv::Vec4f> _p, std::vector<cv::Vec4f> _h_p);

  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);
  bool GetLinks(vector<GraphLink> &links);
  cv::Point3f GetCenter3D() {return center3D;}
  cv::Point3f GetPlaneNormal() {return normal;}
  void GetCenter3D(cv::Point3f &c) {c = center3D;}
  
  // Learning functions
  double IsClose(Patch3D *p);
  double CompareColor(Patch3D *p);
  void CalculateCoplanarity(Patch3D *p, double &normal_angle, double &plane_distance);
  
  void DrawGestalt3D(TGThread::TomGineThread *tgRenderer, bool randomColor) {DrawGestalt3D(tgRenderer, false, 0);}
  void DrawGestalt3D(TGThread::TomGineThread *tgRenderer, bool use_color = false, float color = 0.0);
  void DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, Video::CameraParameters camPars);
  
  void PrintGestalt3D();
  
  void SetObjectLabel(const unsigned i) {oLabel = i;}
  unsigned GetObjectLabel() {return oLabel;}
};


inline Array<Gestalt3D*>& Patches3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::PATCH);
}
inline Patch3D* Patches3D(StereoCore *score, unsigned id)
{
  return (Patch3D*)score->Gestalts3D(Gestalt3D::PATCH, id);
}
inline unsigned NumPatches3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::PATCH);
}


}

#endif

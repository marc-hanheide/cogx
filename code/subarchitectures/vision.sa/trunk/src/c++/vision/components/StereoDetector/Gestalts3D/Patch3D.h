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
#include <math.h>
#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"
#include "ColorHistogram.h"

#include "highgui.h"

namespace Z
{

/**
 * @brief Class Patch3D
 */
class Patch3D : public Gestalt3D
{
private:
  std::vector<cv::Vec4f> points;            ///< All (projected) points of the plane patch
  std::vector<int> indexes;                 ///< 2D indexes of the points (in image space)
  std::vector<cv::Vec4f> hull_points;       ///< Hull points of the plane
  cv::Point3f normal;                       ///< Normal of the plane                                /// TODO Ändern auf Vec3f ???
  cv::Vec3f center3D;                       ///< Center point of the 3D patch
  double radius;                            ///< Maximum distance from center3D to hull_point       // TODO Wieder weg???
  
  void CalculatePlaneNormal();
  double DistancePoint2Plane(double a, double b, double c, double d, 
                             double x, double y, double z);

public:
 ColorHistogram *hist;                    ///< ColorHistogram of patch

public:
  Patch3D(std::vector<cv::Vec4f> _points, std::vector<int> _indexes, std::vector<cv::Vec4f> _hull_points);

  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);
  bool GetLinks(vector<GraphLink> &links);
  cv::Vec3f GetCenter3D() {return center3D;}
  cv::Point3f GetPlaneNormal() {return normal;}
  void GetCenter3D(cv::Vec3f &c) {c = center3D;}
  std::vector<cv::Vec4f> GetHullPoints() {return hull_points;}
  double GetRadius() {return radius;}
  void GetIndexes(std::vector<int> &_indexes) {_indexes = indexes;}

  // Learning functions
  double CalculateProximity(Patch3D *p);
  double CompareColor(Patch3D *p);
  void CalculateCoplanarity2(Patch3D *p, double &n0n1, double &ppn0, double &ppn1);
  void CalculateCoplanarity(Patch3D *p, double &normal_angle, double &plane_distance);
  double CalculatePointDistance(double x, double y, double z);
  double CalculateParallelity(cv::Point3f &dir);
  
  // Drawing functions
  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color = false, float color = 0.0);
  void DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, Video::CameraParameters camPars);
  
  void PrintGestalt3D();
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

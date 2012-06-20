/**
 * @file Segment3D.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Base class for segments in 3D.
 */

#ifndef Z_SEGMENT3D_HH
#define Z_SEGMENT3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

#include "../../../VisionUtils.h"

namespace Z
{

/**
 * @brief Class Segment3D
 */
class Segment3D : public Gestalt3D
{
private:
  std::vector<cv::Vec4f> points;              ///< Points of the edge in 3D (with color).
  std::vector<int> indexes;                   ///< Indexes of the points, refering to the image space.
  std::vector<cv::Vec4f> edge_support;        ///< Support of color-, depth-, mask-, curvature-edges
  cv::Vec3f point[2];                         ///< Start/End point of the segment in 3D
  cv::Vec3f center3D;                         ///< 3D center point

  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);

public:
  Segment3D(unsigned _vs3ID, 
            std::vector<cv::Vec4f> &_points,
            std::vector<int> &_indexes,
            std::vector<cv::Vec4f> &_es);
  Segment3D(unsigned vs3IDleft, unsigned vs3IDright);
  
  bool GetLinks(vector<GraphLink> &links);
  cv::Vec3f GetCenter3D() {return center3D;}
  void GetCenter3D(cv::Vec3f &c) {c = center3D;}
  cv::Vec3f GetStartPoint() {return point[0];}
  cv::Vec3f GetEndPoint() {return point[0];}
  void GetPoints(std::vector<cv::Vec4f> &p) {p = points;}
  double GetEdgeSupportDepth(int idx) {return (double) edge_support[idx][1];}
  double GetEdgeSupportMask(int idx) {return (double) edge_support[idx][2];}
  double GetEdgeSupportCurvature(int idx) {return (double) edge_support[idx][3];}
  void GetIndexes(std::vector<int> &_indexes) {_indexes = indexes;}
  
  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color = false, float color = 0.0);
  void DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, Video::CameraParameters &camPars);
  void PrintGestalt3D();
};


inline Array<Gestalt3D*>& Segments3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::SEGMENT);
}
inline Segment3D* Segments3D(StereoCore *score, unsigned id)
{
  return (Segment3D*)score->Gestalts3D(Gestalt3D::SEGMENT, id);
}
inline unsigned NumSegments3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::SEGMENT);
}


}

#endif

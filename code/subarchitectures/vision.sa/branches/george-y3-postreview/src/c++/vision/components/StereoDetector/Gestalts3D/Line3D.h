/**
 * @file Line3D.h
 * @author Andreas Richtsfeld
 * @date May 2011
 * @version 0.1
 * @brief Base class for stereo calculated line in 3D.
 */

#ifndef Z_LINE3D_HH
#define Z_LINE3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

#include "../../../VisionUtils.h"
#include "v4r/TomGine/tgTomGineThread.h"

namespace Z
{

/**
 * @brief Class Line3D
 */
class Line3D : public Gestalt3D
{
public:
  cv::Vec4f point[2];                   ///< Start/End point of the line in 3D (with color).
  cv::Vec3f center3D;                 ///< 3D center point of the line
  cv::Point3f dir;                      ///< Direction of the line
  std::vector<cv::Vec4f> edge;          ///< Edge of the line (Segment3D edgels)

  Vertex3D isct3D[2];                    ///< 3D intersection point [START/END]   // TODO Which intersection?
//  Vertex3D armPoints3D[2];            ///< 3D arm points
//  Vector3 armDir3D[2];                ///< 3D direction of the 3 arms of the L-Junction

  Line3D(unsigned _vs3ID, std::vector<cv::Vec4f> &_p);
  Line3D(unsigned vs3IDleft, unsigned vs3IDright);
  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);
  
  bool GetLinks(vector<GraphLink> &links);
  cv::Vec3f GetCenter3D() {return center3D;}
  void GetCenter3D(cv::Point3f &c) {c = center3D;}
  cv::Point3f GetDirection() {return dir;}

  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color = false, float color = 0.0);
  void DrawGestalts3DToImage(cv::Mat_<cv::Vec3b> &image, Video::CameraParameters &camPars);
  void PrintGestalt3D();

  double LLProximity(Line3D *l);
  double LLParallelity(Line3D *l);
};


inline Array<Gestalt3D*>& Lines3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::LINE);
}
inline Line3D* Lines3D(StereoCore *score, unsigned id)
{
  return (Line3D*)score->Gestalts3D(Gestalt3D::LINE, id);
}
inline unsigned NumLines3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::LINE);
}


}

#endif

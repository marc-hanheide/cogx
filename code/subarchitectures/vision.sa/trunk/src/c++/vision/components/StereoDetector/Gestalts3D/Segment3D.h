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

namespace Z
{

/**
 * @brief Class Line3D
 */
class Segment3D : public Gestalt3D
{
public:
  std::vector<cv::Vec4f> points;               ///< Points of the edge in 3D (with color).
//  Vertex3D point[2];                      ///< Start/end point of the 3D line
  
//  Vertex3D isct3D[2];                     ///< 3D intersection point [START/END]
//  Vertex3D armPoints3D[2];             ///< 3D arm points
//  Vector3 armDir3D[2];                 ///< 3D direction of the 3 arms of the L-Junction

  Segment3D(unsigned _vs3ID, std::vector<cv::Vec4f> &_p);
  Segment3D(unsigned vs3IDleft, unsigned vs3IDright);
  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);
  
  bool GetLinks(vector<GraphLink> &links);
  
  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor = true);
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

/**
 * @file Collinearity3D.h
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
#include "Line3D.h"

namespace Z
{

/**
 * @brief Class Collinearity3D
 */
class Collinearity3D : public Gestalt3D
{
public:
  Line3D *line[2];                      ///< Left/right line of collinearity (not same than kcore->Line3D!)
  cv::Vec4f point[3];                   ///< Start/Intersection/End point of the collinearity in 3D (with color).   /// TODO not yet implemented!

  
  std::vector<cv::Vec4f> points;        ///< Points of the edge in 3D (with color).
//  Vertex3D point[2];                      ///< Start/end point of the 3D line
  
//  Vertex3D isct3D[2];                     ///< 3D intersection point [START/END]
//  Vertex3D armPoints3D[2];             ///< 3D arm points
//  Vector3 armDir3D[2];                 ///< 3D direction of the 3 arms of the L-Junction

  Collinearity3D(unsigned _vs3ID, Z::Line3D *_line[2]);
  Collinearity3D(unsigned vs3IDleft, unsigned vs3IDright);
  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);
  
  bool GetLinks(vector<GraphLink> &links);
  
  void DrawGestalt3D(TomGine::tgTomGineThread *tgRenderer, bool randomColor, bool use_color = false, float color = 0.0);
  
  void PrintGestalt3D();
};


inline Array<Gestalt3D*>& Collinearities3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::COLLINEARITY);
}
inline Collinearity3D* Collinearities3D(StereoCore *score, unsigned id)
{
  return (Collinearity3D*)score->Gestalts3D(Gestalt3D::COLLINEARITY, id);
}
inline unsigned NumCollinearities3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::COLLINEARITY);
}


}

#endif

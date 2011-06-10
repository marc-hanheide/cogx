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

namespace Z
{

/**
 * @brief Class Line3D
 */
class Line3D : public Gestalt3D
{
public:
  cv::Vec4f point[2];                   ///< Start/End point of the line in 3D (with color).
  std::vector<cv::Vec4f> edge;          ///< Edge of the line (Segment3D edgels)

//  Vertex3D point[2];                      ///< Start/end point of the 3D line

 Vertex3D isct3D[2];                    ///< 3D intersection point [START/END]   // TODO Which intersection?
//  Vertex3D armPoints3D[2];            ///< 3D arm points
//  Vector3 armDir3D[2];                ///< 3D direction of the 3 arms of the L-Junction

  Line3D(unsigned _vs3ID, std::vector<cv::Vec4f> &_p);
  Line3D(unsigned vs3IDleft, unsigned vs3IDright);
  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);
  
  bool GetLinks(vector<GraphLink> &links);

  void DrawGestalt3D(TGThread::TomGineThread *tgRenderer, bool randomColor = true);
  void PrintGestalt3D();

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

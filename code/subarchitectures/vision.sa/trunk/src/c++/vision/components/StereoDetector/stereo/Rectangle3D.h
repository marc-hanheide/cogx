/**
 * @file Rectangle3D.h
 * @author Andreas Richtsfeld
 * @date Januray 2011
 * @version 0.1
 * @brief Base class for stereo calculated Rectangles in 3D.
 */

#ifndef Z_RECTANGLE3D_HH
#define Z_RECTANGLE3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.hh"

namespace Z
{

/**
 * @brief Class Rectangle3D
 */
class Rectangle3D : public Gestalt3D
{
public:
  Surf3D surf;            ///< 3D surface

public:
  Rectangle3D();
  void CalculateSignificance(double sigLeft, double sigRight);
  unsigned GetSurfaceSize() {return surf.vertices.Size();}
  bool GetLinks(vector<GraphLink> &links);
};


inline Array<Gestalt3D*>& Rectangles3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::RECTANGLE);
}
inline Rectangle3D* Rectangles3D(StereoCore *score, unsigned id)
{
  return (Rectangle3D*)score->Gestalts3D(Gestalt3D::RECTANGLE, id);
}
inline unsigned NumRectangles3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::RECTANGLE);
}


}

#endif

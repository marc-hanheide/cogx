/**
 * @file Ellipse3D.h
 * @author Andreas Richtsfeld
 * @date February 2011
 * @version 0.1
 * @brief Base class for stereo calculated ellipses in 3D.
 */

#ifndef Z_ELLIPSE3D_HH
#define Z_ELLIPSE3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

namespace Z
{

/**
 * @brief Class Ellipse3D
 */
class Ellipse3D : public Gestalt3D
{
private:
 Vertex3D center;                                       ///< 3D center point of the circle with the normal
 double radius;                                         ///< Radius of the circle

public:
  
private:
  
public:
  Ellipse3D(unsigned vs3ID_l, unsigned vs3ID_r, Vertex3D c, double r, double s);
  double Compare(Ellipse3D &ell);
  
  Vertex3D GetCenter() {return center;}
  double GetRadius() {return radius;}
};


inline Array<Gestalt3D*>& Ellipses3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::ELLIPSE);
}
inline Ellipse3D* Ellipses3D(StereoCore *score, unsigned id)
{
  return (Ellipse3D*)score->Gestalts3D(Gestalt3D::ELLIPSE, id);
}
inline unsigned NumEllipses3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::ELLIPSE);
}


}

#endif

/**
 * @file Corner3D.h
 * @author Andreas Richtsfeld
 * @date Januray 2011
 * @version 0.1
 * @brief Base class for stereo calculated corners in 3D.
 */

#ifndef Z_CORNER3D_HH
#define Z_CORNER3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

namespace Z
{

/**
 * @brief Class Corner3D
 */
class Corner3D : public Gestalt3D
{
private:
  Vertex3D isct3D;                     ///< 3D intersection point
  
public:
  Vertex3D armPoints3D[3];             ///< 3D arm points
  Vector3 armDir3D[3];                 ///< 3D direction of the 3 arms of the L-Junction

  Corner3D(Vertex3D is);
  Vertex3D GetIsct3D() {return isct3D;}
};


inline Array<Gestalt3D*>& Corners3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::CORNER);
}
inline Corner3D* Corners3D(StereoCore *score, unsigned id)
{
  return (Corner3D*)score->Gestalts3D(Gestalt3D::CORNER, id);
}
inline unsigned NumCorners3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::CORNER);
}


}

#endif

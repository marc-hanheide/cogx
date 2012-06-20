/**
 * @file LJunction3D.h
 * @author Andreas Richtsfeld
 * @date Januray 2011
 * @version 0.1
 * @brief Base class for stereo calculated l-junctions in 3D.
 */

#ifndef Z_LJUNCTION3D_HH
#define Z_LJUNCTION3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

namespace Z
{

/**
 * @brief Class Corner3D
 */
class LJunction3D : public Gestalt3D
{
private:
  
  
public:
  Vertex3D isct3D;                     ///< 3D intersection point
//  Vertex3D armPoints3D[2];             ///< 3D arm (end?) points				/// TODO What points???
//  Vector3 armDir3D[2];                 ///< 3D direction of the arms of the L-Junction

  LJunction3D(Vertex3D i);
};


inline Array<Gestalt3D*>& LJunctions3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::LJUNCTION);
}
inline LJunction3D* LJunctions3D(StereoCore *score, unsigned id)
{
  return (LJunction3D*)score->Gestalts3D(Gestalt3D::LJUNCTION, id);
}
inline unsigned NumLJunctions3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::LJUNCTION);
}


}

#endif

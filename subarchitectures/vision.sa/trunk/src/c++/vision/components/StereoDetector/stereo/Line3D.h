/**
 * @file Line3D.h
 * @author Andreas Richtsfeld
 * @date February 2011
 * @version 0.1
 * @brief Base class for stereo calculated line in 3D.
 */

#ifndef Z_LINE3D_HH
#define Z_LINE3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.hh"

namespace Z
{

/**
 * @brief Class Line3D
 */
class Line3D : public Gestalt3D
{
public:
 Vertex3D isct3D[2];                     ///< 3D intersection point [START/END]
//  Vertex3D armPoints3D[2];             ///< 3D arm points
//  Vector3 armDir3D[2];                 ///< 3D direction of the 3 arms of the L-Junction

  Line3D(unsigned vs3IDleft, unsigned vs3IDright);
  void CalculateSignificance(double angle2Dleft, double angle2Dright, double angle3Dz);
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

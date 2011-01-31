/**
 * @file Closure3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Closures in 3D.
 */

#ifndef Z_CLOSURE3D_HH
#define Z_CLOSURE3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.hh"

namespace Z
{

/**
 * @brief Class Closure3D
 */
class Closure3D : public Gestalt3D
{
public:
  Surf3D surf;                                                  ///< 3D surface

public:
  Closure3D();
  void CalculateSignificance(double sigLeft, double sigRight);
};


inline Array<Gestalt3D*>& Closures3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::CLOSURE);
}
inline Closure3D* Closures3D(StereoCore *score, unsigned id)
{
  return (Closure3D*)score->Gestalts3D(Gestalt3D::CLOSURE, id);
}
inline unsigned NumClosures3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::CLOSURE);
}


}

#endif

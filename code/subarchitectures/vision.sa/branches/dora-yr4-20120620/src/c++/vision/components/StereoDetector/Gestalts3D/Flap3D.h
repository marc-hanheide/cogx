/**
 * @file Flap3D.h
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Flaps in 3D.
 */

#ifndef Z_FLAP3D_HH
#define Z_FLAP3D_HH

#include "Gestalt3D.h"
#include "StereoTypes.h"
#include "StereoCore.h"

namespace Z
{

/**
 * @brief Class Flap3D
 */
class Flap3D : public Gestalt3D
{
private:
  bool pointsCalculated;        ///< True if CalcIdealFlap was already called
  Vertex3D point[6];            ///< The six corner points of the flap (0 and 3 are joint points, the rest clockwise)

public:
  Surf3D surf[2];               ///< The two matched 3D surfaces

  Flap3D();
  void CalcIdealFlap();
  bool IsCalcIdealFlap() {return pointsCalculated;}
  bool GetPoints(Vertex3D p[6]);
  bool GetLinks(std::vector<GraphLink> &links);
};


inline Array<Gestalt3D*>& Flaps3D(StereoCore *score)
{
  return score->Gestalts3D(Gestalt3D::FLAP);
}
inline Flap3D* Flaps3D(StereoCore *score, unsigned id)
{
  return (Flap3D*)score->Gestalts3D(Gestalt3D::FLAP, id);
}
inline unsigned NumFlaps3D(StereoCore *score)
{
  return score->NumGestalts3D(Gestalt3D::FLAP);
}


}

#endif

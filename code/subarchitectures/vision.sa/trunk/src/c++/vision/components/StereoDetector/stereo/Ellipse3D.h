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
#include "StereoCore.hh"

namespace Z
{

/**
 * @brief Class Ellipse3D
 */
class Ellipse3D : public Gestalt3D
{
private:
//  int tmpID;                                             ///< Position of the 2D features in ellipses[2]				// TODO TODO überall integrieren, zum nachverfolgen?
//  Vertex3D center;                                       ///< 3D center point of the circle with the normal
//  double radius;                                         ///< Radius of the circle
//  double significance;                                   ///< Significance value of the estimated circle
//  Vertex2D leftHullPoint[6];                             ///< The solutions (x[i][j], y_g[i]) (i=0,..,6 / j=0,1)
//  bool reliableCPoint[6];                                ///< Which circle points are reliable
//  Vertex3D cPoints[6];                                   ///< Real circle points in 3D
//  double distance[6];                                    ///< Distance between center and cPoints
//  double deviation[6];                                   ///< Deviation between distance and mean (radius)
//  unsigned nrCPointsCalc;                                ///< Number of calculated circle points (cPointsCalc)
//  Vertex3D cPointsCalc[20];                              ///< Calculated points on a "real" circle.

public:
  unsigned vs3ID[2];                      ///< The vs3-IDs of the Gestalts
  
private:	

public:
  Ellipse3D();
  double Compare(Ellipse3D &ell);
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

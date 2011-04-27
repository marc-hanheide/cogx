/**
 * @file Ellipse3D.cpp
 * @author Andreas Richtsfeld
 * @date February 2011
 * @version 0.1
 * @brief Base class for stereo calculated ellipses in 3D.
 */

#include "Ellipse3D.h"
#include "Ellipse.hh"

namespace Z
{

/**
 * @brief Constructor of class Line3D.
 * @param vs3ID_l ID of the left vs3 ellipse.
 * @param vs3ID_r ID of the right vs3 ellipse.
 * @param c Center point with position and normal
 * @param r radius
 * @param s significance
 */
Ellipse3D::Ellipse3D(unsigned vs3ID_l, unsigned vs3ID_r, Vertex3D c, double r, double s) : Gestalt3D(Gestalt3D::ELLIPSE)
{
  vs3IDs[LEFT] = vs3ID_l; 
  vs3IDs[RIGHT] = vs3ID_r;
  center = c;
  radius = r;
  sig = s;
}



/**
 * @brief Compare two ellipses for filtering.
 * @param ell 3D ellipse to compare
 * @return Return certainty value for equality [0,100]
 */
double Ellipse3D::Compare(Ellipse3D &ell)
{
  printf("Ellipse3D::Compare: Not yet implemented!\n");
  /// use center and radius
  
  /// distance between center points
//   double dist = Length(center.p - ell.center.p) + fabs(radius-ell.radius);
  
// 	printf("  Ellipse3D::Compare: dist: %4.2f\n", dist);
  
//   return dist;
  return 0;
}

}



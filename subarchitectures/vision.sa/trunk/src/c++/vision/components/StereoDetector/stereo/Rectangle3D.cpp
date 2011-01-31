/**
 * @file Rectangle3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Rectangles in 3D.
 */

#include "Rectangle3D.h"

namespace Z
{

/**
 * @brief Constructor of Class Rectangle3D
 */
Rectangle3D::Rectangle3D() : Gestalt3D(Gestalt3D::RECTANGLE)
{
}

void Rectangle3D::CalculateSignificance(double sigLeft, double sigRight)
{
  sig = sigLeft * sigRight;
//   printf(" Sigs: %4.1f/%4.1f => sig: %4.1f\n", sigLeft, sigRight, sig);
}


}



/**
 * @file Closure3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Closures in 3D.
 */

#include "Closure3D.h"

namespace Z
{

/**
 * @brief Constructor of Class Closure3D
 */
Closure3D::Closure3D() : Gestalt3D(Gestalt3D::CLOSURE)
{
}

void Closure3D::CalculateSignificance(double sigLeft, double sigRight)
{
  sig = sigLeft * sigRight;
//   printf(" Sigs: %4.1f/%4.1f => sig: %4.1f\n", sigLeft, sigRight, sig);
}


}



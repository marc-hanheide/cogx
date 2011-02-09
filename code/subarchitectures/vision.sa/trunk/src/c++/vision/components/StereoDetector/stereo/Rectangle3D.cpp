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

/**
 * @brief Calculate significance value for the new estimated stereo Gestalt.
 * @param sigLeft Significance value of the Gestalt from the left image.
 * @param sigRight Significance value of the Gestalt from the right image.
 */
void Rectangle3D::CalculateSignificance(double sigLeft, double sigRight)
{
  sig = sigLeft * sigRight;
}


}



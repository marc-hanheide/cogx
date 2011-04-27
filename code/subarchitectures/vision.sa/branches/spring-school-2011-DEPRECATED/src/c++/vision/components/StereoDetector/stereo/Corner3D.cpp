/**
 * @file Corner3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Corners in 3D.
 */

#include "Corner3D.h"
#include "Corner.hh"

namespace Z
{

/**
 * @brief Constructor of class Corner3D.
 */
Corner3D::Corner3D(Vertex3D is) : Gestalt3D(Gestalt3D::CORNER)
{
  isct3D = is;
}

}



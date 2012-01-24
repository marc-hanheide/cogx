/**
 * @file Line3D.cpp
 * @author Andreas Richtsfeld
 * @date February 2011
 * @version 0.1
 * @brief Base class for stereo calculated line in 3D.
 */

#include "LJunction3D.h"
#include "LJunction.hh"

namespace Z
{

/**
 * @brief Constructor of class LJunction3D.
 */
LJunction3D::LJunction3D(Vertex3D i) : Gestalt3D(Gestalt3D::LJUNCTION)
{
  isct3D = i;
}

}



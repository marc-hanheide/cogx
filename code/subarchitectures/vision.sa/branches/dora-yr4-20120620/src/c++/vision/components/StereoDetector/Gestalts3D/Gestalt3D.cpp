/**
 * @file Gestalt3D.cpp
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Base class for stereo calculated Gestalts in 3D.
 */

#include "Gestalt3D.h"

namespace Z
{

static const int NAME_LENGTH = 40;
static const char type_names[][NAME_LENGTH] = {
  "PATCH",        /// TODO PCL-Patch ??? 
  "SEGMENT",
  "COLLINEARITY",
  "LJUNCTION",
  "CORNER",
  "LINE",
  "CLOSURE",
  "RECTANGLE",
  "FLAP",
  "ELLIPSE",
  "PCL_EDGEL",
  "PCL_LINE",
  "PCL_CYLINDER",
  "PCL_SPHERE",
  "UNDEF"
};
static const int type_names_length[] = {5, 7, 12, 9, 6, 4, 7, 9, 4, 7, 9, 8, 13, 11, 5};
 

/**
 * @brief Returns the name of a given gestalt type.
 * @param t Type of Gestalt
 * @return Name of the Gestalt as text.
 */
const char* Gestalt3D::TypeName(Type t)
{
  return type_names[t];
}

/**
 * @brief Returns the length of the name of a given gestalt type.
 * @param t Type of Gestalt
 * @return Lenght of Gestalt type name.
 */
const int Gestalt3D::TypeNameLength(Type t)
{
  return type_names_length[t];
}

/**
 * @brief Return the enum type of a given gestalt type name.
 * @param type_name Name of the Gestalt as text.
 * @return Type of the Gestalt
 */
Gestalt3D::Type Gestalt3D::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}

/**
 * @brief 3D Gestalt constructor.
 * @param _type Type of 3D Gestalt
 * @param _id Unique ID of the Gestalt3D
 */
Gestalt3D::Gestalt3D(Type _type)
{
  type = _type;
  sig = 0.;
  id = -1;
  nodeID = -1;
  objectLabel = -1;
  graphCutLabel = -1;
  drawNodeID = false;
}

}



/**
 * @file Gestalt3D.h
 * @author Andreas Richtsfeld
 * @date Januray 2011
 * @version 0.1
 * @brief Base class for stereo calculated Gestalts in 3D.
 */

#ifndef Z_GESTALT3D_HH
#define Z_GESTALT3D_HH

#include "StereoTypes.h"

namespace Z
{

/**
 * @class Gestalt3D
 * @brief Base class for stereo calculated Gestalts in 3D.
 */
class Gestalt3D
{
public:
  enum Type
  {
    LJUNCTION,
    CORNER,
    RECTANGLE,
    FLAP,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

protected:
//  StereoCore *score;
  Type type;
  unsigned id;
  unsigned rank;

private:
  static const char* TypeName(Type t);
  static const int TypeNameLength(Type t);
  static Type EnumType(const char *type_name);


public:
  Gestalt3D(/*StereoCore *sc, */Type t);
  Type GetType() const {return type;}

};

}

#endif

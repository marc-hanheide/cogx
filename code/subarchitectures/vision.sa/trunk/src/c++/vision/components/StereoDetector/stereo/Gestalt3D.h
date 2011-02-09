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
    LINE,
    CLOSURE,
    RECTANGLE,
    FLAP,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };

protected:
//  StereoCore *score;
  Type type;            // Type of stereo Gestalt
  unsigned id;          // Unique ID of the stereo Gestalt
  unsigned vs3IDs[2];   // IDs of the left and right vs3 Gestalts				/// TODO für alle 3D Gestalts nachziehen
  double sig;           // Significance value
  unsigned rank;	// Rank of the the stereo Gestalt

private:
  static const char* TypeName(Type t);
  static const int TypeNameLength(Type t);
  static Type EnumType(const char *type_name);


public:
  Gestalt3D(/*StereoCore *sc, */Type t);
  Type GetType() const {return type;}
  double GetSignificance() {return sig;}
  unsigned GetVs3ID(unsigned side) {return vs3IDs[side];}
};

}

#endif

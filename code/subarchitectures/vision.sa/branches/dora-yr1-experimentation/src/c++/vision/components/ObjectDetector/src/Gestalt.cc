/**
 * @file Gestalt.cc
 * @author Zillich
 * @date Februar 2007
 * @version 0.1
 * @brief Prototype of Gestalts
 **/

#include "Math.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"
#include "GestaltPrinciple.hh"
#include <string.h>

namespace Z
{

static const int NAME_LENGTH = 40;
static const char type_names[][NAME_LENGTH] = {
  "SEGMENT",
  "LINE",
  "ARC",
  "PARALLEL_LINE_GROUP",
  "CONVEX_ARC_GROUP",
  "ELLIPSE",
	"BALL",
  "COLLINEARITY",
  "T_JUNCTION",
  "L_JUNCTION",
  "E_JUNCTION",
  "EXTELLIPSE",
  "CYLINDER",
  "CONE",
  "CORNER",
  "CLOSURE",
  "EXTCLOSURE",
  "RECTANGLE",
  "EXTRECTANGLE",
  "FLAP",
  "CUBE",
	"WALL_LINE",
  "WALL",
	"EXIT",
	"MOTION_FIELD_ELEMENT",
	"MOTION_FIELD",
	"TRKT_RECTANGLE",
	"TRKT_FLAP",
	"TRKT_CUBE",
  "OBJECT",
	"BEST_RESULT",
	"TRACKEDCUBE",
	"TRACKEDCONE",
	"TRACKEDCYLINDER",
	"TRACKEDBALL",
  "UNDEF"
  };

/**
 * @brief Returns the name of a given gestalt type.
 * @param t Type of the Gestalt
 * @return Returns the name of the type as character-field.
 */
const char* Gestalt::TypeName(Type t)
{
  return type_names[t];
}

/**
 * @brief Return the enum type of a given gestalt type name.
 * @param type_name Name of the Gestalt type
 * @return Returns the Gestalt type
 */
Gestalt::Type Gestalt::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}

/**
 * @brief Gestalt constructor.
 * A new gestalt gets assigned the next higher id, i.e. the current number of
 * gestalts of that type.
 */
Gestalt::Gestalt(Type t)
{
  type = t;
  SetID(NumGestalts(type));
  SetRank(id); // as long as no ranking is performed, the rank is simply the id
  acc = 0.;
  sig = 0.;
  weight = 0.;
// 	clear = true;
  masked = UNDEF_ID;
  Gestalts(type).PushBack(this);
  RankedGestalts(type).PushBack(id);
}

/**
 * Return some textual information.
 * Note: the returned string is allocated statically and will be modified by
 * the next call to GetInfo().
 */
const char* Gestalt::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  snprintf(info_text, info_size,
      "ID: %u rank: %u masked: %u\nnon-acc: %f significance: %f\nweight: %f\n",
      id, rank, masked, 1. - acc, sig, weight);
  return info_text;
}

}

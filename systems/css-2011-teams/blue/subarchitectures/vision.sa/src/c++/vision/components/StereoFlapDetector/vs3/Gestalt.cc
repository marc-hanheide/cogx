/**
 * $Id: Gestalt.cc,v 1.21 2007/02/18 18:02:48 mxz Exp mxz $
 */

#include "Math.hh"
#include "VisionCore.hh"
#include "Gestalt.hh"
#include "GestaltPrinciple.hh"

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
  "COLLINEARITY",
  "T_JUNCTION",
  "L_JUNCTION",
  "A_JUNCTION",
  "CLOSURE",
  "RECTANGLE",
  "FLAP",
  "UNDEF"
  };

/**
 * Returns the name of a given gestalt type.
 */
const char* Gestalt::TypeName(Type t)
{
  return type_names[t];
}

/**
 * Return the enum type of a given gestalt type name.
 */
Gestalt::Type Gestalt::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}

/**
 * Gestalt constructor.
 * A new gestalt gets assigned the next higher id, i.e. the current number of
 * gestalts of that type.
 */
Gestalt::Gestalt(VisionCore *c, Type t)
{
  core = c;
  type = t;
  SetID(core->NumGestalts(type));
  SetRank(id); // as long as no ranking is performed, the rank is simply the id
  acc = 0.;
  sig = 0.;
  masked = UNDEF_ID;
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
      "ID: %u rank: %u masked: %u\nnon-acc: %f significance: %f\n",
      id, rank, masked, 1. - acc, sig);
  return info_text;
}

}


/**
 * $Id: GestaltPrinciple.cc,v 1.15 2006/11/24 13:47:03 mxz Exp mxz $
 */

#include <string.h>
#include "VisionCore.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

static const int NAME_LENGTH = 40;
static const char type_names[][NAME_LENGTH] = {
  "FORM_SEGMENTS",
  "FORM_LINES",
  "FORM_ARCS",
  "FORM_PARALLEL_LINE_GROUPS",
  "FORM_ARC_JUNCTIONS",
  "FORM_CONVEX_ARC_GROUPS",
  "FORM_ELLIPSES",
  "FORM_JUNCTIONS",
  "FORM_CLOSURES",
  "FORM_RECTANGLES",
  "FORM_FLAPS",
  "UNDEF"
  };

/**
 * Returns the name of a given gestalt principle type.
 */
const char* GestaltPrinciple::TypeName(Type t)
{
  return type_names[t];
}

GestaltPrinciple::GestaltPrinciple(VisionCore *vc)
{
  core = vc;
  ResetRunTime();
}

/**
 * Return the enum type of a given gestalt principle type name.
 */
GestaltPrinciple::Type GestaltPrinciple::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}

void GestaltPrinciple::RankGestalts(Gestalt::Type type,
    int(*compar)(const void *, const void *))
{
  // first rank gestalts
  core->RankedGestalts(type).Sort(compar);
  // then set rank for each gestalt
  for(unsigned i = 0; i < core->NumGestalts(type); i++)
    core->RankedGestalts(type, i)->SetRank(i);
}

}


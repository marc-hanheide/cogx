/**
 * @file GestaltPrinciple.hh
 * @author Michael Zillich
 * @date November 2006
 * @version 0.1
 * @brief Implementation of the prototype Gestalt-Principle
 **/

#include <string.h>
#include "VisionCore.hh"
#include "GestaltPrinciple.hh"

namespace Z
{

static const int NAME_LENGTH = 50;
static const char type_names[][NAME_LENGTH] = {
  "FORM_SEGMENTS",
  "FORM_LINES",
  "FORM_ARCS",
  "FORM_PARALLEL_LINE_GROUPS",
  "FORM_CONVEX_ARC_GROUPS",
  "FORM_ELLIPSES",
	"FORM_BALLS",
  "FORM_JUNCTIONS",
  "FORM_EXTELLIPSES",
  "FORM_CYLINDERS",
  "FORM_CONES",
  "FORM_CORNERS",
  "FORM_CLOSURES",
  "FORM_EXTCLOSURES",
  "FORM_RECTANGLES",
  "FORM_EXTRECTANGLES",
  "FORM_FLAPS",
  "FORM_CUBES",
	"FORM_WALLLINES",
  "FORM_WALLS",
	"FORM_EXITS",
	"FORM_MOTION_FIELD",
	"TRACK_RECTANGLES",
	"TRACK_FLAPS",
	"TRACK_CUBES",
  "TRACK_OBJECTS",
	"FORM_BEST_RESULTS",
  "UNDEF"
  };

/**
 *	@brief Returns the name of a given gestalt principle type.
 */
const char* GestaltPrinciple::TypeName(Type t)
{
  return type_names[t];
}

/**
 *	@brief Constructor of class GestaltPrinciple
 *	@param cfg Config defines the configuration for vs3 from a file
 */
GestaltPrinciple::GestaltPrinciple(Config *cfg)
{
	maxAge = 10;
  config = cfg;
  ResetRunTime();
}

/**
 *	@brief Return the enum type of a given gestalt principle type name.
 *	@param type_name Type name
 */
GestaltPrinciple::Type GestaltPrinciple::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}

/**
 *	@brief Ranking gestalts by the significance value
 */
void GestaltPrinciple::RankGestalts(Gestalt::Type type,
    int(*compar)(const void *, const void *))
{
  // first rank gestalts
  RankedGestalts(type).Sort(compar);
  // then set rank for each gestalt
  for(unsigned i = 0; i < NumGestalts(type); i++)
    Gestalts(type, RankedGestalts(type, i))->SetRank(i);
}

/**
 *	@brief Starts the timer for calculation of the processing time for each Gestalt principle.
 */
void GestaltPrinciple::StartRunTime()
{
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &startTime);
}

/**
 *	@brief Stops the timer for calculation of the processing time for each Gestalt principle and adds the time to the runtime counter.
 */
void GestaltPrinciple::StopRunTime()
{
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &endTime);
  runtime += timespec_diff(&endTime, &startTime);
}

}

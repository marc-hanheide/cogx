/**
 * @file StereoBase.cpp
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Base class for stereo caculation of Gestalts, with class Vertex2D, Vertex3D, Surf2D and Surf3D.
 */

#include "StereoBase.h"

namespace Z
{

  
static const int NAME_LENGTH = 40;
static const char type_names[][NAME_LENGTH] = {
  "STEREO_LJUNCTION",
  "STEREO_CORNER",
  "STEREO_ELLIPSE",
  "STEREO_LINE",
  "STEREO_CLOSURE",
  "STEREO_RECTANGLE",
  "STEREO_FLAP",
  "STEREO_FLAP_ARI",
  "STEREO_CUBE",
  "UNDEF"
  };

static const int stereo_type_names_length[] = {16, 13, 14, 11, 14, 16, 11, 15, 11, 5};

/**
 * @brief Returns the length of the name of a given stereo type.
 * @param t Stereo type
 * @return Lenght of Gestalt type name.
 */
const int StereoBase::StereoTypeNameLength(Type t)
{
  return stereo_type_names_length[t];
}

/**
 * @brief Returns the name of a given gestalt type.
 * @param t Type of the stereo principle
 */
const char* StereoBase::TypeName(Type t)
{
  return type_names[t];
}

/**
 * @brief Return the enum type of a given gestalt type name.
 * @param type_name Name of the stereo principle.
 */
StereoBase::Type StereoBase::EnumType(const char *type_name)
{
  for(int i = 0; i < MAX_TYPE; i++)
    if(strncmp(type_name, type_names[i], NAME_LENGTH) == 0)
      return (Type)i;
  return UNDEF;
}


/**
 * @brief Constructor of the stereo base.
 */
StereoBase::StereoBase(StereoCore *sc)
{
  score = sc;
  enabled = false;                      // disabled by default
  pPara.pruning = false;                // disabled by default
}

/**
 * @brief Enables or disables the stereo principle
 * @param status True to enable, false to disable principle.
 */
void StereoBase::EnablePrinciple(bool status)
{
  enabled = status;
}


/**																																													/// TODO TODO gehört das hierher????
 * @brief Calculate Matching score of a surface
 * @param left_surf Left tmp. surface
 * @param right_surf Right tmp. surface.
 * @param match_offs  offset to be added to the indices of the right points in order to
 *             match with the left points.
 *             I.e. left_surf.p[i] corresponds to right_surf.p[i + match_offs]
 *             (of course with proper modulo)
 * @return Returns the sum of the vertical deviations between the surface points (in px.)
 */
double StereoBase::MatchingScoreSurf(Surf2D &left_surf, Surf2D &right_surf, unsigned &match_offs)
{
//  static const double MAX_DELTA_V = 10.;								// TODO: nasty threshold (max. vertical deviation)  SEHR HOCH!!!!
//  static const double MIN_DISPARITY = 0.;								// TODO: obtain from config file?
  double sumv, sumv_min = HUGE, dv, dv_max, du, du_min;
  unsigned ncorners = left_surf.pr.size(), i, j, offs;
  match_offs = UNDEF_ID;
  if(left_surf.pr.size() == right_surf.pr.size())
  {
// printf("      MatchingScoreSurf: %u - %u\n", left_surf.pr.size(), right_surf.pr.size());
// printf("        points: %4.3f - %4.3f und %4.3f - %4.3f\n", left_surf.pr[0].x, left_surf.pr[0].y, right_surf.pr[0].x, right_surf.pr[0].y);

    for(offs = 0; offs < ncorners; offs++)
    {
      sumv = 0.;
      dv_max = 0.;
      du_min = HUGE;
      for(i = 0; i < ncorners; i++)
      {
        j = (i + offs)%ncorners;
        // distance in y-dir (should be zero)
        dv = fabs(left_surf.pr[i].y - right_surf.pr[j].y);
        sumv += dv;
        dv_max = fmax(dv_max, dv);
        // distance in x-dir = disparity, must be > 0, TODO d.h. Punkt muss vor der Kamera liegen => man könnte Bereich einschränken, denn hier gilt von 0 bis unendlich!
        du = left_surf.pr[i].x - right_surf.pr[j].x;
        du_min = fmin(du_min, du);
      }
// printf("          dv_max = %6.5f  du_min: %6.5f => sumv: %6.5f\n", dv_max, du_min, sumv);
      if(dv_max < SC_MAX_DELTA_V_SURF && du_min > SC_MIN_DISPARITY)				// jedes dv muss unter MAX_DELTA_V liegen  UND  jedes du muss über MIN_DISPARITY liegen
			{
        if(sumv < sumv_min)																			// sumv muss kleiner als HUGE sein
        {
// printf("        \n		sumv = %6.5f\n", sumv);
          sumv_min = sumv;
          match_offs = offs;
        }
			}
    }
  }
  return sumv_min;
}

/**										/// TODO TODO gehört das hierher? => Eigentlich eine Vertex2D Berechnung?
 * @brief Calculate Matching score of a point. Returns the vertical deviation \n
 * between the two points, if they are within defined boundaries.
 * @param left_point Left 2D point
 * @param right_point Right 2D point
 * @return Returns the the vertical deviation between the points (in px.)
 */
double StereoBase::MatchingScorePoint(Vertex2D &left_point, Vertex2D &right_point)
{
  double dv, du;
  dv = fabs(left_point.pr.y - right_point.pr.y);        // distance in y-dir (should be zero)
  du = left_point.pr.x - right_point.pr.x;              // distance in z-dir 

//printf("  StereoBase::MatchingScorePoint => du: %4.2f - dv: %4.2f\n", du, dv);

  if(dv < SC_MAX_DELTA_V_POINT && du > SC_MIN_DISPARITY)
    return dv;
  else return HUGE;
}




}



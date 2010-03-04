/**
 * @file FlapAri.hh
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Class file of Gestalt FlapAri: Flaps from rectangles.
 **/

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Rectangle.hh"
#include "FlapAri.hh"

namespace Z
{

/**
 * @brief Constructor of Gestalt class FlapAri.
 * @param r0 Rectangle 0
 * @param r1 Rectangle 1
 * @param mG Mean gap between rectangle corners (shared line)
 * @param sL Shared lines
 * @param iJ Inner Junctions [0/1 = rect0] [2/3 = rect1]
 * @param oJ Outer Junctions [0/1 = rect0] [2/3 = rect1]
 */
FlapAri::FlapAri(VisionCore *c, unsigned r0, unsigned r1, double mG, Array<unsigned> sL, 
		unsigned iJ[4], unsigned oJ[4], Vector2 oIsctR0[4], Vector2 oIsctR1[4]) : Gestalt(c, FLAP_ARI)
{
	rectangles.PushBack(Rectangles(core, r0));
	rectangles.PushBack(Rectangles(core, r1));

  meanGap = mG;
  sharedLines = sL;
  for (int i=0; i<4; i++)
  {
		innerJcts[i] = iJ[i];													/// TODO weg damit
		outerJcts[i] = oJ[i];

		orderedIsctR0[i] = oIsctR0[i];
		orderedIsctR1[i] = oIsctR1[i];
  }

	// calculate center and radius
	center = 	(orderedIsctR0[0] + orderedIsctR0[1] + orderedIsctR1[0] + orderedIsctR1[1])/4.;

	double rad0 = (orderedIsctR0[2] - center).Length();
  double rad1 = (orderedIsctR0[3] - center).Length();
  double rad2 = (orderedIsctR1[2] - center).Length();
  double rad3 = (orderedIsctR1[3] - center).Length();
  
  radius = Max(rad0, rad1);
  radius = Max(radius, rad2);
  radius = Max(radius, rad3);  

	// calculate rectCenter and rectRadius
	rectCenter[0] = (orderedIsctR0[0] + orderedIsctR0[1] + orderedIsctR0[2] + orderedIsctR0[3])/4.;
// 	LJunctions(core, innerJcts[0])->isct + LJunctions(core, innerJcts[1])->isct + 
// 										LJunctions(core, outerJcts[0])->isct + LJunctions(core, outerJcts[1])->isct)/4.;

	rectCenter[1] = (orderedIsctR1[0] + orderedIsctR1[1] + orderedIsctR1[2] + orderedIsctR1[3])/4.;

	// radius = length between center and middle of shared line.
	rectRadius[0] = Length(rectCenter[0] - (orderedIsctR0[0] + orderedIsctR0[1])/2.);
	rectRadius[1] = Length(rectCenter[1] - (orderedIsctR1[0] + orderedIsctR1[1])/2.);

	CalcOrientation();
  CalculateSignificance();
}

/**
 * @brief Estimate, wheter rectangles are on the top of each other or side by side. Distinguish between 6 cases.
 * TODO TODO TODO TODO TODO TODO TODO TODO TODO  Das gehört früher oder später weg!
 */
void FlapAri::CalcOrientation()
{
	bool condition1 = false;
	bool condition2 = false;
  bool condition3 = false;
  bool condition4 = false;

	Vector2 corner_point_r0[2];		// outer_corner_points of rect[0]
	Vector2 corner_point_r1[2];		// outer_corner_points of rect[1]

	corner_point_r0[0] = LJunctions(core, outerJcts[0])->isct;
	corner_point_r0[1] = LJunctions(core, outerJcts[1])->isct;
  corner_point_r1[0] = LJunctions(core, outerJcts[2])->isct;
	corner_point_r1[1] = LJunctions(core, outerJcts[3])->isct;

	// condition 1: outerJunctions.x from rect0 > rect1
	if (corner_point_r0[0].x > corner_point_r1[0].x && corner_point_r0[1].x > corner_point_r1[0].x &&
			corner_point_r0[0].x > corner_point_r1[1].x && corner_point_r0[1].x > corner_point_r1[1].x) condition1 = true; 
	// condition 2: outerJunctions.x from rect0 "<" rect1
	if (corner_point_r0[0].x < corner_point_r1[0].x && corner_point_r0[1].x < corner_point_r1[0].x &&
			corner_point_r0[0].x < corner_point_r1[1].x && corner_point_r0[1].x < corner_point_r1[1].x) condition2 = true; 
	// condition 3: outerJunctions.y from rect0 > rect1
	if (corner_point_r0[0].y > corner_point_r1[0].y && corner_point_r0[1].y > corner_point_r1[0].y &&
			corner_point_r0[0].y > corner_point_r1[1].y && corner_point_r0[1].y > corner_point_r1[1].y) condition3 = true; 
	// condition 4: outerJunctions.y from rect0 "<" rect1
	if (corner_point_r0[0].y < corner_point_r1[0].y && corner_point_r0[1].y < corner_point_r1[0].y &&
			corner_point_r0[0].y < corner_point_r1[1].y && corner_point_r0[1].y < corner_point_r1[1].y) condition4 = true; 

  if (condition1 && !condition3) oCase = 1;
  if (condition2 && !condition4) oCase = 2;
  if (!condition1 && condition3) oCase = 3;
  if (!condition2 && condition4) oCase = 4;
	if (condition1 && condition3) oCase = 5;
	if (condition2 && condition4) oCase = 6;
}

/**
 *	@brief Returns true, if both rectangle center points are inside the two different rectangle radius.
 *	@param flap Index of Flap to compare.
 */
bool FlapAri::IsInside(unsigned flap)
{
	if((((FlapsAri(core, flap)->rectCenter[0] - rectCenter[0]).Length() < rectRadius[0]) &&
		 ((FlapsAri(core, flap)->rectCenter[1] - rectCenter[1]).Length() < rectRadius[1])) ||
		 (((FlapsAri(core, flap)->rectCenter[0] - rectCenter[1]).Length() < rectRadius[0]) &&
		 ((FlapsAri(core, flap)->rectCenter[1] - rectCenter[0]).Length() < rectRadius[1]))) return true;
	else return false;

	/// TODO TODO Calculate new
//   if((Flaps(flap)->center - center).Length() < radius) return true;
//   else return false; 
}


/**
 *	@brief Draws the Gestalt Flap to the main draw area.
 *	@param detail Degree of detail.
 */
void FlapAri::Draw(int detail)
{
  if(detail <= 1 || detail == 2)
  {
//     for(unsigned i = 0; i < 4; i++)
// 		{
//       DrawLine2D(
// 				rectangles[0]->isct[i].x,
// 				rectangles[0]->isct[i].y,
// 				rectangles[0]->isct[(i<3?i+1:0)].x,
// 				rectangles[0]->isct[(i<3?i+1:0)].y);
//       DrawLine2D(
// 				rectangles[0]->isct[i].x,
// 				rectangles[0]->isct[i].y,
// 				rectangles[0]->isct[(i<3?i+1:0)].x,
// 				rectangles[0]->isct[(i<3?i+1:0)].y);
//     }

		// draw with the intersection points
    for(unsigned i = 0; i < 4; i++)
		{
      DrawLine2D(
				orderedIsctR0[i].x,
				orderedIsctR0[i].y,
				orderedIsctR0[(i<3?i+1:0)].x,
				orderedIsctR0[(i<3?i+1:0)].y, RGBColor::yellow);
      DrawLine2D(
				orderedIsctR1[i].x,
				orderedIsctR1[i].y,
				orderedIsctR1[(i<3?i+1:0)].x,
				orderedIsctR1[(i<3?i+1:0)].y, RGBColor::yellow);
    }
  }

  if(detail == 1)
		for(unsigned i=0; i<sharedLines.Size(); i++)
		{
			Lines(core, sharedLines[i])->Draw(detail);
		}	
	
	if(detail == 2)
	{
		// Draw center point and radius
		FillEllipse2D(rectCenter[0].x, rectCenter[0].y, 2, 2, 0, RGBColor::white);
		FillEllipse2D(rectCenter[1].x, rectCenter[1].y, 2, 2, 0, RGBColor::white);

		DrawEllipse2D(rectCenter[0].x, rectCenter[0].y, rectRadius[0], rectRadius[0], 0, RGBColor::white);
		DrawEllipse2D(rectCenter[1].x, rectCenter[1].y, rectRadius[1], rectRadius[1], 0, RGBColor::white);
	}
  if(detail > 2)
  {
  	rectangles[0]->Draw(detail-3);
		rectangles[0]->Draw(detail-3);
  }
}

/**																																													/// TODO Überarbeiten!
 * @brief Returns all information about the Gestalt.
 * @return Returns all information about the Gestalt.
 */
const char* FlapAri::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
	
  n += snprintf(info_text, info_size, "%srects: %u - %u\nshared lines: %u\n",
      Gestalt::GetInfo(), rectangles[0]->ID(), rectangles[1]->ID(), sharedLines.Size());

  n += snprintf(info_text + n, info_size - n,"mean gap: %f\n", meanGap);

//   n += snprintf(info_text + n, info_size - n,"outerJcts: ");
//   for(unsigned i=0; i<4; i++)
// 	n += snprintf(info_text + n, info_size - n, "L(%i) ", outerJcts[i]);
//  
//   n += snprintf(info_text + n, info_size - n,"\ninnerJcts: ");
//   for(unsigned i=0; i<4; i++)
// 	n += snprintf(info_text + n, info_size - n, "L(%i) ", innerJcts[i]);
 
//   n += snprintf(info_text + n, info_size - n,"\noCase: ");
//   switch (oCase)
// 	{
// 		case 1: n += snprintf(info_text + n, info_size - n, "right - left"); break;
// 		case 2: n += snprintf(info_text + n, info_size - n, "left - right"); break;
// 		case 3: n += snprintf(info_text + n, info_size - n, "front - top"); break;
// 		case 4: n += snprintf(info_text + n, info_size - n, "top - front"); break;
// 		case 5: n += snprintf(info_text + n, info_size - n, "left - top"); break;
// 		case 6: n += snprintf(info_text + n, info_size - n, "top - right"); break;
// 		default: break;
// 	}

  return info_text;
}

/**
 * @brief Checks if Gestalt is at position x,y
 * @param x x-coordinate
 * @param y y-coordinate
 * @return Returns true if feature is at this position.
 */
bool FlapAri::IsAtPosition(int x, int y)
{
  return rectangles[0]->IsAtPosition(x, y) || rectangles[1]->IsAtPosition(x, y);
}

/**
 * @brief Calculates significance from rectangles-significance and from the meanGap.
 */
void FlapAri::CalculateSignificance()
{
//	if (meanGap!=0) sig = 100*(1/meanGap);
	sig = rectangles[0]->sig + rectangles[1]->sig;
	if (meanGap!=0) sig -= meanGap*5.;
	if (sig < 0) sig = 0.;
}

}

/**
 * @file Flap.hh
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief Class file of Gestalt Flap
 **/

#include "Draw.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Rectangle.hh"
#include "Flap.hh"

namespace Z
{

/**
 * @brief Constructor of Gestalt class Flap.
 * @param r0 Rectangle 0
 * @param r1 Rectangle 1
 * @param mG Mean gap between rectangle corners (shared line)
 * @param sL Shared lines
 * @param iJ Inner Junctions [0/1 = rect0] [2/3 = rect1]
 * @param oJ Outer Junctions [0/1 = rect0] [2/3 = rect1]
 */
Flap::Flap(unsigned r0, unsigned r1, double mG, Array<unsigned> sL, 
			unsigned iJ[4], unsigned oJ[4]) : Gestalt(FLAP)
{
  rects[0] = r0;
  rects[1] = r1;
  meanGap = mG;
  sharedLines = sL;
  for (int i=0; i<4; i++)
  {
		innerJcts[i] = iJ[i];
		outerJcts[i] = oJ[i];
  }

	// calculate center and radius
	center = 	(	LJunctions(innerJcts[0])->isct + LJunctions(innerJcts[1])->isct + 
							LJunctions(innerJcts[2])->isct + LJunctions(innerJcts[3])->isct)/4.;

	double rad0 = (LJunctions(outerJcts[0])->isct - center).Length();
  double rad1 = (LJunctions(outerJcts[1])->isct - center).Length();
  double rad2 = (LJunctions(outerJcts[2])->isct - center).Length();
  double rad3 = (LJunctions(outerJcts[3])->isct - center).Length();
  
  radius = Max(rad0, rad1);
  radius = Max(radius, rad2);
  radius = Max(radius, rad3);  


	// calculate rectCenter and rectRadius
	rectCenter[0] = (	LJunctions(innerJcts[0])->isct + LJunctions(innerJcts[1])->isct + 
							LJunctions(outerJcts[0])->isct + LJunctions(outerJcts[1])->isct)/4.;

	rectCenter[1] = (	LJunctions(innerJcts[2])->isct + LJunctions(innerJcts[3])->isct + 
							LJunctions(outerJcts[2])->isct + LJunctions(outerJcts[3])->isct)/4.;

	// radius = length between center and middle of shared line.
	rectRadius[0] = Length(rectCenter[0] - (LJunctions(innerJcts[0])->isct + LJunctions(innerJcts[1])->isct)/2.);
	rectRadius[1] = Length(rectCenter[1] - (LJunctions(innerJcts[2])->isct + LJunctions(innerJcts[3])->isct)/2.);

	CalcOrientation();
  CalculateSignificance();

	CalculateFlapProperties();
}

/**
 * @brief Estimate, wheter rectangles are on the top of each other or side by side
 */
void Flap::CalcOrientation()
{
	bool condition1 = false;
	bool condition2 = false;
  bool condition3 = false;
  bool condition4 = false;

	Vector2 corner_point_r0[2];		// outer_corner_points of rect[0]
	Vector2 corner_point_r1[2];		// outer_corner_points of rect[1]

	corner_point_r0[0] = LJunctions(outerJcts[0])->isct;
	corner_point_r0[1] = LJunctions(outerJcts[1])->isct;
  corner_point_r1[0] = LJunctions(outerJcts[2])->isct;
	corner_point_r1[1] = LJunctions(outerJcts[3])->isct;

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


/*
	printf("Cond: %u - %u - %u - %u\n", condition1, condition2, condition3, condition4);

  if (condition1 && !condition3) printf("Flap %u is SideBySide (Right/Left)\n", id);
  if (condition2 && !condition4) printf("Flap %u is SideBySide (Left/Right)\n", id);

  if (!condition1 && condition3) printf("Flap %u is TopOfOther (Front/Top)\n", id);
  if (!condition2 && condition4) printf("Flap %u is TopOfOther (Top/Front)\n", id);

	if (condition1 && condition3) printf("Flap %u is TopOfOther (Left/Top)\n", id);
	if (condition2 && condition4) printf("Flap %u is TopOfOther (Top/Right)\n", id);
  printf("\n");
*/
}

/**
 *	@brief Returns true, if both rectangle center points are inside the two different rectangle radius.
 *	@param flap Index of Flap to compare.
 */
bool Flap::IsInside(unsigned flap)
{
	if((((Flaps(flap)->rectCenter[0] - rectCenter[0]).Length() < rectRadius[0]) &&
		 ((Flaps(flap)->rectCenter[1] - rectCenter[1]).Length() < rectRadius[1])) ||
		 (((Flaps(flap)->rectCenter[0] - rectCenter[1]).Length() < rectRadius[0]) &&
		 ((Flaps(flap)->rectCenter[1] - rectCenter[0]).Length() < rectRadius[1]))) return true;
	else return false;

	/// TODO TODO Calculate new
//   if((Flaps(flap)->center - center).Length() < radius) return true;
//   else return false; 
}


/**
 *	@brief Draws the Gestalt Flap to the main draw area.
 *	@param detail Degree of detail.
 */
void Flap::Draw(int detail)
{
  if(detail <= 1 || detail == 2)
  {
    for(unsigned i = 0; i < 4; i++)
		{
      DrawLine2D(
				LJunctions(Rectangles(rects[0])->jcts[i])->isct.x,
				LJunctions(Rectangles(rects[0])->jcts[i])->isct.y,
				LJunctions(Rectangles(rects[0])->jcts[(i<3?i+1:0)])->isct.x,
				LJunctions(Rectangles(rects[0])->jcts[(i<3?i+1:0)])->isct.y,
							RGBColor::coral);
      DrawLine2D(
				LJunctions(Rectangles(rects[1])->jcts[i])->isct.x,
				LJunctions(Rectangles(rects[1])->jcts[i])->isct.y,
				LJunctions(Rectangles(rects[1])->jcts[(i<3?i+1:0)])->isct.x,
				LJunctions(Rectangles(rects[1])->jcts[(i<3?i+1:0)])->isct.y,
							RGBColor::coral);
    }
  }

  if(detail == 1)
		for(unsigned i=0; i<sharedLines.Size(); i++)
		{
			Lines(sharedLines[i])->Draw(detail);
		}	
	
	if(detail == 2)
	{
// 		Draw center point and radius
		FillEllipse2D(rectCenter[0].x, rectCenter[0].y, 2, 2, 0, RGBColor::white);
		FillEllipse2D(rectCenter[1].x, rectCenter[1].y, 2, 2, 0, RGBColor::white);

		DrawEllipse2D(rectCenter[0].x, rectCenter[0].y, rectRadius[0], rectRadius[0], 0, RGBColor::white);
		DrawEllipse2D(rectCenter[1].x, rectCenter[1].y, rectRadius[1], rectRadius[1], 0, RGBColor::white);
	}
  if(detail > 2)
  {
  	Rectangles(rects[0])->Draw(detail-3);
		Rectangles(rects[1])->Draw(detail-3);
  }
}

/**
 * @brief Returns all information about the Gestalt.
 */
const char* Flap::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n = 0;
	
  n += snprintf(info_text, info_size, "%srects: %u - %u\nshared lines: %u\n",
      Gestalt::GetInfo(), rects[0], rects[1], sharedLines.Size());

  n += snprintf(info_text + n, info_size - n,"mean gap: %f\n", meanGap);

  n += snprintf(info_text + n, info_size - n,"outerJcts: ");
  for(unsigned i=0; i<4; i++)
	n += snprintf(info_text + n, info_size - n, "L(%i) ", outerJcts[i]);
 
  n += snprintf(info_text + n, info_size - n,"\ninnerJcts: ");
  for(unsigned i=0; i<4; i++)
	n += snprintf(info_text + n, info_size - n, "L(%i) ", innerJcts[i]);
 
  n += snprintf(info_text + n, info_size - n,"\noCase: ");
  switch (oCase)
	{
		case 1: n += snprintf(info_text + n, info_size - n, "right - left"); break;
		case 2: n += snprintf(info_text + n, info_size - n, "left - right"); break;
		case 3: n += snprintf(info_text + n, info_size - n, "front - top"); break;
		case 4: n += snprintf(info_text + n, info_size - n, "top - front"); break;
		case 5: n += snprintf(info_text + n, info_size - n, "left - top"); break;
		case 6: n += snprintf(info_text + n, info_size - n, "top - right"); break;
		default: break;
	}

  return info_text;
}

bool Flap::IsAtPosition(int x, int y)
{
  return Rectangles(rects[0])->IsAtPosition(x, y) ||
    Rectangles(rects[1])->IsAtPosition(x, y);
}

void Flap::CalculateSignificance()												// TODO ARI: Calculate
{
//	if (meanGap!=0) sig = 100*(1/meanGap);
		
	sig = Rectangles(rects[0])->sig + Rectangles(rects[1])->sig;
	if (meanGap!=0) sig -= meanGap*5.;
	if (sig < 0) sig = 0.;
}

void Flap::CalculateFlapProperties()
{
	/// TODO Set properties of flap

// 	fp.corner_points[0][0] = 0.0;
}
}

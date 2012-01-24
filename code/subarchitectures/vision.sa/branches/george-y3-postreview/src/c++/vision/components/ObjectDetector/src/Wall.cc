/**
 * $Id: Wall.cc,v 1.0 2008/06/16 13:47:03 mxz Exp mxz $
 */

#include "math.h"
#include "Wall.hh"
#include "Draw.hh"
#include "Line.hh"
#include "WallLine.hh"

namespace Z
{

Wall::Wall(bool c, bool lL, Vector2 *bP, unsigned *wL, Vector2 *p, unsigned cP) : Gestalt(WALL)
{
	corner = c;
	longLine = lL;

	borderPoints[0] = bP[0];
	borderPoints[1] = bP[1];

	wallLines[0] = wL[0];
	wallLines[1] = wL[1];

	points[0] = p[0];
	points[1] = p[1];

	cornerPos = cP;

	lineLength = 0.;
	LengthMasking();

	CalculateSignificance();
}

/**
**	LengthMasking()
**	Mask wall if length of line is too short
*/	
void Wall::LengthMasking()
{
	// calculate length of both lines, if there is no corner
	if(!corner && !longLine)
	{
		lineLength = Length(borderPoints[0] - points[0]);
		lineLength += Length(borderPoints[1] - points[1]);

		if(lineLength < (VisionCore::IW()*0.5)) Mask(10000);	// TODO Masking with 10000, if line is not long enough
	}
}

void Wall::Draw(int detail)
{
	if (corner)
	{
		if(detail == 0)
		{
			DrawLine2D(borderPoints[0].x, borderPoints[0].y, points[0].x, points[0].y, RGBColor::red);
			DrawLine2D(borderPoints[1].x, borderPoints[1].y, points[0].x, points[0].y, RGBColor::red);
			DrawLine2D(points[0].x, points[0].y, points[1].x, points[1].y, RGBColor::red);
		}
		if(detail >= 1)
		{
			WallLines(wallLines[0])->Draw(detail-1);
			WallLines(wallLines[1])->Draw(detail-1);
		}
	}
	else if(longLine)
	{
		if(detail == 0)
		{
			TransparentQuadrilateral2D(borderPoints[0].x, borderPoints[0].y, borderPoints[1].x, borderPoints[1].y, 
					borderPoints[1].x, borderPoints[1].y-10., borderPoints[0].x, borderPoints[0].y-10., RGBColor::red, true, 64);

			DrawLine2D(borderPoints[0].x, borderPoints[0].y, borderPoints[1].x, borderPoints[1].y, RGBColor::red);
		}
		if(detail >= 1)
		{
			WallLines(wallLines[0])->Draw(detail-1);
// 			WallLines(wallLines[1])->Draw(detail-1);	// undefined, for a longLine
		}
	}
	else
	{
		if(detail == 0)
		{
			TransparentQuadrilateral2D(borderPoints[0].x, borderPoints[0].y, borderPoints[1].x, borderPoints[1].y, 
					borderPoints[1].x, borderPoints[1].y-10., borderPoints[0].x, borderPoints[0].y-10., RGBColor::red, true, 64);

			DrawLine2D(borderPoints[0].x, borderPoints[0].y, points[0].x, points[0].y, RGBColor::red);
			DrawLine2D(points[0].x, points[0].y, points[1].x, points[1].y, RGBColor::coral);
			DrawLine2D(points[1].x, points[1].y, borderPoints[1].x, borderPoints[1].y, RGBColor::red);
		}
		if(detail >= 1)
		{
			WallLines(wallLines[0])->Draw(detail-1);
			WallLines(wallLines[1])->Draw(detail-1);
		}
	}
}

const char* Wall::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
		"%s \n", Gestalt::GetInfo());

	if (corner)
 		n += snprintf(info_text + n, info_size - n, "lines with corner: true\ncornerPos: %u\n", cornerPos);
	else if (longLine)
 		n += snprintf(info_text + n, info_size - n, "long line\n");
	else
 		n += snprintf(info_text + n, info_size - n, "lines with corner: false\n");


 	n += snprintf(info_text + n, info_size - n, "wall lines: %u-%u\n", wallLines[0], wallLines[1]);


/*
	for(unsigned i=0; i<lines.Size(); i++)
  	n += snprintf(info_text + n, info_size - n, "%u ", lines[i]);

	if (twoOutLines)
  	n += snprintf(info_text + n, info_size - n, "\ntwo out lines: true");
	else
  	n += snprintf(info_text + n, info_size - n, "\ntwo out lines: false");

   	n += snprintf(info_text + n, info_size - n, "\ndir(x/y): %4.3f / %4.3f\n", dir.x, dir.y);

   	n += snprintf(info_text + n, info_size - n, "outLineNearPoint: %u\n", linesNearPoints[0]);*/

	return info_text;
}

bool Wall::IsAtPosition(int x, int y)
{

	if(wallLines[1] != UNDEF_ID) return (WallLines(wallLines[0])->IsAtPosition(x, y) || WallLines(wallLines[1])->IsAtPosition(x, y));
  return true;
}

/**
**	IsInside()
**	True, if the wall border points are at one side inside the radius of another
**	=> threshold with 20 pixel
*/
bool Wall::IsInside(unsigned wall)
{
																							/// TODO TODO TODO TODO if one of the 4 start points are "to near" (threshold of 20 pixel)
 	double nearThreshold = 50.;
	if (!Walls(wall)->corner)
	{
		if((Walls(wall)->borderPoints[0] - borderPoints[0]).Length() < nearThreshold ||
			(Walls(wall)->borderPoints[1] - borderPoints[0]).Length() < nearThreshold ||
			(Walls(wall)->borderPoints[0] - borderPoints[1]).Length() < nearThreshold ||
			(Walls(wall)->borderPoints[1] - borderPoints[1]).Length() < nearThreshold) return true;
	}
 	return false;
}

void Wall::CalculateSignificance()
{
	/// TODO TODO TODO TODO TODO TODO TODO TODO TODO 
	/// wie kann man die Significance berechnen?
	/// => 3 Linien => 3 Winkelunterschiede multiplizieren?
	/// aber was macht man dann bei durchgehenden Linien?
	if (!corner && !longLine)
 	{
		Vector2 line0 = points[0] - borderPoints[0];					// first line
		Vector2 lineDir0 = Normalise(line0);
		Vector2 line1 = points[1] - borderPoints[1];					// second line
		Vector2 lineDir1 = Normalise(line1);
		lineDir1.x = -lineDir1.x;															// invert direction of second line
		lineDir1.y = -lineDir1.y;
		Vector2 con = points[1] - points[0];									// connection line
		Vector2 conDir = Normalise(con);
	
		double dPhiLine = acos(Dot(lineDir0, lineDir1));
		double dPhi0 = acos(Dot(lineDir0, conDir));
		double dPhi1 = acos(Dot(lineDir1, conDir));
	
		sig = 0.0001/(dPhiLine*dPhi0*dPhi1);
	}
	else if(corner)
	{
		sig = (WallLines(wallLines[0])->sig * WallLines(wallLines[1])->sig)/100.;
	}
	else	/// longLine
	{
		sig = 1. - 0.001*id;	/// first longLine ist the best line
	}
}

}

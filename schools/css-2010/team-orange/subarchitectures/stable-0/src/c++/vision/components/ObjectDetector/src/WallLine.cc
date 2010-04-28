/**
 * $Id: WallLine.cc,v 1.0 2008/07/02 13:47:03 mxz Exp mxz $
 */

#include "math.h"
#include "WallLine.hh"
#include "Draw.hh"
#include "Line.hh"

namespace Z
{

WallLine::WallLine(unsigned oL, unsigned oLB, Array<unsigned> &l, Array<unsigned> &lNP, bool wL) : Gestalt(WALL_LINE)
{
	outLine = oL;
	outLineBorder = oLB;
	lines = l;
	linesNearPoints = lNP;
	twoOutLines = wL;

	point[START] = Lines(lines[0])->point[linesNearPoints[0]];
	point[END] = Lines(lines[lines.Size()-1])->point[Other(linesNearPoints[linesNearPoints.Size()-1])];

	Vector2 line = point[END] - point[START];
	dir = Normalise(line);

  CalculateSignificance();
}

void WallLine::Draw(int detail)
{
	for(unsigned i=0; i<lines.Size(); i++)
		Lines(lines[i])->Draw(detail);

}

const char* WallLine::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
		"%s \noutLine (border): %u (%u)\nLines: ", Gestalt::GetInfo(), outLine, outLineBorder);

	for(unsigned i=0; i<lines.Size(); i++)
  	n += snprintf(info_text + n, info_size - n, "%u ", lines[i]);

	if (twoOutLines)
  	n += snprintf(info_text + n, info_size - n, "\ntwo out lines: true");
	else
  	n += snprintf(info_text + n, info_size - n, "\ntwo out lines: false");

   	n += snprintf(info_text + n, info_size - n, "\ndir(x/y): %4.3f / %4.3f\n", dir.x, dir.y);

   	n += snprintf(info_text + n, info_size - n, "outLineNearPoint: %u\n", linesNearPoints[0]);

	return info_text;
}

bool WallLine::IsAtPosition(int x, int y)
{
  for(unsigned i = 0; i < lines.Size(); i++)
    if(Lines(lines[i])->IsAtPosition(x, y))
      return true;
  return false;
}

void WallLine::CalculateSignificance()
{
	/// wie kann man die Significance berechnen?


	/// => über Winkel von Linien (was, wenn ein Knick (Eck) dazwischen)
	bool isGreater = false;
	double phiSum = 0.;
	for(unsigned i=0; i<lines.Size()-1; i++)
	{
		double dPhi = (Lines(lines[i])->phi)-(Lines(lines[i+1])->phi);
		if (dPhi > M_PI) dPhi -= M_PI;
		if (dPhi < -M_PI) dPhi +=M_PI;
		if (dPhi > M_PI/2.) dPhi -= M_PI/2.;
		if (dPhi < -M_PI/2.) dPhi += M_PI/2.;

		if (fabs(dPhi) > 0.5 && !isGreater && lines.Size()>2)
			isGreater = true;			/// TODO: Ein Winkel darf größer als 0.5 sein (und wird dann nicht gezählt
		else	
			phiSum += fabs(dPhi);

//printf("dPhi: %f\n", fabs(dPhi));
	}

//printf("phiSum: %f\n\n", phiSum);

	/// => über significance von Collinearitäten (was, wenn eine L-Junction dazwischen)

	if (phiSum == 0.) sig = Lines(outLine)->len/2.;				// TODO only one line (no phiSum)
	else
	{
		if (twoOutLines) sig = 10/phiSum;
		else sig = 1/phiSum;
	}
}

}

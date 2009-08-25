/**
 * $Id: Exit.cc,v 1.0 2008/08/02 13:47:03 mxz Exp mxz $
 */

#include "Exit.hh"
#include "Draw.hh"

namespace Z
{

Exit::Exit(Vector2 *wB, Vector2 *eLS, Vector2 *eLI) : Gestalt(EXIT_)
{
	wallBorder[0] = wB[0];
	wallBorder[1] = wB[1];

	exitLinesStart[0] = eLS[0];
	exitLinesStart[1] = eLS[1];

	exitLinesIntersection[0] = eLI[0];
	exitLinesIntersection[1] = eLI[1];

	CalculateSignificance();
}

void Exit::Draw(int detail)
{
	DrawLine2D(exitLinesStart[0].x, exitLinesStart[0].y, exitLinesIntersection[0].x, exitLinesIntersection[0].y, RGBColor::red);
	DrawLine2D(exitLinesStart[1].x, exitLinesStart[1].y, exitLinesIntersection[1].x, exitLinesIntersection[1].y, RGBColor::red);
	DrawLine2D(exitLinesIntersection[0].x, exitLinesIntersection[0].y, exitLinesIntersection[1].x, exitLinesIntersection[1].y, RGBColor::red);

	TransparentQuadrilateral2D(exitLinesStart[0].x, exitLinesStart[0].y, exitLinesStart[1].x, exitLinesStart[1].y, 
			exitLinesIntersection[1].x, exitLinesIntersection[1].y, exitLinesIntersection[0].x, exitLinesIntersection[0].y, RGBColor::green, true, 64);
}

const char* Exit::GetInfo()
{
  const unsigned info_size = 10000;
  static char info_text[info_size] = "";
  int n=0;
  n += snprintf(info_text, info_size, 
		"%s\n\nexitLinesStart[0]: %4.3f - %4.3f\nexitLinesStart[1]: %4.3f - %4.3f\n\nexitLinesIntersection[0]: %4.3f - %4.3f\nexitLinesIntersection[1]: %4.3f - %4.3f", 
		Gestalt::GetInfo(), exitLinesStart[0].x, exitLinesStart[0].y, exitLinesStart[1].x, exitLinesStart[1].y, exitLinesIntersection[0].x, exitLinesIntersection[0].y, exitLinesIntersection[1].x, exitLinesIntersection[1].y);

// 	if (corner)
//  		n += snprintf(info_text + n, info_size - n, "lines with corner: true\ncornerPos: %u\n", cornerPos);
// 	else
//  		n += snprintf(info_text + n, info_size - n, "lines with corner: false\n");
// 
// 
//  	n += snprintf(info_text + n, info_size - n, "wall lines: %u-%u\n", wallLines[0], wallLines[1]);


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

bool Exit::IsAtPosition(int x, int y)
{
// 	return (WallLines(wallLines[0])->IsAtPosition(x, y) || WallLines(wallLines[1])->IsAtPosition(x, y));
  return true;
}

void Exit::CalculateSignificance()
{

}

}

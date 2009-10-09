/**
 * $Id: FormCones.cc,v 1.0 2008/06/16 13:47:03 mxz Exp mxz $
 */

#include "FormWalls.hh"
#include "VisionCore.hh"
#include "Line.hh"
#include "Collinearity.hh"
#include "WallLine.hh"
#include "Wall.hh"
#include "FormJunctions.hh"

namespace Z
{

static int CmpWalls(const void *a, const void *b)
{
  if( Walls(*(unsigned*)a)->sig > Walls(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormWalls::Rank()
{
  RankGestalts(Gestalt::WALL, CmpWalls);
}

/**
**	Mask()
**	Masking walls
*/
void FormWalls::Mask()
{
  for(unsigned i=0; i<NumWalls(); i++)
  {
		for(unsigned j=0; j<NumWalls(); j++)
		{
			if(!Walls(i)->IsMasked() && !Walls(j)->IsMasked())
			if(Walls(i)->sig <= Walls(j)->sig && i != j)
			{
				if(Walls(i)->IsInside(j))
				{	
					Walls(i)->Mask(j);		 
				}
			}
		}
	}
}

bool FormWalls::NeedsOperate()
{ 
  return needsOperate;	
}

void FormWalls::Reset(const Image *img)
{
	needsOperate = true;
}

FormWalls::FormWalls(Config *cfg) : GestaltPrinciple(cfg)
{
  needsOperate = true;
	firstCall = true;
}

/**
**	InformNewGestalt()
*/
void FormWalls::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
}

/**
*		Operate()
*/
void FormWalls::Operate(bool incremental)
{
  StartRunTime();
	if (firstCall)
	{
		imgWidth = VisionCore::IW();
		imgHeight = VisionCore::IH();
	} 
	firstCall = false;
	needsOperate = false;
  StopRunTime();
}

/**
*		OperateNonIncremental()
*		call from VisionCore after incremental operating
*/
void FormWalls::OperateNonIncremental()
{
  StartRunTime();

//	FindContinousLines();				/// durchgehende wall-line mit collinearity (>0.3) als Ecke und Eckkante nach oben
//printf("FormWalls: div by zero: cl\n");

	//FindConnectedLines();			/// wall lines with direct connection
	//FindLConnectedLines();			/// wall lines, die über eine L-Junction miteinander verbunden sind
	FindWallWithGap();					/// Try to built a straight wall from wallLines with a gap between the lines
//printf("FormWalls: div by zero: wg\n");

	FindLongWallLine();

	Rank();
	Mask();

//printf("FormWalls: div by zero end\n");

	StopRunTime();
}


/**
**	FindContinousLines
**	Line from one to another border without gap and with an collinearity as corner
**	There must be a wallLine from the top to the corner
*/
void FormWalls::FindContinousLines()
{
	for(unsigned i=0; i<NumWallLines(); i++)
	{
		if(WallLines(i)->TwoOutLines())
		{
			// find an corner in the line
			Array<unsigned> wallLines = WallLines(i)->lines;
			Array<unsigned> wallLinesNearPoints = WallLines(i)->linesNearPoints;

			// get the two border points of the wallLines
			Vector2 borderPoints[2];
			borderPoints[0] = Lines(wallLines[0])->point[wallLinesNearPoints[0]];
			borderPoints[1] = Lines(wallLines[wallLines.Size()-1])->point[Other(wallLinesNearPoints[wallLines.Size()-1])];

			double maxPhi = 0.;		// maximum difference of opening angle
			unsigned iOfMaxPhi;		// place in array of maxPhi

			Vector2 cPoint;				// corner point

			for(unsigned j=1; j<wallLines.Size()-1; j++)
			{
				// get the point from the lines
				Vector2 conPoint = Lines(wallLines[j])->point[wallLinesNearPoints[j]];

				Vector2 line0 = conPoint - borderPoints[0];					// first line
				Vector2 line1 = borderPoints[1] - conPoint;					// second line
				Vector2 lineDir0 = Normalise(line0);
				Vector2 lineDir1 = Normalise(line1);
				double dPhiLine = acos(Dot(lineDir0, lineDir1));
			
				if (dPhiLine > maxPhi)
				{
					maxPhi = dPhiLine;
					iOfMaxPhi = j;					/// TODO weitergeben, damit man weiß, wo der Knick ist
					cPoint = conPoint;
				}
			}

			if(maxPhi > 0.3)																			/// TODO TODO Threshold for corner: 0.3 rad = 15,6 degree
			{
				/// now find the corner edge to the top
				for(unsigned k=0; k<NumWallLines(); k++)
				{
					// get all wall lines with outLineBorder = 1 (top)
					if (WallLines(k)->outLineBorder == 1)
					{
						// get end point of the line
						Vector2 endPoint = WallLines(k)->point[1];

						Vector2 dist = endPoint - cPoint;
						if(fabs(dist.Length()) < 30.)										/// TODO TODO Threshold for "near" = 30 Pixel !!!
						{
							unsigned wallLines[2];
							wallLines[0] = i;
							wallLines[1] = k;

							Vector2 points[2];
							points[0] = cPoint;
							points[1] = WallLines(k)->point[0];

							NewWall(true, false, wallLines, borderPoints, points, iOfMaxPhi);
						}
					}
				}
			}
		}
	}
}

/**
**	FindConnectedLines()
**	Find wall lines which are connected direct => TODO TODO TODO TODO Hat das überhaupt Sinn?
*/
void FormWalls::FindConnectedLines()
{
	for(unsigned i=0; i<NumWallLines(); i++)
	{
		for(unsigned j=i; j<NumWallLines(); j++)
		{
			if (i != j && WallLines(i)->outLineBorder != WallLines(j)->outLineBorder && 
					!WallLines(i)->twoOutLines && !WallLines(j)->twoOutLines)
			{
				/// Wie kann man diese Linien vergleichen?

				// alle linien der ersten Wall mit allen Linien der zweiten Wall vergleichen
				Array<unsigned> linesI = WallLines(i)->lines;
				Array<unsigned> linesJ = WallLines(j)->lines;

				for(unsigned k=0; k<linesI.Size(); k++)
				{
					if (linesJ.Contains(linesI[k]))
					{
printf("XXXXXXXXXXXXXXXXX Line enthält Line (i/j): %u - %u\n", i, j);
					}
				}
			}
		}
	}
}


/**
**	FindLConnectedLines()
**	Find wall lines which are connected through a l-junction  => TODO TODO TODO TODO Hat das überhaupt Sinn?
*/
void FormWalls::FindLConnectedLines()
{
	for(unsigned i=0; i<NumWallLines(); i++)
	{
		for(unsigned j=i; j<NumWallLines(); j++)
		{
			if (i != j && WallLines(i)->outLineBorder != WallLines(j)->outLineBorder && 
					!WallLines(i)->twoOutLines && !WallLines(j)->twoOutLines)
			{
				// alle linien der ersten Wall mit allen Linien der zweiten Wall vergleichen
				Array<unsigned> linesI = WallLines(i)->lines;
				Array<unsigned> linesJ = WallLines(j)->lines;

				for(unsigned k=0; k<linesI.Size(); k++)
				{
					for(unsigned l=0; l<linesJ.Size(); l++)
					{
						Array<unsigned> linesILJ;
						Array<unsigned> linesJLJ;
						Lines(linesI[k])->GetAllLJunctions(&linesILJ);
						Lines(linesJ[l])->GetAllLJunctions(&linesJLJ);
						for(unsigned m=0; m<linesILJ.Size(); m++)
						{
							if (linesJLJ.Contains(linesILJ[m]))
							{
	printf("XXXXXXXXXXXXXXXXX FindLConnectedLines (i/j): %u - %u\n", i, j);
							}
						}
					}
				}
			}
		}
	}
}



/**
**	FindWallWithGap()
**	Try to build a wall from wallLines with an gap
**	Find two wallLines with
**		- "same" angles and 
			- compare the angle of the connection between, with the line-angles
			When they are smaller than 0.1 => create a new wall
**/

/// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
/// Hier gibt es eine div by zero Fehler!
void FormWalls::FindWallWithGap()
{
//printf("Division problem?\n");
 	for(unsigned i=0; i<NumWallLines(); i++)
 	{
		for(unsigned j=i+1; j<NumWallLines(); j++)
		{
			double dPhi = acos(Dot(WallLines(i)->dir, WallLines(j)->dir));
			if(dPhi > M_PI/2.) dPhi = M_PI - dPhi;

			if(dPhi < 0.1 && WallLines(i)->outLineBorder != WallLines(j)->outLineBorder)						/// TODO ARI: Threshold for dPhi = 0.1
			{
				// get connecting vector between the two line ends and the direction
				Vector2 con = WallLines(j)->point[END] - WallLines(i)->point[END];

				/// if lines are connected direct: happens ever? => sollte nicht auftreten, weil versch. Linien nie den selben Endpixel haben sollten. (kommt aber trotzdem vor!)
				if (con.x == 0 && con.y == 0)
				{
					printf("XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX	FormWalls: Direct connection found: %u - %u\n", i, j);
				}
				else
				{
					Vector2 conDir = Normalise(con);

					// get the directions of the two lines and invert the second line direction
					Vector2 lineDir0, lineDir1;
					lineDir0 = WallLines(i)->dir;	
					lineDir1.x = -WallLines(j)->dir.x;	// invert direction of second line
					lineDir1.y = -WallLines(j)->dir.y;	// invert direction of second line

					double dPhiLine0 = acos(Dot(lineDir0, conDir));
					double dPhiLine1 = acos(Dot(lineDir1, conDir));

					if(dPhiLine0 < 0.1 && dPhiLine1 < 0.1)																					/// TODO ARI: Threshold for maximal dPhi
					{
// printf("%u - %u - dPhi: %4.2f\n", i, j, dPhi);
// printf("	dPhis: %4.3f / %4.3f\n", dPhiLine0, dPhiLine1);

						unsigned wL[2];				// wall lines
						wL[0] = i;
						wL[1] = j;

						Vector2 borderPoints[2];
						borderPoints[0] = WallLines(i)->point[START];
						borderPoints[1] = WallLines(j)->point[START];

						Vector2 points[2];
						points[0] = WallLines(i)->point[END];
						points[1] = WallLines(j)->point[END];

						// recalculate borderPoints
						Vector2 border0[2];
						switch(WallLines(wL[0])->outLineBorder)
						{
							case 0: border0[0].x = 0.; border0[0].y = 0.; border0[1].x = 0.; border0[1].y = imgHeight; break;
							case 1: border0[0].x = 0.; border0[0].y = 0.; border0[1].x = imgWidth; border0[1].y = 0.; break;
							case 2: border0[0].x = imgWidth; border0[0].y = 0.; border0[1].x = imgWidth; border0[1].y = imgHeight; break;
							case 3: border0[0].x = 0.; border0[0].y = imgHeight; border0[1].x = imgWidth; border0[1].y = imgHeight; break;
							default: break;
						}
						Vector2 border1[2];
						switch(WallLines(wL[1])->outLineBorder)
						{
							case 0: border1[0].x = 0.; border1[0].y = 0.; border1[1].x = 0.; border1[1].y = imgHeight; break;
							case 1: border1[0].x = 0.; border1[0].y = 0.; border1[1].x = imgWidth; border1[1].y = 0.; break;
							case 2: border1[0].x = imgWidth; border1[0].y = 0.; border1[1].x = imgWidth; border1[1].y = imgHeight; break;
							case 3: border1[0].x = 0.; border1[0].y = imgHeight; border1[1].x = imgWidth; border1[1].y = imgHeight; break;
							default: break;
						}

						borderPoints[0] = LineIntersection(borderPoints[1], borderPoints[0] - borderPoints[1], border0[0], border0[1] - border0[0]);
						borderPoints[1] = LineIntersection(borderPoints[0], borderPoints[1] - borderPoints[0], border1[0], border1[1] - border1[0]);
					
						NewWall(false, false, wL, borderPoints, points, UNDEF_ID);
					}
				}
			}
		}
	}
//printf("=> no division problem\n");
}

/**
**	FindLongWallLine()
**	If there is a wall line, which is longer than 70 Percent of the image width, and
**	the first part of the line has the same direction as the whole line => wall found?
*/
void FormWalls::FindLongWallLine()
{
// 	printf("find long wall line\n");
	
	for(unsigned i=0; i<NumWallLines(); i++)
 	{
		// wall line is longer than 70 Percent of the image?
		if (Length(WallLines(i)->point[0] - WallLines(i)->point[1]) > 0.7*VisionCore::IW())
		{
			// first line?
// printf("found a long wall line!!!\n");
	
			unsigned wL[2];				// wall lines
			wL[0] = i;
			wL[1] = UNDEF_ID;

			Vector2 points[2];		// undefined points

			Vector2 borderPoints[2];
			borderPoints[0] = WallLines(i)->point[START];
			borderPoints[1] = WallLines(i)->point[END];

			// recalculate borderPoints
			Vector2 border0[2], border1[2];
			if(WallLines(wL[0])->outLineBorder == 0)
			{
				border0[0].x = 0.; border0[0].y = 0.; border0[1].x = 0.; border0[1].y = imgHeight;
				border1[0].x = imgWidth; border1[0].y = 0.; border1[1].x = imgWidth; border1[1].y = imgHeight;
				borderPoints[0] = LineIntersection(borderPoints[1], borderPoints[0] - borderPoints[1], border0[0], border0[1] - border0[0]);
				borderPoints[1] = LineIntersection(borderPoints[0], borderPoints[1] - borderPoints[0], border1[0], border1[1] - border1[0]);
				NewWall(false, true, wL, borderPoints, points, UNDEF_ID);
			}
			else if (WallLines(wL[0])->outLineBorder == 2)
			{
				border0[0].x = imgWidth; border0[0].y = 0.; border0[1].x = imgWidth; border0[1].y = imgHeight; break;
				border1[0].x = 0.; border1[0].y = 0.; border1[1].x = 0.; border1[1].y = imgHeight;
				borderPoints[0] = LineIntersection(borderPoints[1], borderPoints[0] - borderPoints[1], border0[0], border0[1] - border0[0]);
				borderPoints[1] = LineIntersection(borderPoints[0], borderPoints[1] - borderPoints[0], border1[0], border1[1] - border1[0]);
				NewWall(false, true, wL, borderPoints, points, UNDEF_ID);
			}
		}
	}
}

/**
**	NewWall()
**	Built a new wall
**/
void FormWalls::NewWall(bool corner, bool longLine, unsigned *wallLines, Vector2 *borderPoints, Vector2 *points, unsigned cornerPos)
{
	NewGestalt(new Wall(corner, longLine, borderPoints, wallLines, points, cornerPos));
}

}

	/// Fallunterscheidungen:
	/// 	- 2 outLines mit Object dazwischen (occlusion)
	/// 	- 2 outLines mit Colls dazwischen
	/// 	- 2 outLines mit Colls und 2 Richtungen von Linien (falsche L-Junction)
	///		- 2 outLines mit L-Junction dazwischen
























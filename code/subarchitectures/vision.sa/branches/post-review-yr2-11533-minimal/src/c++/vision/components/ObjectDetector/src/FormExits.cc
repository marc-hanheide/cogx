/**
 * $Id: FormExits.cc,v 1.0 2008/09/01 13:47:03 mxz Exp mxz $
 */

#include "FormExits.hh"
#include "Exit.hh"
#include "Line.hh"
#include "WallLine.hh"
#include "Wall.hh"

namespace Z
{

static int CmpExits(const void *a, const void *b)
{
  if( Exits(*(unsigned*)a)->sig > Exits(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormExits::Rank()
{
  RankGestalts(Gestalt::EXIT_, CmpExits);
}

/**
**	Mask()
**	Masking walls
*/
void FormExits::Mask()
{
//   for(unsigned i=0; i<NumWalls(); i++)
//   {
// 		for(unsigned j=0; j<NumWalls(); j++)
// 		{
// 			if(!Exits(i)->IsMasked() && !Exits(j)->IsMasked())
// 			if(Exits(i)->sig <= Exits(j)->sig && i != j)
// 			{
// 				if(Exits(i)->IsInside(j))
// 				{	
// 					Exits(i)->Mask(j);		 
// 				}
// 			}
// 		}
// 	}
}

bool FormExits::NeedsOperate()
{ 
  return needsOperate;	
}

void FormExits::Reset(const Image *img)
{
	needsOperate = true;
	firstCall = true;
}

FormExits::FormExits(Config *cfg) : GestaltPrinciple(cfg)
{
	// print find exit results
	findExit = false;
	firstCall = true;
}

/**
**	InformNewGestalt()
*/
void FormExits::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
}

/**
*		Operate()
*/
void FormExits::Operate(bool incremental)
{
  StartRunTime();

	if (firstCall)
	{
		imgWidth = VisionCore::IW();
		imgHeight = VisionCore::IH();
		firstCall = false;
	} 

  StopRunTime();
}

/**
*		OperateNonIncremental()
*		call from VisionCore after incremental operating
*/
void FormExits::OperateNonIncremental()
{
  StartRunTime();
	FindExit();

	Rank();
	Mask();

	StopRunTime();
}

/**
*		FindExit()
*		
*/
void FormExits::FindExit()
{
	for(unsigned i=0; i<NumWalls(); i++)
	{
		// only wall lines without a corner
		if(!Walls(i)->corner && Walls(i)->masked == UNDEF_ID)
		{	
if(findExit) printf("Search WallLine for Wall %u to perform an exit!\n", i);

			// find wall lines which intersect with the wall
			Array<unsigned> wLines;										// wall lines which intersect with the wall
			Array<Vector2> wLinesIsct;								// intersection point of the wall line
			Array<Vector2> wLinesStart;								// start point of the wall line
			for(unsigned j=0; j<NumWallLines(); j++)
			{
				if(WallLines(j)->outLineBorder == 1)
				{
if(findExit) printf("outLineBorder %u found\n", j);

					// Intersecting wallLine
					if(LinesIntersecting(Walls(i)->borderPoints[0], Walls(i)->borderPoints[1], WallLines(j)->point[0], WallLines(j)->point[1]))
					{
						Vector2 isct = LineIntersection(Walls(i)->borderPoints[0], Walls(i)->borderPoints[1] - Walls(i)->borderPoints[0],
															 WallLines(j)->point[0], WallLines(j)->point[1] - WallLines(j)->point[0]);
if(findExit) printf("	Intersection found: %4.2f - %4.2f\n", isct.x, isct.y);

						// find line from WallLine, which end ist closest to the wall (intersection)
						double smallestDistance = 10000.; 	// distance from last line to intersection
						unsigned bestLine;									// line with the smallest gap to the intersection
						Vector2 bestLineEnd;								// end point of best line
						for(unsigned k=0; k<WallLines(j)->lines.Size(); k++)
						{
							Vector2 lineEnd = Lines(WallLines(j)->lines[k])->point[Other(WallLines(j)->linesNearPoints[k])];
							double distance = Length(lineEnd - isct);

							if(distance < smallestDistance)
							{
								bestLine = k;
								smallestDistance = distance;
								bestLineEnd = lineEnd;
							}
						}

if(findExit) printf("		lineEnd-distance for best line %u: %4.2f\n", WallLines(j)->lines[bestLine], smallestDistance);
						// recalculate line intersection
						isct = LineIntersection(Walls(i)->borderPoints[0], Walls(i)->borderPoints[1] - Walls(i)->borderPoints[0],
															 WallLines(j)->point[0], bestLineEnd - WallLines(j)->point[0]);

if(findExit) printf("		bestLineEnd: %4.2f - %4.2f	isct: %4.2f - %4.2f\n", bestLineEnd.x, bestLineEnd.y, isct.x, isct.y);

						// save all possible lines, start-/end- and intersection-points
						wLines.PushBack(j);
						wLinesIsct.PushBack(isct);
						wLinesStart.PushBack(WallLines(j)->point[0]);
					}
					else	// Non-intersecting lines
					{
						Vector2 isct = LineIntersection(Walls(i)->borderPoints[0], Walls(i)->borderPoints[1] - Walls(i)->borderPoints[0],
															 WallLines(j)->point[0], WallLines(j)->point[1] - WallLines(j)->point[0]);

						// calculate gap between wallLine[End] and intersection
						double gap = Length(isct - WallLines(j)->point[1]);

if(findExit) printf("	Non intersecting line %u with gap: %4.2f\n", j, gap);
						if (gap < 20.)																				// TODO Threshold for lines which not intersect with wall
						{
							wLines.PushBack(j);
							wLinesIsct.PushBack(isct);
							wLinesStart.PushBack(WallLines(j)->point[0]);
						}
					}
				}
			}
if(findExit) printf("Find the 2 best wallLines\n");
			/// try to find the 2 best wallLines 
			if (wLines.Size() > 1) 
			{
				/// kriterien zum finden?
				// - weiteste linie links und rechts?
				// - beste winkelübereinstimmung?
				// - weitester Abstand?


				// greatest distance between intersection-points
				bool found = false;
				double reference = 0.;
				unsigned bestWLines[2];
				for(unsigned k=0; k<wLines.Size()-1; k++)
				{
// printf("k=%u\n", k);
// printf("wLines.Size(): %u - wLinesIsct.Size(): %u\n", wLines.Size(), wLinesIsct.Size());
					// calculate distances between all lines
					for(unsigned l=k+1; l<wLines.Size(); l++)
					{
// printf("k=%u  //  l=%u\n", k, l);
						if(Length(wLinesIsct[k] - wLinesIsct[l]) > reference)
						{
// printf("1 - ");
							reference = Length(wLinesIsct[k] - wLinesIsct[l]);
// printf("2 - ");
							bestWLines[0] = k;
// printf("3 - ");
							bestWLines[1] = l;
// printf("no\n");
							found = true;
						}
// 						else 
// printf("Length = false\n");
					}
				}

				if(found)
				{	
if(findExit) printf("bestLines: %u - %u\n", wLines[bestWLines[0]], wLines[bestWLines[1]]);
	
					Vector2 wallBorder[2];
					wallBorder[0] = Walls(i)->borderPoints[0];
					wallBorder[1] = Walls(i)->borderPoints[1];
	
					Vector2 exitLinesStart[2];
					exitLinesStart[0] = wLinesStart[bestWLines[0]];
					exitLinesStart[1] = wLinesStart[bestWLines[1]];
	
					Vector2 exitLinesIntersection[2];
					exitLinesIntersection[0] = wLinesIsct[bestWLines[0]];
					exitLinesIntersection[1] = wLinesIsct[bestWLines[1]];
	
					// recalculate exitLinesStart
					Vector2 border[2];
					border[0].x = 0.;
					border[0].y = 0.;
					border[1].x = imgWidth;
					border[1].y = 0.;

if(findExit) printf("Lines intersect? => %4.2f, %4.2f, %4.2f, %4.2f", border[0].x, border[0].y, border[1].x, border[1].y);
					exitLinesStart[0] = LineIntersection(exitLinesIntersection[0], exitLinesStart[0] - exitLinesIntersection[0], border[0], border[1]);
					exitLinesStart[1] = LineIntersection(exitLinesIntersection[1], exitLinesStart[1] - exitLinesIntersection[1], border[0], border[1]);
	
					// create new exit, only if intersections are more than 20 pixels afar
					if(Length(exitLinesIntersection[0] - exitLinesIntersection[1]) > 20.)						// TODO Threshold of 20 pixel
						NewGestalt(new Exit(wallBorder, exitLinesStart, exitLinesIntersection));
				}
			}
			else if(findExit) printf("FormExits: can not find two wall lines\n");
		}

		/// TODO walls with corner?
	}
}



}


















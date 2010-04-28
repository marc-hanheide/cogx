/**
 * @file FormWallLine.hh
 * @author Richtsfeld Andreas
 * @date 2008
 * @version 0.1
 * @brief Class of the Gestalt principle Wall_Line
 **/

/// TODO TODO TODO TODO 
/// The FormWallLines-Class is not incremental
/// If you use this class with incremental processing, you get an combinatorial explosion!!!
/// OperateNonIncremental functions should delete all gestalts, bevor recalculating => But this
/// would also be a hack
/// TODO TODO TODO TODO 

#include "FormWallLines.hh"
#include "VisionCore.hh"
#include "Line.hh"
#include "Collinearity.hh"
#include "WallLine.hh"
#include "Wall.hh"
#include "FormJunctions.hh"

namespace Z
{

static int CmpWallLines(const void *a, const void *b)
{
  if( WallLines(*(unsigned*)a)->sig > WallLines(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormWallLines::Rank()
{
  RankGestalts(Gestalt::WALL_LINE, CmpWallLines);
}

void FormWallLines::Mask()
{
}

bool FormWallLines::NeedsOperate()
{ 
  return needsOperate;	
}

void FormWallLines::Reset(const Image *img)
{
  needsOperate = true;
}

FormWallLines::FormWallLines(Config *cfg) : GestaltPrinciple(cfg)
{
  needsOperate = true;
	firstCall = true;
}

/**
 * @brief InformNewGestalt()
 */
void FormWallLines::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
//   StartRunTime();
//   StopRunTime();
}

/**
 * @brief Operate()
 */
void FormWallLines::Operate(bool incremental)
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
 * @brief Call from VisionCore after incremental operating
 */
void FormWallLines::OperateNonIncremental()
{
  StartRunTime();

	// clear the outLine arrays for the next calculation
	outLines.Clear();
	outLinesNearPoints.Clear();
	outLinesBorders.Clear();

	SearchLinesForWalls();				// search lines for building walls

	Rank();

	StopRunTime();
}

/**
 * @brief SearchLinesForWalls()
 */
void FormWallLines::SearchLinesForWalls()
{
	// get all lines
 	VoteImage *vote_img = FormJunctions::vote_img;
	unsigned lines = FormJunctions::baseOffset;			// number of lines (with search lines (no split lines, no e-junction search lines))
	unsigned sline, eline;

// printf("FormWallLines: lines do not intersect-fehler?");

	for(unsigned i=0; i<lines; i++)
	{
		sline = i*12 + 2;															// tangential search line at start
		eline = i*12 + 3;															// tangential search line at end
		switch (vote_img->AtImageBorder(sline))
	  {
			case 0: 	// left image border
				outLines.PushBack(i);
				outLinesNearPoints.PushBack(END);
				outLinesBorders.PushBack(0);
				break;
			case 1: 	// top image border
				outLines.PushBack(i);
				outLinesNearPoints.PushBack(END);
				outLinesBorders.PushBack(1);
				break;
			case 2: 	// right image border
				outLines.PushBack(i);
				outLinesNearPoints.PushBack(END);
				outLinesBorders.PushBack(2);
				break;
			case 3: 	// bottom image border
				outLines.PushBack(i);
				outLinesNearPoints.PushBack(END);
				outLinesBorders.PushBack(3);
				break;
			default: break;
		}

		switch (vote_img->AtImageBorder(eline))
	  {
			case 0: 	// left image border
				outLines.PushBack(i);
				outLinesNearPoints.PushBack(START);
				outLinesBorders.PushBack(0);
				break;
			case 1: 	// top image border
				outLines.PushBack(i);
				outLinesNearPoints.PushBack(START);
				outLinesBorders.PushBack(1);
				break;
			case 2: 	// right image border
				outLines.PushBack(i);
				outLinesNearPoints.PushBack(START);
				outLinesBorders.PushBack(2);
				break;
			case 3: 	// bottom image border
				outLines.PushBack(i);
				outLinesNearPoints.PushBack(START);
				outLinesBorders.PushBack(3);
				break;
			default: break;
		}
	}
// printf(" NO\n");

	// Find all next lines for the outLines
	for(unsigned i=0; i<outLines.Size(); i++)
	{
		Array<unsigned> nextLines;										// the coll lines from outLines
		Array<unsigned> nextLinesNearPoints;					// near points of the nextLines
		Array<unsigned> lastLines;										// the foregoing lines 

		/// split-lines: if outLine is part of a split line, get the origin outLine
  	Array<unsigned> col = Lines(outLines[i])->coll[Other(outLinesNearPoints[i])];

		if (col.Size() != 0)
		{
			while (col.Size() != 0)
			{
				/// get the other line
				if (Collinearities(col[0])->line[0] == outLines[i])
				{
					outLines[i] = Collinearities(col[0])->line[1];
					outLinesNearPoints[i] = Collinearities(col[0])->near_point[1];
				}
				else if (Collinearities(col[0])->line[1] == outLines[i])
				{
					outLines[i] = Collinearities(col[0])->line[0];
					outLinesNearPoints[i] = Collinearities(col[0])->near_point[0];
				}
				col.Clear();

				// if there is another split in the line, get the next collinearity
				col = Lines(outLines[i])->coll[Other(outLinesNearPoints[i])];

			}
		}

		GetNextLines(outLines[i], outLinesNearPoints[i], nextLines, nextLinesNearPoints, lastLines);

		CreateWallLine(outLines[i], Other(outLinesNearPoints[i]), outLinesBorders[i], nextLines, nextLinesNearPoints, lastLines);
	}
}


/**
 *	@brief Get all lines, which are connected with collinearities to "line" at "lineNearPoint" => store it in nextLines
 */
void FormWallLines::GetNextLines(unsigned line, unsigned lineNearPoint, Array<unsigned> &nextLines, Array<unsigned> &nextLinesNearPoints, 
			Array<unsigned> &lastLines)//, bool firstLine)	/// TODO FirstLine sinnlos?
{
  // get all collinearities on line end
  Array<unsigned> col = Lines(line)->coll[lineNearPoint];

	// get all second lines from the collinearities
  for(unsigned i=0; i<col.Size(); i++)
  {
 		if (Collinearities(col[i])->line[0] == line)
		{
			nextLines.PushBack(Collinearities(col[i])->line[1]);
			nextLinesNearPoints.PushBack(Collinearities(col[i])->near_point[1]);
			lastLines.PushBack(line);
			
			unsigned l = Collinearities(col[i])->line[1];
			unsigned lnp = Other(Collinearities(col[i])->near_point[1]);
			GetNextLines(l, lnp, nextLines, nextLinesNearPoints, lastLines);
		}
		if (Collinearities(col[i])->line[1] == line)
		{
			nextLines.PushBack(Collinearities(col[i])->line[0]);
			nextLinesNearPoints.PushBack(Collinearities(col[i])->near_point[0]);
			lastLines.PushBack(line);
			
			unsigned l = Collinearities(col[i])->line[0];
			unsigned lnp = Other(Collinearities(col[i])->near_point[0]);
			GetNextLines(l, lnp, nextLines, nextLinesNearPoints, lastLines);
		}
	}
}


/**
**	CreateWallLine()
**	Take all found lines and extract the several lines
*/
void FormWallLines::CreateWallLine(unsigned outLine, unsigned outLineNearPoint, unsigned outLineBorder,
			Array<unsigned> &nextLines, Array<unsigned> &nextLinesNearPoints, Array<unsigned> &lastLines)
{
	bool first = true;
	Array<unsigned> oneLine;						// one Line of lines with collinearities
	Array<unsigned> oneLineNearPoint;		// near points of the lines from oneLine

	// if only outLine and no nextLines
	if(nextLines.Size() == 0)
	{
		oneLine.PushBack(outLine);
		oneLineNearPoint.PushBack(outLineNearPoint);

		MakeWallLine(outLine, outLineBorder, oneLine, oneLineNearPoint);
	}

	for(unsigned i=0; i<nextLines.Size(); i++)															// get all nextLines
	{
		if(first) 																														// push back the outLine and the first nextLines at first
		{	
			oneLine.PushBack(outLine);
			oneLineNearPoint.PushBack(outLineNearPoint);
			oneLine.PushBack(nextLines[i]);
			oneLineNearPoint.PushBack(nextLinesNearPoints[i]);
			first = false;
		}
		else
		{																																			// push back nextLines if lastLines is equal last nextLine
			if(nextLines[i-1] == lastLines[i])
			{
				oneLine.PushBack(nextLines[i]);
				oneLineNearPoint.PushBack(nextLinesNearPoints[i]);
			}
			else																																// make new wall line if it is last line and erase 
			{
				MakeWallLine(outLine, outLineBorder, oneLine, oneLineNearPoint);
				if (oneLine.Contains(lastLines[i]))
				{
					unsigned erase = oneLine.Find(lastLines[i]);										// erase all lines from the end to the lastLine

					for(unsigned j=oneLine.Size()-1; j>erase; j--)
					{
						oneLine.Erase(j);
						oneLineNearPoint.Erase(j);
					}

					oneLine.PushBack(nextLines[i]);																	// push back the new line
					oneLineNearPoint.PushBack(nextLinesNearPoints[i]);
				}
				else							/// TODO ARI: Darf nicht passieren, oder?
				{
						printf("FromWallLines::CreateWallLine: undefined status\n");
				}
			}
		}
 		if(i == nextLines.Size()-1 && oneLine.Size() > 0)											// end of nextLines reached => create Wall
 			MakeWallLine(outLine, outLineBorder, oneLine, oneLineNearPoint);
	}
}


/**
**	MakeWallLine()
*/
void FormWallLines::MakeWallLine(unsigned outLine, unsigned outLineBorder, Array<unsigned> &oneLine, Array<unsigned> &oneLineNearPoint)
{
	/// TODO ARI: nur Linen verwenden, deren phi sich nicht um mehr als 0.5? unterscheidet? (damit reißen Linien wahrscheinlich schnell ab)

//  	printf("\noneLine für outLine: %u\n", outLine);
// 	for(unsigned i=0; i<oneLine.Size(); i++)															// get all nextLines
// 		printf("oneLine[%u]: %u 	nearPoint: %u\n", i, oneLine[i], oneLineNearPoint[i]);


	// only if wallLine does not exist
	if(!IsWallLine(oneLine))
	{
		/// Try to find wall line which has an outLine at start and end of the wall line
		unsigned lastLine = oneLine[oneLine.Size()-1];
		if(outLines.Contains(lastLine) && oneLine.Size() > 1)
		{
			NewGestalt(new WallLine(outLine, outLineBorder, oneLine, oneLineNearPoint, true));					// wall line with outLine at Start/End
		}
		else
		{
			NewGestalt(new WallLine(outLine, outLineBorder, oneLine, oneLineNearPoint, false));					// TODO ARI: diese Zeile wieder weg!
		}
	}
}


/**
**	Returns true, if same wallLine exists already
**/
bool FormWallLines::IsWallLine(Array<unsigned> nLines)
{
	bool isWallLine;
	// get every wallLine
	for(unsigned i=0; i<NumWallLines(); i++)
	{
		isWallLine = true;
		Array<unsigned> lines = WallLines(i)->lines;
		for(unsigned j=0; j<nLines.Size(); j++)
		{
			if(!(lines.Contains(nLines[j]))) isWallLine = false;
		}
		if(isWallLine) return true;
	}
	return false;
}


}






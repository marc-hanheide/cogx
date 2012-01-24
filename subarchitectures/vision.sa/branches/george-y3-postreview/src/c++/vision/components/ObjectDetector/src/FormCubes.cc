/**
 * @file FormCubes.cc
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief Class of Gestalt principle cube.
 **/

#include "math.h"
#include "Array.hh"
#include "FormCubes.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "Rectangle.hh"
#include "Flap.hh"
#include "Cube.hh"

namespace Z
{

static int CmpCubes(const void *a, const void *b)
{
  if( Cubes(*(unsigned*)a)->sig > Cubes(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

FormCubes::CubeCand::CubeCand(unsigned f, Array<unsigned> *l, Array<unsigned> *lNP)
{
  flap = f;

  lines[0] = l[0];  
  lines[1] = l[1];

  linesNearPoints[0] = lNP[0];
  linesNearPoints[1] = lNP[1];
}


void FormCubes::Rank()
{
  RankGestalts(Gestalt::CUBE, CmpCubes);
}

/**
 * @brief Mask cubes which have the center point inside the cube radius of another cube with higher significance.
 */
void FormCubes::Mask()										
{
  for(unsigned i=0; i<NumCubes(); i++)
  {
		for(unsigned j=0; j<NumCubes(); j++)
		{
			if(!Cubes(i)->IsMasked() && !Cubes(j)->IsMasked())
			if(Cubes(i)->sig < Cubes(j)->sig)
			{
				if(Cubes(i)->IsInside(j))
				{	
					Cubes(i)->Mask(j);		 
				}
			}		  
		}
  }
}

void FormCubes::Reset(const Image *img)
{
  cands = 0;	
}


FormCubes::FormCubes(Config *cfg)
: GestaltPrinciple(cfg)
{
}


bool FormCubes::NeedsOperate()
{
  return false;
}


/**
**	InformNewGestalt()
**	receive new Gestalts (flap, collinearities, l_junction
*/
void FormCubes::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  switch(type)
  {
    case Gestalt::FLAP:
      Create(idx);		
      break;
	case Gestalt::COLLINEARITY:
	  NewCollinearity(idx);
	  break;
	case Gestalt::L_JUNCTION:
	  NewLJunction(idx);
	  break;
    default:
      break;
  }
  Rank();
  Mask();
  StopRunTime();
}


void FormCubes::Operate(bool incremental)
{
  //Create();
}


void FormCubes::Create(unsigned flap)
{
  // create cubes from flaps
  bool found = false;
  found = CreateFromFlaps(flap);												
	
  // create cube from flap and lines
  if (!found) CloseFlap(flap);
}


/**
** Create cube, based on three flaps
*/
bool FormCubes::CreateFromFlaps(unsigned flap)
{
  unsigned rects[2];
  rects[0] = Flaps(flap)->rects[0];
  rects[1] = Flaps(flap)->rects[1];
						
  // get every flap
  for (unsigned i=0; i<(NumFlaps()-1); i++)
  {	  
	unsigned openRects[2];
	unsigned r[2];
	r[0] = Flaps(i)->rects[0];
	r[1] = Flaps(i)->rects[1];
	  
	// one rectangle of the flap is equal
	bool found = false;
	if (r[0] == rects[0] && r[1] != rects[1]) 
	{
	  found = true;
	  openRects[0] = rects[1];
	  openRects[1] = r[1];
	}
	if (r[0] == rects[1] && r[1] != rects[0])
	{
	  found = true;
	  openRects[0] = rects[0];
	  openRects[1] = r[1];
	}
	if (r[1] == rects[0] && r[0] != rects[1])
	{
	  found = true;
	  openRects[0] = rects[1];
	  openRects[1] = r[0];
	}
	if (r[1] == rects[1] && r[0] != rects[0])
	{
	  found = true;
	  openRects[0] = rects[0];
	  openRects[1] = r[0];
	}

  // find a third flap with the two other rectangles
	if (found)
	{
	  for (unsigned j=0; j<(NumFlaps()-1); j++)
	  {
		unsigned rof[2];
		rof[0] = Flaps(j)->rects[0];
		rof[1] = Flaps(j)->rects[1];
		  
		if ((rof[0] == openRects[0] && rof[1] == openRects[1]) ||
			(rof[0] == openRects[1] && rof[1] == openRects[0]))
		{
			
		  // the 3 flaps have to be ordered counter-clockwise
		  if (Flaps(flap)->rects[0] == Flaps(i)->rects[1])
			NewGestalt(new Cube(flap, i, j));	
		  if (Flaps(flap)->rects[0] == Flaps(j)->rects[1])
			NewGestalt(new Cube(flap, j, i));	
		  
		  return true;
		}	  
	  }
	}
  }
  return false;
}

/**
**	CloseFlap()
**	close a flap to a cube with one l-junction (or collinearities) 
**	and with lines 
*/
void FormCubes::CloseFlap(unsigned flap)
{ 
  bool colClosed = false;
	bool lClosed = false;
  Array<unsigned> lines[2];								// lines[LEFT/RIGHT]
  Array<unsigned> linesNearPoints[2];			// near_points of the lines
  Array<unsigned> lastLines[2];						// the foregoing line (collinearity)
	
  GetLines(flap, lines, linesNearPoints, lastLines);
		
  lClosed = TryFlapClosing(flap, lines, linesNearPoints, lastLines);

	colClosed = TryFlapClosingWithCollinearity(flap, lines, linesNearPoints, lastLines);
	
  if (!lClosed && !colClosed)
	cands.PushBack(new CubeCand(flap, lines, linesNearPoints));			// TODO ARI: ist das speichern der lines, near_points sinnvoll? brauch ich die?
}


/**
**	Get all connected lines from the outer junctions of a flap
**	(first as l-junction and then collinearity-lines)
*/
void FormCubes::GetLines(unsigned flap, Array<unsigned> *lines, 
	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines)
{
  // get the lines from the 2 outer junctions (outerJcts[0] and outerJcts[2]) for closing the flap
  unsigned *outerJcts = Flaps(flap)->outerJcts;
  Array<unsigned> jctLines[2];							// [LEFT/RIGHT]
  Array<unsigned> jctLinesNearPoint[2];			// [LEFT/RIGHT]

  jctLines[LEFT].PushBack(LJunctions(outerJcts[0])->line[LEFT]);
  jctLinesNearPoint[LEFT].PushBack(LJunctions(outerJcts[0])->near_point[LEFT]);
  jctLines[LEFT].PushBack(LJunctions(outerJcts[0])->line[RIGHT]);
  jctLinesNearPoint[LEFT].PushBack(LJunctions(outerJcts[0])->near_point[RIGHT]);

  jctLines[RIGHT].PushBack(LJunctions(outerJcts[2])->line[LEFT]);
  jctLinesNearPoint[RIGHT].PushBack(LJunctions(outerJcts[2])->near_point[LEFT]);
  jctLines[RIGHT].PushBack(LJunctions(outerJcts[2])->line[RIGHT]);
  jctLinesNearPoint[RIGHT].PushBack(LJunctions(outerJcts[2])->near_point[RIGHT]);

  // get all lines who have a l-junction to one of the jctLines
  for(unsigned fSide=LEFT; fSide<=RIGHT; fSide++)
  {
		// get every junction-line [LEFT or RIGHT]									// gleiche Linien der jctLines auch verarbeiten? Ja, weil versch. end
    for (unsigned i=0; i<jctLines[fSide].Size(); i++)
    {
			// get all L-Junctions from the jctLines at jctLinesNearPoint
			Array<unsigned> lj;
			Lines(jctLines[fSide][i])->GetLJunctions(jctLinesNearPoint[fSide][i], lj);

	  	// every l-jcts from one of the line									// Überprüfen, ob L-Junction schon dabei war (alreadyCalculated)  
			for(unsigned j=0; j<lj.Size(); j++)
			{
				// get the two lines and nearPoints of the l-junction
				unsigned line[2]; 
				unsigned nearPoint[2];
				line[0] = LJunctions(lj[j])->line[0];
				line[1] = LJunctions(lj[j])->line[1];
				nearPoint[0] = LJunctions(lj[j])->near_point[0];
				nearPoint[1] = LJunctions(lj[j])->near_point[1];
	
				// estimate the other line of the l-junction and push back to lines
				if(jctLines[fSide].Contains(line[0]) && !lines[fSide].Contains(line[1]) &&
						!Closures(Rectangles(Flaps(flap)->rects[0])->clos)->lines.Contains(line[1])  &&
						!Closures(Rectangles(Flaps(flap)->rects[1])->clos)->lines.Contains(line[1]))
				{ 
					lines[fSide].PushBack(line[1]);	
					linesNearPoints[fSide].PushBack(nearPoint[1]);
					lastLines[fSide].PushBack(UNDEF_ID);			
					GetNextLines(line[1], Other(nearPoint[1]), lines, linesNearPoints, lastLines, fSide, false);
				}
				else if (jctLines[fSide].Contains(line[1]) && !lines[fSide].Contains(line[0]) &&
					!Closures(Rectangles(Flaps(flap)->rects[0])->clos)->lines.Contains(line[0])  &&
					!Closures(Rectangles(Flaps(flap)->rects[1])->clos)->lines.Contains(line[0]))
				{
					lines[fSide].PushBack(line[0]);	
					linesNearPoints[fSide].PushBack(nearPoint[0]);  
					lastLines[fSide].PushBack(UNDEF_ID);		
					GetNextLines(line[0], Other(nearPoint[0]), lines, linesNearPoints, lastLines, fSide, false);
				}
			}
		} 

		// GetNextLines for all jctLines (Colls of jctLines)	
		for(unsigned i=0; i<jctLines[fSide].Size(); i++)
		{
			GetNextLines(jctLines[fSide][i], jctLinesNearPoint[fSide][i], lines, linesNearPoints, lastLines, fSide, true);
		}
  }
}

/**
**	GetNextLines:
**	Get all lines, witch have a collinearity to the line and all following
**	lines.
*/
void FormCubes::GetNextLines(unsigned line, unsigned lineEnd,					// line END !!!!
	Array<unsigned> *lines, Array<unsigned> *linesNearPoints, 
	Array<unsigned> *lastLines, unsigned fSide, bool firstLine)
{
  unsigned l, lE;		// new line and lineEnd
	
  // Get all Collinearities on lineEnd
  Array<unsigned> col = Lines(line)->coll[lineEnd];
	
  // get all lines of the found collinearities
  for(unsigned i=0; i<col.Size(); i++)
  {
		if (Collinearities(col[i])->line[0] == line &&																/// TODO: don´t push back if line is line of closure
			!lines[fSide].Contains(Collinearities(col[i])->line[1]))
		{																			// and line is already in lines
			lines[fSide].PushBack(Collinearities(col[i])->line[1]);
			linesNearPoints[fSide].PushBack(Collinearities(col[i])->near_point[1]);	
			if (firstLine) 
				lastLines[fSide].PushBack(UNDEF_ID);
			else 
				lastLines[fSide].PushBack(Collinearities(col[i])->line[0]);			
			l = Collinearities(col[i])->line[1];
			lE = Other(Collinearities(col[i])->near_point[1]);
			GetNextLines(l, lE, lines, linesNearPoints, lastLines, fSide, false);
		}
		else if (Collinearities(col[i])->line[1] == line &&														/// TODO: don´t push back if line is line of closure
			!lines[fSide].Contains(Collinearities(col[i])->line[0]))
		{
			lines[fSide].PushBack(Collinearities(col[i])->line[0]);
			linesNearPoints[fSide].PushBack(Collinearities(col[i])->near_point[0]);
			if(firstLine) 
				lastLines[fSide].PushBack(UNDEF_ID);
			else 
				lastLines[fSide].PushBack(Collinearities(col[i])->line[1]);			
			l = Collinearities(col[i])->line[0];
			lE = Other(Collinearities(col[i])->near_point[0]);
			GetNextLines(l, lE, lines, linesNearPoints, lastLines, fSide, false);
		}
  }
}


/**
**	Try to close the flap to get a cube
*/
bool FormCubes::TryFlapClosing(unsigned flap, Array<unsigned> *lines,			// TODO ARI: real near points
	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines)
{
	// get every line from left junctions of flap
	unsigned fSide = LEFT;			
	for(unsigned i=0; i<lines[fSide].Size(); i++)
	{
	  // get all L-Junctions from line end: Other(near_point)
	  Array<unsigned> ljct;
	  unsigned end = Other(linesNearPoints[fSide][i]);
	  Lines(lines[fSide][i])->GetLJunctions(end, ljct);


	  // get every l-junction 			
	  for(unsigned j=0; j<ljct.Size(); j++)
	  {		  
			unsigned secondLine;				// number of the second line from the L-Junction
			unsigned secondLineSide; 		// LEFT/RIGHT line of L-Junction

			if (LJunctions(ljct[j])->line[0] == lines[fSide][i])
			{
				secondLine = LJunctions(ljct[j])->line[1];
				secondLineSide = 1;
			}
			else
			{
				secondLine = LJunctions(ljct[j])->line[0];
				secondLineSide = 0;
			}

			// if other side contains the second line and linesNearPoint is equal nearPoint of line from L-Junction
			if (lines[Other(fSide)].Contains(secondLine) && linesNearPoints[fSide][i] == LJunctions(ljct[j])->near_point[secondLineSide])
			{
				unsigned linePos0 = i;																				// position in left lines-array
				unsigned linePos1 = lines[Other(fSide)].Find(secondLine);			// position in right lines-array

				Array<unsigned> foregoing[2]; 				// find foregoing lines [LEFT/RIGHT]
	
				while (lastLines[fSide][linePos0] != UNDEF_ID)
				{ 
					foregoing[fSide].PushBack(lastLines[fSide][linePos0]);
					linePos0 = lines[fSide].Find(lastLines[fSide][linePos0]);
				} 
	
				while (lastLines[Other(fSide)][linePos1] != UNDEF_ID)
				{
					foregoing[Other(fSide)].PushBack(lastLines[Other(fSide)][linePos1]);
					linePos1 = lines[Other(fSide)].Find(lastLines[Other(fSide)][linePos1]);
				}

				NewGestalt(new Cube(flap, ljct[j], UNDEF_ID, lines[fSide][i], secondLine, foregoing));

				return true;
			}
	  }
	}
  return false;
}


/**
**	Try to close the flap to get a cube with the lines
*/
bool FormCubes::TryFlapClosingWithCollinearity(unsigned flap, Array<unsigned> *lines,
	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines)
{
//printf("Try flap closing of flap: %u\n", flap);

  // Remember the colls, the j and i and the second lines
  Array<unsigned> foundColls;		// the found Colls
  Array<unsigned> js;						// the j
  Array<unsigned> is;						// the i
  Array<unsigned> seconds; 			// the second lines

	// get every line from left junctions of flap
	unsigned fSide = LEFT;	
	for(unsigned i=0; i<lines[fSide].Size(); i++)
	{
	  // get all Collinearities from line end: Other(near_point)
	  Array<unsigned> coll;
	  unsigned end = Other(linesNearPoints[fSide][i]);
	  Lines(lines[fSide][i])->GetCollinearities(end, coll);
		
	  // get every collinearity 			
	  for(unsigned j=0; j<coll.Size(); j++)
	  {		  
			// get 2nd line of the collinearity
			unsigned secondLine;
			if (Collinearities(coll[j])->line[0] == lines[fSide][i])
				secondLine = Collinearities(coll[j])->line[1];
			else
				secondLine = Collinearities(coll[j])->line[0];
			
			if (lines[Other(fSide)].Contains(secondLine))
			{
				// remember all found colls with the second and foregoing lines
				foundColls.PushBack(coll[j]);
				is.PushBack(i);
				js.PushBack(j);
				seconds.PushBack(secondLine);
			}
	  }	  
	}


	if (foundColls.Size() > 0)
	{
		/// do not search the smallest angle => bad results at flap-corners
		// get coll with the smallest angle
// 		double ref = 3.142;
// 		unsigned smallestColl;
// 		for (unsigned i=0; i<foundColls.Size(); i++)
// 		{
// 		  if (Collinearities(foundColls[i])->OpeningAngle() < ref)
// 		  {
// 				smallestColl = i;
// 				ref = Collinearities(foundColls[i])->OpeningAngle();
// 		  }
// 		}


		// calculate, where the corner point of a cube should be
		Vector2 corner = LJunctions(Flaps(flap)->outerJcts[2])->isct +
										 LJunctions(Flaps(flap)->outerJcts[0])->isct -
										 LJunctions(Flaps(flap)->innerJcts[1])->isct;

		// search the collinearity with which is the nearest to the corner-point of the flap
		double refVertex = 1000.;
 		unsigned smallestColl;		for (unsigned i=0; i<foundColls.Size(); i++)
		{
		  if ((Collinearities(foundColls[i])->vertex - corner).Norm() < refVertex)
		  {
				smallestColl = i;
				refVertex = (Collinearities(foundColls[i])->vertex - corner).Norm();
		  }
		}

		
		unsigned linePos0 = is[smallestColl];
		unsigned linePos1 = lines[Other(fSide)].Find(seconds[smallestColl]);

		Array<unsigned> foregoing[2]; // [LEFT/RIGHT]

		while (lastLines[fSide][linePos0] != UNDEF_ID)
		{ 
		  foregoing[fSide].PushBack(lastLines[fSide][linePos0]);
		  linePos0 = lines[fSide].Find(lastLines[fSide][linePos0]);
		} 

		while (lastLines[Other(fSide)][linePos1] != UNDEF_ID)
		{
		  foregoing[Other(fSide)].PushBack(lastLines[Other(fSide)][linePos1]);
		  linePos1 = lines[Other(fSide)].Find(lastLines[Other(fSide)][linePos1]);
		}

	  	NewGestalt(new Cube(flap, UNDEF_ID, foundColls[smallestColl], lines[fSide][linePos0], seconds[smallestColl], foregoing));
		return true;
	}

  return false;
}

/*
**	Update the cube candidates, if one of the collinearity-lines is part
**	of cube-candidates
*/
void FormCubes::NewCollinearity(unsigned col)
{
  unsigned line[2]; 
  line[0] = Collinearities(col)->line[0];
  line[1] = Collinearities(col)->line[1];

  for(unsigned i=0; i<cands.Size(); i++)
  {
	if (cands[i]->lines[0].Contains(line[0]) ||
		cands[i]->lines[1].Contains(line[0]) ||
		cands[i]->lines[0].Contains(line[1]) ||
		cands[i]->lines[1].Contains(line[1]))
	  UpdateCubeCand(i);
  }
}

/*
**	Try to close the flap, if one of the lines from the L-Junction is 
**	part of an cube candidate
*/
void FormCubes::NewLJunction(unsigned ljct)
{
  unsigned line[2]; 
  line[0] = LJunctions(ljct)->line[0];
  line[1] = LJunctions(ljct)->line[1];
	
  for(unsigned i=0; i<cands.Size(); i++)
  {
	if (cands[i]->lines[0].Contains(line[0]) ||
		cands[i]->lines[1].Contains(line[0]) ||
		cands[i]->lines[0].Contains(line[1]) ||
		cands[i]->lines[1].Contains(line[1]))
	  UpdateCubeCand(i);
  }
}

/*
**	Update Cube Candidate
*/
void FormCubes::UpdateCubeCand(unsigned cand)
{
  unsigned flap = cands[cand]->flap;
  Array<unsigned> lines[2];
  Array<unsigned> linesNearPoints[2];
  Array<unsigned> lastLines[2];
	
  GetLines(flap, lines, linesNearPoints, lastLines);
	
  bool closed = TryFlapClosing(flap, lines, linesNearPoints, lastLines);

  if (closed)
		cands.Erase(cand);															
  else																	// TODO ARI: ist diese Speicherung sinnvoll? nur flap und lines?
  {
		cands[cand]->lines[0] = lines[0];
		cands[cand]->lines[1] = lines[1];
		cands[cand]->linesNearPoints[0] = linesNearPoints[0];
		cands[cand]->linesNearPoints[1] = linesNearPoints[1];
		cands[cand]->lastLines[0] = lastLines[0];
		cands[cand]->lastLines[1] = lastLines[1];
  }
}






/*
**	TODO ARI: VERALTET
**	Achtung: diese Funktion ist nicht inkrementell: Es werden
**	nur die Linien einmal nach Junctions durchsucht. Es sollte eine Funktion
** 	geben, die neue Junctions zu den Flaps zuordnet und damit erweitert.
**
**	Try to close the flap: Find 2 lines with intersections in outerJunctions
**	of the flap and with intersection itselfs. 
**

void FormCubes::CloseFlap(unsigned flap, unsigned r0, unsigned r1, unsigned *oJ)
{
//printf("\nFLAP: %i\n\n", flap);
  // get the outer junctions of the flap
  unsigned outerJcts[4];
  for (int i=0; i<4; i++){ outerJcts[i] = oJ[i];
  }
	
  // get the 8 lines from the outer junctions
  unsigned jctLines[8];
  for (int i=0; i<4; i++) 
  {
	jctLines[i*2] = LJunctions(outerJcts[i])->line[LEFT];
	jctLines[i*2+1] = LJunctions(outerJcts[i])->line[RIGHT];
  }
  
  // the junctions and colls where the lines are involved
  Array<unsigned> JctsOfLines;			// found L-Jcts
  Array<unsigned> CollsOfLines;			// found Colls
  Array<unsigned> LineJctsOfLines; 		// 2nd lines of the found L-Jct
  Array<unsigned> LineCollsOfLines; 	// 2nd lines of the found Coll
  
  // for every line of the 8 jctLines
  for (int i=0; i<8; i++) {
	  
	// seperate out the doubled lines from jctLines
	bool alreadyCalculated = false;
	for (int j=0; j<i; j++){
	  if (jctLines[i] == jctLines[j]) alreadyCalculated = true;
	}
	  
 	if (!alreadyCalculated) {
 
	  // check all existing L-Junctions
      for (unsigned j=0; j<NumLJunctions(); j++) {
		// is left line of L-Junction a line from junction of rect
		if (LJunctions(j)->line[LEFT] == jctLines[i]) 
		{
		  // reject if junction is already a "outer Junction"
		  bool isOuterJct = false;
		  for (int k=0; k<4; k++){
		    if (j == outerJcts[k]) isOuterJct = true;	  
		  }
		  
		  // reject if line is from Closure r0
		  bool isClosLine = false;
		  unsigned nrOfClosLines = Closures(Rectangles(r0)->clos)->lines.Size();
		  for (unsigned l=0; l<nrOfClosLines; l++)
		  {
			unsigned line = Closures(Rectangles(r0)->clos)->lines[l];
			if (LJunctions(j)->line[RIGHT] == line && line != UNDEF_ID) 
			  isClosLine = true;
		  }
		  // reject if line is from Closure r1
		  nrOfClosLines = Closures(Rectangles(r1)->clos)->lines.Size();
		  for (unsigned l=0; l<nrOfClosLines; l++)
		  {
			unsigned line = Closures(Rectangles(r1)->clos)->lines[l];
			if (LJunctions(j)->line[RIGHT] == line && line != UNDEF_ID) 
			  isClosLine = true;
		  }
			
		  // junction is no outer junction && line isn´t part of the closure 
		  if (!isOuterJct && !isClosLine){		
			// push back the junction j and the 2nd line from junction j
			if (!JctsOfLines.Contains(j)){	
			  JctsOfLines.PushBack(j);
			  LineJctsOfLines.PushBack(LJunctions(j)->line[RIGHT]);
			}
		  }			
	    }
		
		// is right line of L-Junctions a line from junctions of rect
		if (LJunctions(j)->line[RIGHT] == jctLines[i])
		{
		  // reject if junction is already a "outerJunction"
		  bool isOuterJct = false;
		  for (int k=0; k<4; k++){
		    if (j == outerJcts[k]) isOuterJct = true;	  
		  }
		  
		  // reject if line is from Closure r0
		  bool isClosLine = false;
		  unsigned nrOfClosLines = Closures(Rectangles(r0)->clos)->lines.Size();
		  for (unsigned l=0; l<nrOfClosLines; l++){
			unsigned line = Closures(Rectangles(r0)->clos)->lines[l];
			if (LJunctions(j)->line[LEFT] == line && line != UNDEF_ID)
			  isClosLine = true;
		  }
		  // reject if line is from Closure r1
		  nrOfClosLines = Closures(Rectangles(r1)->clos)->lines.Size();
		  for (unsigned l=0; l<nrOfClosLines; l++)
		  {
			unsigned line = Closures(Rectangles(r1)->clos)->lines[l];
			if (LJunctions(j)->line[LEFT] == line && line != UNDEF_ID) 
			  isClosLine = true;
		  }

		  // junction is no outer junction && line isn´t part of the closure 
		  if (!isOuterJct && !isClosLine){		
			// push back the junction j and the 2nd line from junction j
			if (!JctsOfLines.Contains(j)){	
			  JctsOfLines.PushBack(j);
			  LineJctsOfLines.PushBack(LJunctions(j)->line[LEFT]);
			}
		  }			
	    }
	  }
    }
	
	// TODO ARI: 
	// Get also all Collinearities => lines from Collinearities
  } 

//for(unsigned i=0; i<LineJctsOfLines.Size(); i++)
//	printf("LineJctsOfLines: %i\n", LineJctsOfLines[i]);
//printf("\n");
  
  // Estimate junctions which are near outerJunctions 
  Array<unsigned> goodJcts;			// JctsOfLines which are near a outer junctions
  Array<unsigned> linesOfGoodJcts;	// Lines of the good junctions
  for (unsigned i=0; i<JctsOfLines.Size(); i++)
  {
	// intersection of found L-Junction
	Vector2 isct = LJunctions(JctsOfLines[i])->isct;
	  
	// get the intersection points from outerJunctions
	for(unsigned j=0; j<4; j++)
	{
	  Vector2 outerJctsIsct = LJunctions(outerJcts[j])->isct;
		
	  // calculate distance between the intersection points
	  double sqrX = Sqr(isct.x - outerJctsIsct.x);
	  double sqrY = Sqr(isct.y - outerJctsIsct.y);
	  double dist = sqrt(sqrX + sqrY);

	  // seperate out the bad junctions
	  // TODO ARI: Threshold: dist < 10 pts.
	  if (dist < 15.) 
	  {
		goodJcts.PushBack(JctsOfLines[i]);
		linesOfGoodJcts.PushBack(LineJctsOfLines[i]);
	  }		
	}
  }
//for(unsigned i=0; i<linesOfGoodJcts.Size(); i++)
//	printf("linesOfGoodJcts: %i\n", linesOfGoodJcts[i]);
//printf("\n\n");
  
  // Find all LJcts and Colls between pairs of linesOfGoodJcts
  Array<unsigned> lineLJcts;	// LJcts between pair of linesOfGoodJcts
  Array<unsigned> lineColls;	// Colls between pair of linesOfGoodJcts
	
  // find L-Junction between pair of linesOfGoodJcts
  for (unsigned i=0; i<linesOfGoodJcts.Size(); i++)
  {
	for (unsigned j=0; j<NumLJunctions(); j++)
	{
	  // browse left lines of all LJunctions
	  if (linesOfGoodJcts[i] == LJunctions(j)->line[LEFT])
	  {
		for (unsigned l=0; l<linesOfGoodJcts.Size(); l++)
		{
		  if (linesOfGoodJcts[l] == LJunctions(j)->line[RIGHT])
		  {
			if(!lineLJcts.Contains(j))
			{
			  lineLJcts.PushBack(j);
			}
		  }
		}
	  }
	}
  }	  
  
  // find Collinearities for pair of linesOfGoodJcts
  for (unsigned i=0; i<linesOfGoodJcts.Size(); i++)
  {
	for (unsigned j=0; j<NumCollinearities(); j++)
	{
	  // browse left lines of all collinearities
	  if (linesOfGoodJcts[i] == Collinearities(j)->line[LEFT])
	  {
		for (unsigned l=0; l<linesOfGoodJcts.Size(); l++)
		{
		  if (linesOfGoodJcts[l] == Collinearities(j)->line[RIGHT])
		  {
			if(!lineColls.Contains(j))
			{ 
			  lineColls.PushBack(j);
			}
		  }
		}
	  }
    }
  }	  
  
//for(unsigned i=0; i<lineLJcts.Size(); i++)
//	printf("lineLJcts: %i\n", lineLJcts[i]);
//printf("\n\n");
//for(unsigned i=0; i<lineColls.Size(); i++)
//	printf("lineColls: %i\n", lineColls[i]);
//printf("\n\n");
  
// Create Cube
// überprüfe ob beide Linienenden in der Nähe von outerJunctions sind
// die beiden outerJunctions müssen sich unterscheiden
// die beiden outer Junctions müssen auf der gegenüberliegenden Seite sein
  
  // Create Cube for all lineColls if endpoints of lines near a 
  // outer junction
  for(unsigned i=0; i<lineColls.Size(); i++)
  {
	unsigned foundJct = UNDEF_ID;	// found jct for closing flap
	Vector2 endPointLine0;	// endPoint of first line from Coll
	Vector2 endPointLine1;	// endPoint of second line from Coll
	  
	// get the end points from the two lines of the good Colls
	if (Collinearities(lineColls[i])->near_point[0] == 0)
		endPointLine0 = Lines(Collinearities(lineColls[i])->line[0])->point[1];
	else
		endPointLine0 = Lines(Collinearities(lineColls[i])->line[0])->point[0];

	if (Collinearities(lineColls[i])->near_point[1] == 0)
		endPointLine1 = Lines(Collinearities(lineColls[i])->line[1])->point[1];
	else
		endPointLine1 = Lines(Collinearities(lineColls[i])->line[1])->point[0];
	
	//calculate the minimum distance of the endpoint to the next outerJunction
	double minDist0 = UNDEF_ID;	// min. distance of endPointLine0 to outerJct.
	double minDist1 = UNDEF_ID;	// min. distance of endPointLine1 to outerJct.
	unsigned oJminDist0 = UNDEF_ID; // number of outerJunction of minDist0
	unsigned oJminDist1 = UNDEF_ID; // number of outerJunction of minDist1
		
	// get every outer junction
	for(int j=0; j<4; j++)
	{
	  Vector2 outerJctsIsct = LJunctions(outerJcts[j])->isct;

	  // calculate distance between the intersection point and endPointLine0
	  double sqrX0 = Sqr(endPointLine0.x - outerJctsIsct.x);
	  double sqrY0 = Sqr(endPointLine0.y - outerJctsIsct.y);
	  double dist0 = sqrt(sqrX0 + sqrY0);

	  // calculate distance between the intersection points and endPointLine1
	  double sqrX1 = Sqr(endPointLine1.x - outerJctsIsct.x);
	  double sqrY1 = Sqr(endPointLine1.y - outerJctsIsct.y);
	  double dist1 = sqrt(sqrX1 + sqrY1);
		
	  if (dist0 < minDist0 || minDist1 == UNDEF_ID)
	  {
		minDist0 = dist0;
		oJminDist0 = j; 
	  }
	  if (dist1 < minDist1 || minDist1 == UNDEF_ID)
	  {
		minDist1 = dist1;
		oJminDist1 = j;
	  }	
    }

//printf("minDist: %f - %f\n", minDist0, minDist1);
	
// TODO ARI: Threshold for "near" 
	
	// found a junction for closing
	if (minDist0 < 20 && minDist1 < 20 &&
		((oJminDist0 < 2 && oJminDist1 > 1) || (oJminDist0 > 1 && oJminDist1 < 2)))
	{
	  foundJct = lineColls[i];
		
	  unsigned closingLines[2];
	  closingLines[0] = Collinearities(foundJct)->line[LEFT];
	  closingLines[1] = Collinearities(foundJct)->line[RIGHT];  

	  NewGestalt(new Cube(flap, UNDEF_ID, UNDEF_ID, UNDEF_ID, lineColls[i]));
		  
	  Rank();
	}
  }

  // Create Cube for all lineLJcts if endpoints of lines near a 
  // outer junction
  for(unsigned i=0; i<lineLJcts.Size(); i++)
  {
	unsigned foundJct = UNDEF_ID;	// found jct for closing flap
	Vector2 endPointLine0;	// endPoint of first line from Coll
	Vector2 endPointLine1;	// endPoint of second line from Coll
	  
	// get the end points from the two lines of the good Colls
	if (LJunctions(lineLJcts[i])->near_point[0] == 0)
		endPointLine0 = Lines(LJunctions(lineLJcts[i])->line[0])->point[1];
	else
		endPointLine0 = Lines(LJunctions(lineLJcts[i])->line[0])->point[0];

	if (LJunctions(lineLJcts[i])->near_point[1] == 0)
		endPointLine1 = Lines(LJunctions(lineLJcts[i])->line[1])->point[1];
	else
		endPointLine1 = Lines(LJunctions(lineLJcts[i])->line[1])->point[0];
	
// both endPoints "near" a outerJctsIsct
	//calculate the minimum distance of the endpoint to the next outerJunction
	double minDist0 = UNDEF_ID;	// min. distance of endPointLine0 to outerJct.
	double minDist1 = UNDEF_ID;	// min. distance of endPointLine1 to outerJct.
	unsigned oJminDist0 = UNDEF_ID; // number of outerJunction of minDist0
	unsigned oJminDist1 = UNDEF_ID; // number of outerJunction of minDist1

	// get every outer junction and calculte distance
	for(int j=0; j<4; j++)
	{
	  Vector2 outerJctsIsct = LJunctions(outerJcts[j])->isct;

	  // calculate distance between the intersection point and endPointLine0
	  double sqrX0 = Sqr(endPointLine0.x - outerJctsIsct.x);
	  double sqrY0 = Sqr(endPointLine0.y - outerJctsIsct.y);
	  double dist0 = sqrt(sqrX0 + sqrY0);

	  // calculate distance between the intersection points and endPointLine1
	  double sqrX1 = Sqr(endPointLine1.x - outerJctsIsct.x);
	  double sqrY1 = Sqr(endPointLine1.y - outerJctsIsct.y);
	  double dist1 = sqrt(sqrX1 + sqrY1);
		
	  if (dist0 < minDist0 || minDist1 == UNDEF_ID)
	  {
		minDist0 = dist0;
  		oJminDist0 = j; 
	  }
	  if (dist1 < minDist1 || minDist1 == UNDEF_ID)
	  {
		minDist1 = dist1;
  		oJminDist1 = j; 
	  }	
    }

	// found a junction for closing
	if (minDist0 < 10 && minDist1 < 10 &&
		((oJminDist0 < 2 && oJminDist1 > 1) || (oJminDist0 > 1 && oJminDist1 < 2)))
	{
	  foundJct = lineLJcts[i];
		
	  unsigned closingLines[2];
	  closingLines[0] = LJunctions(foundJct)->line[LEFT];
	  closingLines[1] = LJunctions(foundJct)->line[RIGHT];  

	  NewGestalt(new Cube(flap, UNDEF_ID, UNDEF_ID, lineLJcts[i], UNDEF_ID));
		  
	  Rank();
	}
  }
} 
*/
}

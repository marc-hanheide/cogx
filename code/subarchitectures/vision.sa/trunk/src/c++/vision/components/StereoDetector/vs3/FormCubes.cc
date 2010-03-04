/**
 * @file FormCubes.cc
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief Class of Gestalt principle cube.
 **/

#include "math.h"								/// TODO Für was math.h?
#include "FormCubes.hh"

namespace Z
{

/**
 * @brief Compare function for cubes.
 * @param a First Gestalt to compare.
 * @param b Second Gestalt to compare.
 */
static int CmpCubes(const void *a, const void *b)
{
  if( (*(Cube**)a)->sig > (*(Cube**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}


/**
 * @brief Constructor of cube candiate.
 * @param f Flap of candidate
 * @param l Lines
 * @param lNP Lines near points
 */
FormCubes::CubeCand::CubeCand(unsigned f, Array<unsigned> *l, Array<unsigned> *lNP)
{
  flap = f;

  lines[0] = l[0];  
  lines[1] = l[1];

  linesNearPoints[0] = lNP[0];
  linesNearPoints[1] = lNP[1];
}


/**
 * @brief Rank cubes.
 */
void FormCubes::Rank()
{
  RankGestalts(Gestalt::CUBE, CmpCubes);
}

/**
 * @brief Mask cubes which have the center point inside the cube radius of another cube with higher significance.
 */
void FormCubes::Mask()										
{
  for(unsigned i=0; i<NumCubes(core); i++)
		for(unsigned j=0; j<NumCubes(core); j++)
			if(!Cubes(core, i)->IsMasked() && !Cubes(core, j)->IsMasked())
				if(Cubes(core, i)->sig < Cubes(core, j)->sig)
					if(Cubes(core, i)->IsInside(j))
						Cubes(core, i)->Mask(j);		 
}


/**
 * @brief Reset FormCubes class.
 */
void FormCubes::Reset()
{
  cands = 0;	
}


/**
 * @brief Constructor of FormCubes class.
 * @param vc Vision core
 */
FormCubes::FormCubes(VisionCore *vc) : GestaltPrinciple(vc)
{
}

/**
 * @brief Informs vision core, if principle needs operate.
 * @return Returns true, if FormCubes needs operate call (non-incremental)
 */
bool FormCubes::NeedsOperate()
{
  return false;
}


/**
 * @brief InformNewGestalt: receive new Gestalts from vision core.
 */
void FormCubes::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
//   StartRunTime();
  switch(type)
  {
		case Gestalt::FLAP_ARI:
			Create(idx);		
			break;
		case Gestalt::COLLINEARITY:
// 			NewCollinearity(idx);																												/// TODO War das implementiert?
			break;
		case Gestalt::L_JUNCTION:
// 			NewLJunction(idx);																													/// TODO War das implementiert?
			break;
    default:
			printf("FormCubes::InformNewGestalt: Received unknown Gestalt.\n");
      break;
  }
  Rank();
  Mask();
//   StopRunTime();
}


/**
 * @brief Operate call.
 * @param incremental Operate incremental or non-incremental
 */
void FormCubes::Operate(bool incremental)
{
}


/**
 * @brief Try to create new cubes.
 * @param flap Id of flap
 */
void FormCubes::Create(unsigned flap)
{
  // create cube from flaps
  bool found = false;
  found = CreateFromFlaps(flap);
	
  // create cube from flap and lines
  if (!found) CloseFlap(flap);
}


/**
 * @brief Create cube, based on three flaps.
 * @param flap ID of new flap
 * @return Returns true, if new flap could be found.
 */
bool FormCubes::CreateFromFlaps(unsigned flap)
{
  unsigned rects[2];
  rects[0] = FlapsAri(core, flap)->rectangles[0]->ID();
  rects[1] = FlapsAri(core, flap)->rectangles[1]->ID();
						
  // get every flap
  for (unsigned i=0; i<(NumFlapsAri(core)-1); i++)
  {	  
		unsigned openRects[2];
		unsigned r[2];
		r[0] = FlapsAri(core, i)->rectangles[0]->ID();
		r[1] = FlapsAri(core, i)->rectangles[1]->ID();

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
			for (unsigned j=0; j<(NumFlapsAri(core)-1); j++)
			{
				unsigned rof[2];
				rof[0] = FlapsAri(core, j)->rectangles[0]->ID();
				rof[1] = FlapsAri(core, j)->rectangles[1]->ID();
					
				if ((rof[0] == openRects[0] && rof[1] == openRects[1]) ||
					(rof[0] == openRects[1] && rof[1] == openRects[0]))
				{
					// the 3 flaps have to be ordered counter-clockwise
					if (FlapsAri(core, flap)->rectangles[0]->ID() == FlapsAri(core, i)->rectangles[1]->ID())
						core->NewGestalt(new Cube(core, flap, i, j));	
					if (FlapsAri(core, flap)->rectangles[0]->ID() == FlapsAri(core, j)->rectangles[1]->ID())
						core->NewGestalt(new Cube(core, flap, j, i));	
					
					return true;
				}
			}
		}
  }
  return false;
}

/**
 * @brief Try to close a flap to a cube with one l-junction (or collinearities)
 * and with some additional lines 
 * @param flap ID of new flap
 */
void FormCubes::CloseFlap(unsigned flap)
{ 
//   bool colClosed = false;
// 	bool lClosed = false;
//   Array<unsigned> lines[2];								// lines[LEFT/RIGHT]
//   Array<unsigned> linesNearPoints[2];			// near_points of the lines
//   Array<unsigned> lastLines[2];						// the foregoing line (collinearity)
// 	
//   GetLines(flap, lines, linesNearPoints, lastLines);
// 		
//   lClosed = TryFlapClosing(flap, lines, linesNearPoints, lastLines);
// 
// 	colClosed = TryFlapClosingWithCollinearity(flap, lines, linesNearPoints, lastLines);
// 	
//   if (!lClosed && !colClosed)
// 	cands.PushBack(new CubeCand(flap, lines, linesNearPoints));			// TODO ARI: ist das speichern der lines, near_points sinnvoll? brauch ich die?
}


/**
 * @brief Get all connected lines from the outer junctions of a flap
 * (first as l-junction and then collinearity-lines)
 * @param flap
 * @param lines
 * @param linesNearPoints
 * @param lastLines
 */
void FormCubes::GetLines(unsigned flap, Array<unsigned> *lines, 
	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines)
{
//   // get the lines from the 2 outer junctions (outerJcts[0] and outerJcts[2]) for closing the flap
//   unsigned *outerJcts = FlapsAri(core, flap)->outerJcts;
//   Array</*unsigned*/Line*> jctLines[2];			// [LEFT/RIGHT]
//   Array<unsigned> jctLinesNearPoint[2];			// [LEFT/RIGHT]
// 
//   jctLines[LEFT].PushBack(LJunctions(core, outerJcts[0])->line[LEFT]);
//   jctLinesNearPoint[LEFT].PushBack(LJunctions(core, outerJcts[0])->near_point[LEFT]);
//   jctLines[LEFT].PushBack(LJunctions(core, outerJcts[0])->line[RIGHT]);
//   jctLinesNearPoint[LEFT].PushBack(LJunctions(core, outerJcts[0])->near_point[RIGHT]);
// 
//   jctLines[RIGHT].PushBack(LJunctions(core, outerJcts[2])->line[LEFT]);
//   jctLinesNearPoint[RIGHT].PushBack(LJunctions(core, outerJcts[2])->near_point[LEFT]);
//   jctLines[RIGHT].PushBack(LJunctions(core, outerJcts[2])->line[RIGHT]);
//   jctLinesNearPoint[RIGHT].PushBack(LJunctions(core, outerJcts[2])->near_point[RIGHT]);
// 
//   // get all lines who have a l-junction to one of the jctLines
//   for(int fSide=LEFT; fSide<=RIGHT; fSide++)
//   {
// 		// get every junction-line [LEFT or RIGHT]									// gleiche Linien der jctLines auch verarbeiten? Ja, weil versch. end
//     for (unsigned i=0; i<jctLines[fSide].Size(); i++)
//     {
// 			// get all L-Junctions from the jctLines at jctLinesNearPoint
// 			Array<unsigned> lj;
// // 			Lines(jctLines[fSide][i])->GetLJunctions(jctLinesNearPoint[fSide][i], lj);
// 			jctLines[fSide][i]->GetLJunctions(jctLinesNearPoint[fSide][i], lj);						/// TODO TODO bis hierher bearbeitet
// 
// 	  	// every l-jcts from one of the line									// Überprüfen, ob L-Junction schon dabei war (alreadyCalculated)  
// 			for(unsigned j=0; j<lj.Size(); j++)
// 			{
// 				// get the two lines and nearPoints of the l-junction
// 				unsigned line[2]; 
// 				unsigned nearPoint[2];
// 				line[0] = LJunctions(core, lj[j])->line[0];
// 				line[1] = LJunctions(core, lj[j])->line[1];
// 				nearPoint[0] = LJunctions(core, lj[j])->near_point[0];
// 				nearPoint[1] = LJunctions(core, lj[j])->near_point[1];
// 	
// 				// estimate the other line of the l-junction and push back to lines
// 				if(jctLines[fSide].Contains(line[0]) && !lines[fSide].Contains(line[1]) &&
// 						!Closures(core, Rectangles(core, FlapsAri(core, flap)->rects[0])->clos)->lines.Contains(line[1])  &&
// 						!Closures(core, Rectangles(core, FlapsAri(core, flap)->rects[1])->clos)->lines.Contains(line[1]))
// 				{ 
// 					lines[fSide].PushBack(line[1]);	
// 					linesNearPoints[fSide].PushBack(nearPoint[1]);
// 					lastLines[fSide].PushBack(UNDEF_ID);			
// 					GetNextLines(line[1], Other(nearPoint[1]), lines, linesNearPoints, lastLines, fSide, false);
// 				}
// 				else if (jctLines[fSide].Contains(line[1]) && !lines[fSide].Contains(line[0]) &&
// 					!Closures(core, Rectangles(core, FlapsAri(core, flap)->rects[0])->clos)->lines.Contains(line[0])  &&
// 					!Closures(core, Rectangles(core, FlapsAri(core, flap)->rects[1])->clos)->lines.Contains(line[0]))
// 				{
// 					lines[fSide].PushBack(line[0]);	
// 					linesNearPoints[fSide].PushBack(nearPoint[0]);  
// 					lastLines[fSide].PushBack(UNDEF_ID);		
// 					GetNextLines(line[0], Other(nearPoint[0]), lines, linesNearPoints, lastLines, fSide, false);
// 				}
// 			}
// 		} 
// 
// 		// GetNextLines for all jctLines (Colls of jctLines)	
// 		for(unsigned i=0; i<jctLines[fSide].Size(); i++)
// 		{
// 			GetNextLines(jctLines[fSide][i], jctLinesNearPoint[fSide][i], lines, linesNearPoints, lastLines, fSide, true);
// 		}
//   }
}


/**
 * @brief Get all lines, witch have a collinearity to the line and all following lines.
 * @param line
 * @param lineEnd
 * @param lines
 * @param linesNearPoints
 * @param lastLines
 * @param fSide
 * @param firstLine
 */
void FormCubes::GetNextLines(unsigned line, unsigned lineEnd,					// line END !!!!
	Array<unsigned> *lines, Array<unsigned> *linesNearPoints, 
	Array<unsigned> *lastLines, unsigned fSide, bool firstLine)
{
//   unsigned l, lE;		// new line and lineEnd
// 	
//   // Get all Collinearities on lineEnd
//   Array<unsigned> col = Lines(core, line)->coll[lineEnd];
// 	
//   // get all lines of the found collinearities
//   for(unsigned i=0; i<col.Size(); i++)
//   {
// 		if (Collinearities(core, col[i])->line[0] == line &&																/// TODO: don´t push back if line is line of closure
// 			!lines[fSide].Contains(Collinearities(core, col[i])->line[1]))
// 		{																			// and line is already in lines
// 			lines[fSide].PushBack(Collinearities(core, col[i])->line[1]);
// 			linesNearPoints[fSide].PushBack(Collinearities(core, col[i])->near_point[1]);	
// 			if (firstLine) 
// 				lastLines[fSide].PushBack(UNDEF_ID);
// 			else 
// 				lastLines[fSide].PushBack(Collinearities(core, col[i])->line[0]);			
// 			l = Collinearities(core, col[i])->line[1];
// 			lE = Other(Collinearities(core, col[i])->near_point[1]);
// 			GetNextLines(l, lE, lines, linesNearPoints, lastLines, fSide, false);
// 		}
// 		else if (Collinearities(core, col[i])->line[1] == line &&														/// TODO: don´t push back if line is line of closure
// 			!lines[fSide].Contains(core, Collinearities(col[i])->line[0]))
// 		{
// 			lines[fSide].PushBack(Collinearities(core, col[i])->line[0]);
// 			linesNearPoints[fSide].PushBack(Collinearities(core, col[i])->near_point[0]);
// 			if(firstLine) 
// 				lastLines[fSide].PushBack(UNDEF_ID);
// 			else 
// 				lastLines[fSide].PushBack(Collinearities(core, col[i])->line[1]);			
// 			l = Collinearities(core, col[i])->line[0];
// 			lE = Other(Collinearities(core, col[i])->near_point[0]);
// 			GetNextLines(l, lE, lines, linesNearPoints, lastLines, fSide, false);
// 		}
//   }
}


/**
 * @brief Try to close the flap to get a cube.
 * @param flap
 * @param lines
 * @param linesNearPoint
 * @param lastLines
 */
bool FormCubes::TryFlapClosing(unsigned flap, Array<unsigned> *lines,			// TODO ARI: real near points
	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines)
{
// 	// get every line from left junctions of flap
// 	unsigned fSide = LEFT;			
// 	for(unsigned i=0; i<lines[fSide].Size(); i++)
// 	{
// 	  // get all L-Junctions from line end: Other(near_point)
// 	  Array<unsigned> ljct;
// 	  unsigned end = Other(linesNearPoints[fSide][i]);
// 	  Lines(core, lines[fSide][i])->GetLJunctions(end, ljct);
// 
// 
// 	  // get every l-junction 			
// 	  for(unsigned j=0; j<ljct.Size(); j++)
// 	  {		  
// 			unsigned secondLine;				// number of the second line from the L-Junction
// 			unsigned secondLineSide; 		// LEFT/RIGHT line of L-Junction
// 
// 			if (LJunctions(core, ljct[j])->line[0] == lines[fSide][i])
// 			{
// 				secondLine = LJunctions(core, ljct[j])->line[1];
// 				secondLineSide = 1;
// 			}
// 			else
// 			{
// 				secondLine = LJunctions(core, ljct[j])->line[0];
// 				secondLineSide = 0;
// 			}
// 
// 			// if other side contains the second line and linesNearPoint is equal nearPoint of line from L-Junction
// 			if (lines[Other(fSide)].Contains(secondLine) && linesNearPoints[fSide][i] == LJunctions(ljct[j])->near_point[secondLineSide])
// 			{
// 				unsigned linePos0 = i;																				// position in left lines-array
// 				unsigned linePos1 = lines[Other(fSide)].Find(secondLine);			// position in right lines-array
// 
// 				Array<unsigned> foregoing[2]; 				// find foregoing lines [LEFT/RIGHT]
// 	
// 				while (lastLines[fSide][linePos0] != UNDEF_ID)
// 				{ 
// 					foregoing[fSide].PushBack(lastLines[fSide][linePos0]);
// 					linePos0 = lines[fSide].Find(lastLines[fSide][linePos0]);
// 				} 
// 	
// 				while (lastLines[Other(fSide)][linePos1] != UNDEF_ID)
// 				{
// 					foregoing[Other(fSide)].PushBack(lastLines[Other(fSide)][linePos1]);
// 					linePos1 = lines[Other(fSide)].Find(lastLines[Other(fSide)][linePos1]);
// 				}
// 
// 				core->NewGestalt(new Cube(core, flap, ljct[j], UNDEF_ID, lines[fSide][i], secondLine, foregoing));
// 
// 				return true;
// 			}
// 	  }
// 	}
//   return false;
}


/**
 * @brief Try to close the flap to get a cube with an collinearity at one corner.
 */
bool FormCubes::TryFlapClosingWithCollinearity(unsigned flap, Array<unsigned> *lines,
	Array<unsigned> *linesNearPoints, Array<unsigned> *lastLines)
{
//   // Remember the colls, the j and i and the second lines
//   Array<unsigned> foundColls;		// the found Colls
//   Array<unsigned> js;						// the j
//   Array<unsigned> is;						// the i
//   Array<unsigned> seconds; 			// the second lines
// 
// 	// get every line from left junctions of flap
// 	unsigned fSide = LEFT;	
// 	for(unsigned i=0; i<lines[fSide].Size(); i++)
// 	{
// 	  // get all Collinearities from line end: Other(near_point)
// 	  Array<unsigned> coll;
// 	  unsigned end = Other(linesNearPoints[fSide][i]);
// 	  Lines(core, lines[fSide][i])->GetCollinearities(end, coll);
// 		
// 	  // get every collinearity 			
// 	  for(unsigned j=0; j<coll.Size(); j++)
// 	  {		  
// 			// get 2nd line of the collinearity
// 			unsigned secondLine;
// 			if (Collinearities(core, coll[j])->line[0] == lines[fSide][i])
// 				secondLine = Collinearities(core, coll[j])->line[1];
// 			else
// 				secondLine = Collinearities(core, coll[j])->line[0];
// 			
// 			if (lines[Other(fSide)].Contains(secondLine))
// 			{
// 				// remember all found colls with the second and foregoing lines
// 				foundColls.PushBack(coll[j]);
// 				is.PushBack(i);
// 				js.PushBack(j);
// 				seconds.PushBack(secondLine);
// 			}
// 		}
// 	}
// 
// 	if (foundColls.Size() > 0)
// 	{
// 		/// do not search the smallest angle => bad results at flap-corners
// 		// get coll with the smallest angle
// // 		double ref = 3.142;
// // 		unsigned smallestColl;
// // 		for (unsigned i=0; i<foundColls.Size(); i++)
// // 		{
// // 		  if (Collinearities(foundColls[i])->OpeningAngle() < ref)
// // 		  {
// // 				smallestColl = i;
// // 				ref = Collinearities(foundColls[i])->OpeningAngle();
// // 		  }
// // 		}
// 
// 		// calculate, where the corner point of a cube should be
// 		Vector2 corner = LJunctions(Flaps(flap)->outerJcts[2])->isct +
// 										 LJunctions(Flaps(flap)->outerJcts[0])->isct -
// 										 LJunctions(Flaps(flap)->innerJcts[1])->isct;
// 
// 		// search the collinearity with which is the nearest to the corner-point of the flap
// 		double refVertex = 1000.;
//  		unsigned smallestColl;		for (unsigned i=0; i<foundColls.Size(); i++)
// 		{
// 		  if ((Collinearities(core, foundColls[i])->vertex - corner).Norm() < refVertex)
// 		  {
// 				smallestColl = i;
// 				refVertex = (Collinearities(core, foundColls[i])->vertex - corner).Norm();
// 		  }
// 		}
// 
// 		unsigned linePos0 = is[smallestColl];
// 		unsigned linePos1 = lines[Other(fSide)].Find(seconds[smallestColl]);
// 
// 		Array<unsigned> foregoing[2]; // [LEFT/RIGHT]
// 
// 		while (lastLines[fSide][linePos0] != UNDEF_ID)
// 		{ 
// 		  foregoing[fSide].PushBack(lastLines[fSide][linePos0]);
// 		  linePos0 = lines[fSide].Find(lastLines[fSide][linePos0]);
// 		} 
// 
// 		while (lastLines[Other(fSide)][linePos1] != UNDEF_ID)
// 		{
// 		  foregoing[Other(fSide)].PushBack(lastLines[Other(fSide)][linePos1]);
// 		  linePos1 = lines[Other(fSide)].Find(lastLines[Other(fSide)][linePos1]);
// 		}
// 
// 		core->NewGestalt(new Cube(cube, flap, UNDEF_ID, foundColls[smallestColl], lines[fSide][linePos0], seconds[smallestColl], foregoing));
// 		return true;
// 	}
//   return false;
}


/**
 * @brief Update the cube candidates, if one of the collinearity-lines is part
 * of cube-candidates.
 * @param col ID of new collinearity.
 */
void FormCubes::NewCollinearity(unsigned col)
{
  unsigned line[2]; 
  line[0] = Collinearities(core, col)->line[0]->ID();
  line[1] = Collinearities(core, col)->line[1]->ID();

  for(unsigned i=0; i<cands.Size(); i++)
  {
		if (cands[i]->lines[0].Contains(line[0]) ||
				cands[i]->lines[1].Contains(line[0]) ||
				cands[i]->lines[0].Contains(line[1]) ||
				cands[i]->lines[1].Contains(line[1]))
			UpdateCubeCand(i);
  }
}


/**
 * @brief Try to close the flap, if one of the lines from the L-Junction is 
 * part of an cube candidate
 * @param ljct ID of the new L-Junction
 */
void FormCubes::NewLJunction(unsigned ljct)
{
  unsigned line[2]; 
  line[0] = LJunctions(core, ljct)->line[0]->ID();
  line[1] = LJunctions(core, ljct)->line[1]->ID();
	
  for(unsigned i=0; i<cands.Size(); i++)
  {
	if (cands[i]->lines[0].Contains(line[0]) ||
		cands[i]->lines[1].Contains(line[0]) ||
		cands[i]->lines[0].Contains(line[1]) ||
		cands[i]->lines[1].Contains(line[1]))
	  UpdateCubeCand(i);
  }
}


/**
 * @brief Update cube candidate.
 * @param cand Number of candidate
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


}

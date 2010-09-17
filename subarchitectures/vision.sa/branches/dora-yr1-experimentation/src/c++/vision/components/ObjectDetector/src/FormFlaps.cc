/**
 * @file Flap.cc
 * @author Andreas Richtsfeld
 * @date 2008
 * @version 0.1
 * @brief Class file of Gestalt-principle FormFlaps.
 **/

#include "math.h"
#include "map"
#include "Array.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "Rectangle.hh" 
#include "ExtRectangle.hh"
#include "Flap.hh"
#include "FormFlaps.hh"

namespace Z
{
	
static int CmpFlaps(const void *a, const void *b)
{
  if( Flaps(*(unsigned*)a)->sig > Flaps(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormFlaps::Rank()
{
  RankGestalts(Gestalt::FLAP, CmpFlaps);
}

void FormFlaps::Mask()										
{
  for(unsigned i=0; i<NumFlaps(); i++)
  {
		for(unsigned j=0; j<NumFlaps(); j++)
		{
			if(!Flaps(i)->IsMasked() && !Flaps(j)->IsMasked())
			if(Flaps(i)->sig < Flaps(j)->sig)
				{
				if(Flaps(i)->IsInside(j))
				{	
					Flaps(i)->Mask(j);		 
				}
			}		  
		}
  }
}

bool FormFlaps::NeedsOperate()
{
  return false;
}

FormFlaps::FormFlaps(Config *cfg)
: GestaltPrinciple(cfg)
{
}

void FormFlaps::Operate(bool incremental)
{
}

/**
 * @brief Inform new Gestalt rectangle or extRectangle 
 */
void FormFlaps::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  switch(type)
  {
    case Gestalt::RECTANGLE:
      CreateFlapFromRectangles(idx);		// create flaps from Rectangle
      break;
    case Gestalt::EXTRECTANGLE:
      //CreateFlapFromExtRectangles(idx);		// create flaps from ExtRectangle
      break;
    default:
      break;
  }
  Rank();
  Mask();
  StopRunTime();
}


/**
 * @brief Build flaps from new informed rectangle
 * @param idx Index of new informed rectangle
 */
void FormFlaps::CreateFlapFromRectangles(unsigned idx)
{
  Array<unsigned> newRectLines = Closures(Rectangles(idx)->clos)->lines;

  // search all rectangles
  for (unsigned i=0; i<NumRectangles()-1; i++)
  {
		// get all lines of new and old rectangles
		Array<unsigned> oldRectLines = Closures(Rectangles(i)->clos)->lines;
			for(unsigned j=0;j<newRectLines.Size();j++)
		{
			// get every line from the closure of the old rectangle
			for(unsigned k=0;k<oldRectLines.Size();k++)
			{
				if (newRectLines[j] == oldRectLines[k])
				{	
					// check existing combinations
					if (!IsExistingFlap(idx, i))
					{
						// the two rectangles are superposed?
						bool superposed = RectanglesSuperposed(idx, i);
			
						// get the shared lines
						Array<unsigned> sharedLines;
						if (!superposed) 
						{
							sharedLines = GetSharedLines(idx, i);
				
							unsigned innerJcts[4];
							unsigned outerJcts[4];
							unsigned rect[2];
							rect[0] = idx;
							rect[1] = i;
							double meanGap = MeanGap(rect, innerJcts, outerJcts);
						
							NewGestalt(new Flap(rect[0], rect[1], meanGap, sharedLines, innerJcts, outerJcts));	
						}
					}
				}
			}
		}
  }
}


/*
**	TODO ARI: fertig stellen und bereinigen
**	TODO ARI: Array<unsigned> sharedLines; einbauen!!!
**
**	ARI: build new flaps from extended rectangles
**	
**	1: (neues ExtRectangle->extLines) == (altes ExtRectangle->Rectangle->Closures->Lines)
**	2: (neues ExtRectangle->Rectangle->Closures->Lines) == (altes ExtRectangle->extLines)
**	

void FormFlaps::CreateFlapFromExtRectangles(unsigned idx)
{
  // 1: get every ExtRectangle i 
  for (unsigned i=0; i<NumExtRectangles()-1; i++){
    int nrOfSameLines = 0;
	  
	// get every Line from ExtRectangle30
	int nrOfLines = Closures(Rectangles(ExtRectangles(i)->rect)->clos)->lines.Size();
	for (int j=0; j<nrOfLines; j++){
	  
	  // ExtRectangle->Rectangle->Closure->line
	  unsigned line = Closures(Rectangles(ExtRectangles(i)->rect)->clos)->lines[j];
		
	  // get every extLine of new ExtRectangle
	  int nrOfExtLines = ExtRectangles(idx)->extLines.Size();
	  for (int k=0; k<nrOfExtLines; k++){
	  
		// ExtRectangle->extLines
	  	unsigned extLine = ExtRectangles(idx)->extLines[k];

		// compare lines
		if (line == extLine) nrOfSameLines++; 
	  }
	}

	// make new flap if nrOfSameLines > 1
	if (nrOfSameLines > 1){
		unsigned rect0 = ExtRectangles(idx)->rect;
		unsigned rect1 = ExtRectangles(i)->rect;

		// reject if flap with both rectangles already exists		
		bool exComb = IsExistingFlap(rect0, rect1);		
		// reject if rectangles have shared lines
		bool sharedLines = SharedLines(rect0, rect1);
		// reject if rectangles superposed
//		bool rectanglesSuperposed = RectanglesSuperposed(rect0, rect1);

// ARI: ÜBERLEGEN
		if (!exComb && !sharedLines){
//			NewGestalt(new Flap(rect0, rect1, 2));								// TODO ARI: Type
		}
	}	
  }
	
  // 2: get every ExtRectangle i
  for (unsigned i=0; i<NumExtRectangles()-1; i++){
    int nrOfSameLines = 0;		

	// get every old ExtRectangle
	int nrOfExtLines = ExtRectangles(i)->extLines.Size();
	for (int j=0; j<nrOfExtLines; j++){

	  // get every extLine of old ExtRectangle
	  unsigned extLine = ExtRectangles(i)->extLines[j];
		
	  // get every line from new ExtRectangle
	  int nrOfLines = Closures(Rectangles(ExtRectangles(idx)->rect)->clos)->lines.Size();
	  for (int k=0; k<nrOfLines; k++){

		// ExtRectangle->Rectangle->Closure->lines
		unsigned line = Closures(Rectangles(ExtRectangles(idx)->rect)->clos)->lines[k];

		// compare lines
		if (line == extLine) nrOfSameLines++;
	  }
    }

	// make new Flap if nrOfSameLines > 1
	if (nrOfSameLines > 1){
		unsigned rect0 = ExtRectangles(idx)->rect;
		unsigned rect1 = ExtRectangles(i)->rect;

		// reject if flap with both rectangles already exists		
		bool exComb = IsExistingFlap(rect0, rect1);	
		// reject if rectangles have shared lines
		bool sharedLines = SharedLines(rect0, rect1);
		// reject if rectangles superposed
//		bool rectanglesSuperposed = RectanglesSuperposed(rect0, rect1);

// ARI ÜBERLEGEN
		if (!exComb && !sharedLines){
//			NewGestalt(new Flap(rect0, rect1, 2));								// TODO ARI: Type
		}
	}	
  }
}
*/

/**
 * @brief Return true, if the two rectangles are superposed.
 * @param r0 Rectangle index 1
 * @param r1 Rectangle index 2
 */
bool FormFlaps::RectanglesSuperposed(unsigned r0, unsigned r1)
{
	int equalSense = 0;				// number of lines with same sense
	int nonEqualSense = 0;		// number of lines without same sense
	
	// get lines from closures
	Array<unsigned> closLines0 = Closures(Rectangles(r0)->clos)->lines;
	Array<unsigned> closLines1 = Closures(Rectangles(r1)->clos)->lines;
	
	// compare lines
 	for(unsigned i=0;i<closLines0.Size();i++){
		unsigned l0 = Lines(closLines0[i])->ID();
		
		for(unsigned j=0;j<closLines1.Size();j++){
			unsigned l1 = Lines(closLines1[j])->ID();

			if (l0 == l1){
				// get sign of line
				unsigned sense0 = Closures(Rectangles(r0)->clos)->senses[i];
				unsigned sense1 = Closures(Rectangles(r1)->clos)->senses[j];
				
				// line is equal
				if(sense0 == sense1) equalSense++;
				else nonEqualSense++;
			}
		}	
	}
	
	// return true if rectangle is superposed
	if (equalSense == 0 && nonEqualSense >0) return false;
	else return true;
}

/**
 * @brief Returns all shared lines from two rectangles.
 * @param r0 Rectangle index 1
 * @param r1 Rectangle index 2
 */
Array<unsigned> FormFlaps::GetSharedLines(unsigned r0, unsigned r1)
{
  Array<unsigned> sharedLines;	// shared lines
	
  // get lines from closures
  Array<unsigned> closLines0 = Closures(Rectangles(r0)->clos)->lines;
  Array<unsigned> closLines1 = Closures(Rectangles(r1)->clos)->lines;
	
  // compare lines
  for(unsigned i=0;i<closLines0.Size();i++)
	{
		unsigned l0 = Lines(closLines0[i])->ID();
		
		for(unsigned j=0;j<closLines1.Size();j++){
			unsigned l1 = Lines(closLines1[j])->ID();
	
			if (l0 == l1)
				if(!sharedLines.Contains(l0)) 
					sharedLines.PushBack(l0);
		}
  }
  return sharedLines;
}

/*
**
**	Find shared lines in ExtRectangles
**	(for neighboring rectangles)
**

bool FormFlaps::SharedLines(unsigned r0, unsigned r1)
{
  int sharedLines = 0;

  // get every line from ExtRectangle r0
  unsigned nrOfLinesR0 = Closures(Rectangles(ExtRectangles(r0)->rect)->clos)->lines.Size();
  for(unsigned i=0; i<nrOfLinesR0; i++)
  {
	unsigned line0 = Closures(Rectangles(ExtRectangles(r0)->rect)->clos)->lines[i];
	  
	// get every line from ExtRectangle r1
	unsigned nrOfLinesR1 = Closures(Rectangles(ExtRectangles(r1)->rect)->clos)->lines.Size();
    for(unsigned j=0; j<nrOfLinesR1; j++)
    {
	  unsigned line1 = Closures(Rectangles(ExtRectangles(r1)->rect)->clos)->lines[j];

	  // compare line1 and line2
	  if(line0 == line1) sharedLines++;
    }
  }

  // shared lines found?
  if (sharedLines > 0) return true;
  else return false;
}
*/


/**
 * @brief Checks if the combination of rects exists already as flap.
 * @param r0 Rectangle index 0
 * @param r1 Rectangle index 1
 */
bool FormFlaps::IsExistingFlap(unsigned r0, unsigned r1){
  
  bool isExistingFlap = false;

  // get rectangles of every existing flap
  for (unsigned i=0; i<NumFlaps(); i++){
    unsigned rect0 = Flaps(i)->rects[0];
	unsigned rect1 = Flaps(i)->rects[1];
	  
	// compare the two rectangle-pairs
	if ((rect0==r0 && rect1==r1) || (rect0==r1 && rect1==r0)) 
		isExistingFlap = true;
  }
  
  if (isExistingFlap) return true;
  else return false;
}

/**
 * @brief Calculate the mean gap of the two best superposed corners.
 * @param rect 
 *	
 */
double FormFlaps::MeanGap(unsigned *rect, unsigned *innerJcts, unsigned *outerJcts)
{
  unsigned cornersR0[4];		// corners of rectangle r0
  unsigned cornersR1[4];		// corners of rectangle r1
  Vector2 isctR0[4];			// intersection points from corners of r0
  Vector2 isctR1[4];			// intersection points from corners of r1
	
  double dist[4][4];			// calculated distance from corner of r0
								// to each corner of r1

  double minDistance[2];		// the 2 minimum corner gaps
  minDistance[0] = 2000.;		// start values of 2000 mm						// TODO ARI: Bildrand!
  minDistance[1] = 2000.;

  // get the 4 corners and intersections of each rectangle
  for (int i=0; i<4; i++)
  {
		cornersR0[i] = Rectangles(rect[0])->jcts[i];
		cornersR1[i] = Rectangles(rect[1])->jcts[i];
		isctR0[i] = LJunctions(Rectangles(rect[0])->jcts[i])->isct; 
		isctR1[i] = LJunctions(Rectangles(rect[1])->jcts[i])->isct; 
  }
  
  // find the nearest corner of r1 to each corner of r0
  // each corner of r0
  for (int i=0; i<4; i++)
  {
	// each corner of r1
	for (int j=0; j<4; j++)
	{		
	  // calculate distance to each corner of r1
	  double sqrX = Sqr(isctR0[i].x-isctR1[j].x);
	  double sqrY = Sqr(isctR0[i].y-isctR1[j].y);
	  dist[i][j] = sqrt(sqrX + sqrY);
	}
  }
  
  // find the smallest gap from dist[4][4] without i=j
  int saveI = UNDEF_ID;
  int saveJ = UNDEF_ID;
  for (int i=0; i<4; i++)
  {
		for (int j=0; j<4; j++)
		{
			if (dist[i][j] < minDistance[0])
			{
				minDistance[0] = dist[i][j];
				saveI = i;
				saveJ = j;
			}
		}
  }

  // smallest gap => saveI, saveJ
  innerJcts[0] = cornersR0[saveI];
  innerJcts[1] = cornersR1[saveJ];
  
  // overwrite the found row and column from the dist-array
  for (int i=0; i<4; i++)
  {
		dist[i][saveJ] = 2000.;
		dist[saveI][i] = 2000.;
  }
  
  // find the 2nd smallest gap from dist[4][4] without i=j
  saveI = UNDEF_ID;
  saveJ = UNDEF_ID;
  for (int i=0; i<4; i++)
  {
		for (int j=0; j<4; j++)
		{
			if (dist[i][j] < minDistance[1])
			{
				minDistance[1] = dist[i][j];
				saveI = i;
				saveJ = j;
			}
		}
  }

  // 2nd smallest gap
  innerJcts[2] = cornersR0[saveI];
  innerJcts[3] = cornersR1[saveJ];
  
  // get outer junctions of R0
  int nrFound = 0;
  for (int i=0; i<4; i++)
  {
		bool outerFound = true;
		for (int j=0; j<4; j++)
			if (cornersR0[i] == innerJcts[j])
				outerFound = false;
		if (outerFound)
		{
			outerJcts[nrFound] = cornersR0[i];
			nrFound++;
		}
  }
  
  for (int i=0; i<4; i++)
  {
		bool outerFound = true;
		for (int j=0; j<4; j++)
			if (cornersR1[i] == innerJcts[j])
				outerFound = false;
		if (outerFound)
		{
			outerJcts[nrFound] = cornersR1[i];
			nrFound++;
		}
  }
 
  // order (inner and) outer junctions
  OrderJunctions(rect, innerJcts, outerJcts);
  
  // calculate the mean value of the two smallest gaps
  double meanGap = (minDistance[0] + minDistance[1])/2;
  return meanGap;
}


/**
 * @brief Order Junctions of the flap:
 *	Orders the inner and outer junctions clockwise for r0 and counterclockwise for r1
 *
 *	If the angle between left and right rectangle is greater than PI, than
 *	change the junctions (bottom/top) of the flap and the rects.
 */
void FormFlaps::OrderJunctions(unsigned *rect, unsigned *innerJcts, unsigned *outerJcts)
{
  unsigned first, second;

  // order outer junctions of r0 (outerJunctions[0/1])
  for(unsigned i=0; i<4; i++)
  {
		if (outerJcts[0] == Rectangles(rect[0])->jcts[i])
			first = i;
		if (outerJcts[1] == Rectangles(rect[0])->jcts[i])
			second = i;
  }
  if ((second-first) != 1)
  {
		unsigned store = outerJcts[0];
		outerJcts[0] = outerJcts[1];
		outerJcts[1] = store;
  }	  

  // order outer junctions of r1 (outerJunctions[2/3])  
  for(unsigned i=0; i<4; i++)
  {
		if (outerJcts[2] == Rectangles(rect[1])->jcts[i])
	  	first = i;
		if (outerJcts[3] == Rectangles(rect[1])->jcts[i])
	  	second = i;
  }
  if ((first-second)!=1 && (second-first)!=3) 
  {
		unsigned store = outerJcts[2];
		outerJcts[2] = outerJcts[3];
		outerJcts[3] = store;
  }	  
    
  // order the inner junctions
  // junctions of r0 in clockwise order
  unsigned jctsR0[4];
  for(unsigned i=0; i<4; i++)
	jctsR0[i] = Rectangles(rect[0])->jcts[i];
  
  unsigned next=0; 
  unsigned i=0;
  while (jctsR0[i] != outerJcts[1]) i++;
  next = i+1;
  if (next > 3) next = 0;
  innerJcts[0] = jctsR0[next];
  next++;
  if (next > 3) next = 0;
  innerJcts[1] = jctsR0[next];


  // junctions of r1 in counterclockwise order
  unsigned jctsR1[4];
  for(unsigned i=0; i<4; i++)
	jctsR1[i] = Rectangles(rect[1])->jcts[i];
  
  next=0; i=0;
  while (jctsR1[i] != outerJcts[3]) i++;
  next = i;
  if (next == 0) next = 3;
  else next--;					  // be carefull => unsigned int
  innerJcts[2] = jctsR1[next];
  if (next == 0) next = 4;
  next--;
  innerJcts[3] = jctsR1[next];
  
  
  // Check whether the angle between left and right rectangle at the top is
  // smaller or greater than PI.
  																				// TODO ARI: Check also the cross-product of the 
  																				// bottom side!
  // calculate the direction of r0 and r1
  Vector2 dirR0 = LJunctions(innerJcts[1])->isct - LJunctions(outerJcts[0])->isct;
  Vector2 dirR1 = LJunctions(innerJcts[3])->isct - LJunctions(outerJcts[2])->isct;
  
  // if cross-product of directions is negativ change top and bottum
  if (Cross(dirR0, dirR1) > 0)
  {
		unsigned save0 = outerJcts[0];
		unsigned save1 = outerJcts[1];

		outerJcts[0] = outerJcts[3];
		outerJcts[1] = outerJcts[2];
		outerJcts[2] = save1; 
		outerJcts[3] = save0;  
			
		save0 = innerJcts[0];
		save1 = innerJcts[1];
		innerJcts[0] = innerJcts[3];
		innerJcts[1] = innerJcts[2];
		innerJcts[2] = save1; 
		innerJcts[3] = save0;  
			
		save0 = rect[0];
		rect[0] = rect[1];
		rect[1] = save0;
  }
}

}

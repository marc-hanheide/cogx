/**
 * $Id: FormExtRectangles.cc,v 1.0 2007/10/23 17:27:03 mxz Exp mxz $
 */

#include "Array.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "Rectangle.hh"
#include "ExtRectangle.hh" 
#include "FormExtRectangles.hh"

namespace Z
{
	
static int CmpExtRectangles(const void *a, const void *b)
{
  if( ExtRectangles(*(unsigned*)a)->sig > ExtRectangles(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormExtRectangles::Rank()
{
  RankGestalts(Gestalt::EXTRECTANGLE, CmpExtRectangles);
}

bool FormExtRectangles::NeedsOperate()
{
  return false;
}

FormExtRectangles::FormExtRectangles(Config *cfg)
: GestaltPrinciple(cfg)
{
}

void FormExtRectangles::Operate(bool incremental)
{
}

/*
*	InformNewGestalt receives new rectangles
*/
void FormExtRectangles::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();
  Create(type, idx);
//	Rank();
  StopRunTime();
}

/*
**	TODO ARI: fertig stellen und bereinigen
**	create new extended rectangles (ExtRectangle)
*/
void FormExtRectangles::Create(Gestalt::Type type, unsigned idx)
{
  unsigned rectJcts[4];			// junctions of rectangle
  unsigned rectJctsLines[8];	// lines of junctions of rectangle
  Array<unsigned> extJcts;		// extended Junctions
  Array<unsigned> extLines;		// extended Lines
  Array<unsigned> extColls;		// extended Collinearities

  // get Rectangle->Junctions->Lines  ==  4 L-Jcts - 8 Lines
  for (int i=0; i<4; i++) {
	rectJcts[i] = Rectangles(idx)->jcts[i];
	rectJctsLines[i*2] = LJunctions(rectJcts[i])->line[0];
	rectJctsLines[i*2+1] = LJunctions(rectJcts[i])->line[1];
  }

  // get all L-Jcts and Colls for all 8 lines
  for (int i=0; i<8; i++) {
	  
	// seperate out the doubled lines
	bool alreadyCalculated = false;
	for (int j=0; j<i; j++){
	  if (rectJctsLines[i]==rectJctsLines[j]) alreadyCalculated = true;
	}
	  
	if (!alreadyCalculated) {
	  // check all existing L-Junctions
      for (unsigned j=0; j<NumLJunctions(); j++) {

		// is left line of L-Junctions a line from junctions of rect
		if (LJunctions(j)->line[LEFT] == rectJctsLines[i]) {
		  // reject if junction is a vertex of the rectangle
		  bool isRectJct = false;
		  for (int k=0; k<4; k++){
		    if (j == rectJcts[k]) isRectJct = true;	  
		  }
		  
		  // reject if line is from Closure
		  bool isClosLine = false;
		  unsigned nrOfClosLines = Closures(Rectangles(idx)->clos)->lines.Size();
		  for (unsigned l=0; l<nrOfClosLines; l++){
			unsigned line = Closures(Rectangles(idx)->clos)->lines[l];
			if (LJunctions(j)->line[RIGHT] == line && line != UNDEF_ID) 
			  isClosLine = true;
		  }
			
		  // junction is no vertex && 2nd line isn´t already part of the closure 
		  if (!isRectJct && !isClosLine){		
			// only if jct isn´t already pushed
			if (!extJcts.Contains(j)){	
			  extJcts.PushBack(j);
			}
			// only if line is´nt already pushed
			if (!extLines.Contains(LJunctions(j)->line[RIGHT])){
			  extLines.PushBack(LJunctions(j)->line[RIGHT]);
			}
		  }			
	    }
		
		// is right line of L-Junctions a line from junctions of rect
		if (LJunctions(j)->line[RIGHT] == rectJctsLines[i]) {
		  // reject if junction is a vertex of the rectangle
		  bool isRectJct = false;
		  for (int k=0; k<4; k++){
		    if (j == rectJcts[k]) isRectJct = true;	  
		  }
		  
		  // reject if line is from Closure
		  bool isClosLine = false;
		  unsigned nrOfClosLines = Closures(Rectangles(idx)->clos)->lines.Size();
		  for (unsigned l=0; l<nrOfClosLines; l++){
			unsigned line = Closures(Rectangles(idx)->clos)->lines[l];
			if (LJunctions(j)->line[LEFT] == line && line != UNDEF_ID)
			  isClosLine = true;
		  }

		  // junction is no vertex && 2nd line isn´t already part of the closure 
		  if (!isRectJct && !isClosLine){		
			// only if jct isn´t already pushed
			if (!extJcts.Contains(j)){	
			  extJcts.PushBack(j);
			}
			// only if line is´nt already pushed
			if (!extLines.Contains(LJunctions(j)->line[LEFT])){
			  extLines.PushBack(LJunctions(j)->line[LEFT]);
			}
		  }			
	    }		
	  }

	  // check all existing collinearities
      for (unsigned j=0; j<NumCollinearities(); j++) {
		int nrOfColls = Closures(Rectangles(idx)->clos)->colls.Size();

		// check left line of the collinearity
		if (Collinearities(j)->line[LEFT] == rectJctsLines[i]) {
		  // reject if collinearity is a collinearity of the closure
		  bool isRectCol = false;
		  for (int k=0; k<nrOfColls; k++){
			unsigned coll = Closures(Rectangles(idx)->clos)->colls[k];
		    if (j == coll && coll != UNDEF_ID) isRectCol = true;	  
		  }
		  
		  if (!isRectCol){
			// only if right line from coll isn´t part of the closure
		 	if (!Closures(Rectangles(idx)->clos)->lines.Contains(Collinearities(j)->line[RIGHT])){
			  // only if coll isn´t already pushed
		      if (!extColls.Contains(j)){
		    	extColls.PushBack(j);
			  }
			  // only if line isn´t already pushed
			  if (!extLines.Contains(Collinearities(j)->line[RIGHT])){
		   		extLines.PushBack(Collinearities(j)->line[RIGHT]);
			  }
			}
   		  }
		}
		
		// check right line of the collinearity
		if (Collinearities(j)->line[RIGHT] == rectJctsLines[i]) {
		  // reject if collinearity is a collinearity of the closure
		  bool isRectCol = false;
		  for (int k=0; k<nrOfColls; k++){
			unsigned coll = Closures(Rectangles(idx)->clos)->colls[k];
			if (j == coll && coll != UNDEF_ID) isRectCol = true;
		  }
		   
		  if (!isRectCol){
			// only if other line from coll isn´t part of the closure
		 	if (!Closures(Rectangles(idx)->clos)->lines.Contains(Collinearities(j)->line[LEFT])){
			  // only if coll isn´t already pushed
			  if (!extColls.Contains(j)){
		        extColls.PushBack(j);
			  }
			  // only if line isn´t already pushed
			  if (!extLines.Contains(Collinearities(j)->line[LEFT])){
		        extLines.PushBack(Collinearities(j)->line[LEFT]);
			  }
			}
		  }
		}
      }
    }
  } 

  // TODO ARI: alle Junctions und lines im Array bekannt
  		// haben die Lines weiter Fortsetzungen über Colls?
  		// sollte man diese weiter verfolgen => hat das Sinn
  
  // create new ExtRectangle, if one L-Jct or Coll found
  if (extJcts.Size() >= 1 || extColls.Size() >= 1){
	NewGestalt(new ExtRectangle(idx, extJcts, extLines, extColls));
  }
}

}

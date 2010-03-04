/**
 * @file FormFlapsAri.cc
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Class file of Gestalt-principle FormFlapsAri: Form flaps from rectangles, instead of closures.
 **/

#include "math.h"
#include "map"
#include "Array.hh"
#include "Line.hh"
#include "LJunction.hh"
#include "Collinearity.hh"
#include "Closure.hh"
#include "Rectangle.hh" 
#include "FlapAri.hh"
#include "FormFlapsAri.hh"

namespace Z
{
	
/**
 * @brief Compare function for flaps.
 */
static int CmpFlaps(const void *a, const void *b)
{
  if( (*(FlapAri**)a)->sig > (*(FlapAri**)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}


/**
 * @brief Rank flaps.
 */
void FormFlapsAri::Rank()
{
  RankGestalts(Gestalt::FLAP_ARI, CmpFlaps);
}


/**
 * @brief Mask flaps, if it is above another flap with higher significance.
 * Center points and radius are used to estimate if they are on top of each other.
 */
void FormFlapsAri::Mask()										
{
  for(unsigned i=0; i<NumFlapsAri(core); i++)
		for(unsigned j=0; j<NumFlapsAri(core); j++)
			if(!FlapsAri(core, i)->IsMasked() && !FlapsAri(core, j)->IsMasked())
				if(FlapsAri(core, i)->sig < FlapsAri(core, j)->sig)
					if(FlapsAri(core, i)->IsInside(j))
						FlapsAri(core, i)->Mask(j);		 
}

/**
 * @brief NeedsOperate
 */
bool FormFlapsAri::NeedsOperate()
{
  return false;
}


/**
 * @brief Constructor of FormFlapsAri
 * @param core Vision core
 */
FormFlapsAri::FormFlapsAri(VisionCore *core) : GestaltPrinciple(core)
{
}

/**
 * @brief Operate
 * @param incremental Operate incremental or non-incremental.
 */
void FormFlapsAri::Operate(bool incremental)
{
}

/**
 * @brief Inform new Gestalt rectangle.
 * @param type Gestalt type
 * @param idx Gestalt index
 */
void FormFlapsAri::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
//   StartRunTime();																																				/// TODO Wieder implementieren!
  switch(type)
  {
    case Gestalt::RECTANGLE:
      CreateFlapFromRectangles(idx);		// create flaps from Rectangle
      break;
    default:
			printf("FormFlaps::InformNewGestalt: unknown Gestalt type.\n");
      break;
  }
  Rank();
  Mask();
//   StopRunTime();
}


/**
 * @brief Build flaps from new informed rectangle.
 * Search line-array of rectangles for shared lines
 * @param idx Index of new informed rectangle
 */
void FormFlapsAri::CreateFlapFromRectangles(unsigned idx)
{
  Array<Line*> newRectLines = Rectangles(core, idx)->lines;

  // search all rectangles, except new one
  for (unsigned i=0; i<NumRectangles(core)-1; i++)
  {
		bool alreadyFound = false;

		// get all lines of old rectangles
		Array<Line*> oldRectLines = Rectangles(core, i)->lines;
		for(unsigned j=0;j<newRectLines.Size();j++)
		{
			// get every line from the closure of the old rectangle
			for(unsigned k=0;k<oldRectLines.Size();k++)
			{
				// Build new flap when shared line found an if not already a combnation found.
				if (newRectLines[j]->ID() == oldRectLines[k]->ID() && !alreadyFound)
				{

					// check existing combinations
					if (!IsExistingFlap(idx, i))																	/// TODO Sollte man jetzt nicht mehr brauchen!!! (Kommt die Meldung, dass gefunden? => Sonst löschen!)
					{
						// check if the two rectangles are superposed
						bool superposed = RectanglesSuperposed(idx, i);
	
						Array<unsigned> sharedLines;																									/// TODO Shared lines sollten als Array<Line*> weitergegeben werden
						if (!superposed) 
						{
							sharedLines = GetSharedLines(idx, i);

							// TODO old meanGap function => alles das hier weg (mit Unterfunktionen!)
							unsigned innerJcts[4];
							unsigned outerJcts[4];
							unsigned rect[2];
							rect[0] = idx;
							rect[1] = i;
							double meanGap = MeanGap(rect, innerJcts, outerJcts);												/// TODO Gibt rects weiter und bekommt sortierte inner und outerJunctions

							// TODO new meanGap function
							Vector2 orderedIsctR0[4];
							Vector2 orderedIsctR1[4];
							double meanGap2 = MeanGap(rect, orderedIsctR0, orderedIsctR1);

							alreadyFound = true;
							core->NewGestalt(new FlapAri(core, rect[0], rect[1], meanGap2, sharedLines, innerJcts, outerJcts, orderedIsctR0, orderedIsctR1));			/// TODO inner, outerJcts weg!
						}
					}
				}
			}
		}
  }
}



/**
 * @brief Return true, if the two rectangles are superposed. \n
 * Use the lines of the underlying closures to check if they are superposed.
 * @param r0 Rectangle index 1
 * @param r1 Rectangle index 2
 */
bool FormFlapsAri::RectanglesSuperposed(unsigned r0, unsigned r1)
{
	int equalSense = 0;				// number of lines with same sense
	int nonEqualSense = 0;		// number of lines without same sense
	
	// get lines from closures
	Array<Line*> closLines0 = Closures(core, Rectangles(core, r0)->closure->ID())->lines;
	Array<Line*> closLines1 = Closures(core, Rectangles(core, r1)->closure->ID())->lines;
	
	// compare lines
 	for(unsigned i=0;i<closLines0.Size();i++)
	{
		unsigned l0 = closLines0[i]->ID();
		
		for(unsigned j=0;j<closLines1.Size();j++){
			unsigned l1 = closLines1[j]->ID();

			if (l0 == l1){
				// get sign of line
				unsigned sense0 = Closures(core, Rectangles(core, r0)->closure->ID())->senses[i];
				unsigned sense1 = Closures(core, Rectangles(core, r1)->closure->ID())->senses[j];
				
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


/**																																					/// TODO Ändern auf return: Array<Line*>
 * @brief Returns all shared lines from two rectangles.
 * @param r0 Rectangle index 1
 * @param r1 Rectangle index 2
 */
Array<unsigned> FormFlapsAri::GetSharedLines(unsigned r0, unsigned r1)
{
  Array<unsigned> sharedLines;	// shared lines
	
  // get lines from rectangles
  Array<Line*> closLines0 = Rectangles(core, r0)->lines;
  Array<Line*> closLines1 = Rectangles(core, r1)->lines;
	
  // compare lines
  for(unsigned i=0;i<closLines0.Size();i++)
	{
		for(unsigned j=0;j<closLines1.Size();j++)
		{
			if (closLines0[i]->ID() == closLines1[j]->ID())
				if(!sharedLines.Contains(closLines0[i]->ID())) 
					sharedLines.PushBack(closLines0[i]->ID());
		}
  }
  return sharedLines;
}


/**																																						/// TODO Diese Funktion kann nicht mehr verwendet werden, da auf L-Junctions beruht!
 * @brief Checks if the combination of rects exists already as flap.
 * @param r0 Rectangle index 0
 * @param r1 Rectangle index 1
 */
bool FormFlapsAri::IsExistingFlap(unsigned r0, unsigned r1)
{
  bool isExistingFlap = false;

  // get rectangles of every existing flap
  for (unsigned i=0; i<NumFlapsAri(core); i++)
	{
    unsigned rect0 = FlapsAri(core, i)->rectangles[0]->ID();
		unsigned rect1 = FlapsAri(core, i)->rectangles[1]->ID();

		// compare the two rectangle-pairs
		if ((rect0==r0 && rect1==r1) || (rect0==r1 && rect1==r0)) 
			isExistingFlap = true;
  }
  
if (isExistingFlap) printf("FormFlapsAri::IsExistingFlap: Das kann hier eigentlich gar nicht auftreten: Bitte sofort diese Funktion löschen!\n");

  if (isExistingFlap) return true;
  else return false;
}



/**																																															/// TODO NEUE MeanGap-Funktion!
 * @brief Calculate the mean gap between the best combination of the intersections.
 * We know that the ractangles are sharing at least one line. We know the intersection points \n
 * at the corners of the rectangle. Now we are searching for the best combination between
 * these points. We order them counter clockwise, beginning with the first two intersection
 * points to the other rectangle.
 * @param rect The two rectangle ids
 * @param orderedIsctR0 First, second intersection point with R1, then => counter clockwise
 * @param orderedIsctR1 First, second intersection point with R0, then => counter clockwise
 * The assigned intersections are now: orderedIsctR0[0]-orderedIsctR0[1] and orderedIsctR0[1]-orderedIsctR0[0]
 */
double FormFlapsAri::MeanGap(unsigned *rect, Vector2 *orderedIsctR0, Vector2 *orderedIsctR1)
{
	double meanGap;
  Vector2 isctR0[4];					// intersection points of first rectangele
  Vector2 isctR1[4];					// intersection points of second rectangle
  double dist[4][4];					// calculated distance from corner of r0 to each corner of r1

  // get the 4 intersections of each rectangle
  for (int i=0; i<4; i++)
  {
		isctR0[i] = Rectangles(core, rect[0])->isct[i];
		isctR1[i] = Rectangles(core, rect[1])->isct[i];
  }

  // calculate distance between all intersections
  for (int i=0; i<4; i++)
		for (int j=0; j<4; j++)
			dist[i][j] = Distance(isctR0[i], isctR1[j]);


// printf("Distances of Rectangles: %u-%u\n", rect[0], rect[1]);
// for (int j=0; j<4; j++)
// {		
// // 	printf("  %4.2f - %4.2f - %4.2f - %4.2f\n", dist[0][j], dist[1][j], dist[2][j], dist[3][j]);
// }

  // find the smallest gap from dist[4][4]
  int saveI = UNDEF_ID;
  int saveJ = UNDEF_ID;
  double minDistance = HUGE;
  for (int i=0; i<4; i++)
  {
		for (int j=0; j<4; j++)
		{
			if (dist[i][j] < minDistance)
			{
				minDistance = dist[i][j];
				saveI = i;
				saveJ = j;
			}
		}
  }

	// find the second intersection between the rectangles:
	// 2 possibilities: dist[i-1][j+1] or dist[i+1][j-1]
// printf("Smallest: %u-%u\n", saveI, saveJ);

	double secDist[2];
	int save2I = saveI - 1;
	int save2J = saveJ + 1;
	if(save2I < 0) save2I = 3;
	if(save2J > 3) save2J = 0;
// printf("=> 2nd: %u-%u\n", save2I, save2J);
	secDist[0] = dist[save2I][save2J];
	save2I = saveI + 1;
	save2J = saveJ - 1;
	if(save2I > 3) save2I = 0;
	if(save2J < 0) save2J = 3;
// printf("=> 2nd: %u-%u\n", save2I, save2J);
	secDist[1] = dist[save2I][save2J];


	// change start point of one rectangle
	if(secDist[1] < secDist[0]) 
		saveJ = save2J;
	else
		saveI = save2I;
// printf("Save: %u- %u\n", saveI, saveJ);

	for(unsigned i = 0; i<4; i++)
	{
		saveI++;
		saveJ++;
		if(saveI > 3) saveI = 0;
		if(saveJ > 3) saveJ = 0;
// printf("    => I, J: %u-%u\n", saveI, saveJ);
		orderedIsctR0[i] = isctR0[saveI];
		orderedIsctR1[i] = isctR1[saveJ];
	}
// printf("    => second: %4.2f - %4.2f\n", secDist[0], secDist[1]);

	meanGap = ((Distance(orderedIsctR0[0], orderedIsctR1[1]) + Distance(orderedIsctR0[1], orderedIsctR1[0]))/2.);
	return meanGap;
}


/**
 * @brief Calculate the mean gap of the two best superposed corners.
 * We know that the ractangles are sharing at least one line.
 * @param rect The two rectangle ids
 * @param innerJcts The four inner junctions of the flap.
 * @param outerJcts The four outer junctions of the flap.
 */
double FormFlapsAri::MeanGap(unsigned *rect, unsigned *innerJcts, unsigned *outerJcts)
{
  unsigned cornersR0[4];		// corners of rectangle r0
  unsigned cornersR1[4];		// corners of rectangle r1
  Vector2 isctR0[4];				// intersection points from corners of r0
  Vector2 isctR1[4];				// intersection points from corners of r1
	
  double dist[4][4];				// calculated distance from corner of r0 to each corner of r1

  double minDistance[2];		// the 2 minimum corner gaps
  minDistance[0] = 2000.;		// start values of 2000px																					// TODO ARI: Bildrand!
  minDistance[1] = 2000.;

  // get the 4 corners and intersections of each rectangle
  for (int i=0; i<4; i++)
  {
		cornersR0[i] = Rectangles(core, rect[0])->ljcts[i]->ID();																	/// TODO Hier wird mit L-Junctions gearbeitet: Die sollte
		cornersR1[i] = Rectangles(core, rect[1])->ljcts[i]->ID();																	/// es bei Rectangles nicht geben!	
		isctR0[i] = LJunctions(core, Rectangles(core, rect[0])->ljcts[i]->ID())->isct; 
		isctR1[i] = LJunctions(core, Rectangles(core, rect[1])->ljcts[i]->ID())->isct; 
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

  // find the smallest gap from dist[4][4]
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
  
  // find the 2nd smallest gap from dist[4][4]
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
  SortJunctions(rect, innerJcts, outerJcts);																		/// TODO Sort junctions macht mit Intersections keinen Sinn mehr!
  
  // calculate the mean value of the two smallest gaps
  double meanGap = (minDistance[0] + minDistance[1])/2;
  return meanGap;
}


/**
 * @brief Sort Junctions of the flap:
 * Sorts the inner and outer junctions clockwise for r0 and counterclockwise for r1
 *
 * If the angle between left and right rectangle is greater than PI, than
 * change the junctions (bottom/top) of the flap and the rects.
 */
void FormFlapsAri::SortJunctions(unsigned *rect, unsigned *innerJcts, unsigned *outerJcts)
{
  unsigned first, second;

  // order outer junctions of r0 (outerJunctions[0/1])
  for(unsigned i=0; i<4; i++)
  {
		if (outerJcts[0] == Rectangles(core, rect[0])->ljcts[i]->ID())
			first = i;
		if (outerJcts[1] == Rectangles(core, rect[0])->ljcts[i]->ID())
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
		if (outerJcts[2] == Rectangles(core, rect[1])->ljcts[i]->ID())
	  	first = i;
		if (outerJcts[3] == Rectangles(core, rect[1])->ljcts[i]->ID())
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
	jctsR0[i] = Rectangles(core, rect[0])->ljcts[i]->ID();
  
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
	jctsR1[i] = Rectangles(core, rect[1])->ljcts[i]->ID();
  
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
  Vector2 dirR0 = LJunctions(core, innerJcts[1])->isct - LJunctions(core, outerJcts[0])->isct;
  Vector2 dirR1 = LJunctions(core, innerJcts[3])->isct - LJunctions(core, outerJcts[2])->isct;
  
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

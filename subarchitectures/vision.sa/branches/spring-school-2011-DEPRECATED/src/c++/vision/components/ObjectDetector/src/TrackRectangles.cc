/**
 * @file TrackRectangles.cc
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief Tracking rectangles
 **/

#include "TrackRectangles.hh"
#include "MotionField.hh"
#include "TrktRectangle.hh"
#include "Rectangle.hh"
#include "LJunction.hh"
#include "Math.hh"

namespace Z
{

/**
 * @brief Constructor of class TLJct 
 */
TrackRectangles::TRectangle::TRectangle(unsigned i, RectDef rd)
{
  id = i;
  age = 0;

	rectDef = rd;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void TrackRectangles::TRectangle::FormerTracked(unsigned id)
{
  id_former_tracked = id;
}

void TrackRectangles::TRectangle::NextTracked(unsigned id)
{
	id_next_tracked = id;
}

// ************************************************************************************ //
// Age the Objects
// ************************************************************************************ //

/**
 *	@brief Increase age of all objects (step counter) and erase if one is older than maxAge
 *	@param maxAge Maximum of age befor object will be deleted
 */
void TrackRectangles::AgeTObjects(unsigned maxAge)
{
  AgeTRectangles(maxAge);
}

/**
 * @brief Increment age of tCubes and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void TrackRectangles::AgeTRectangles(unsigned maxAge)
{
  for(unsigned i=tRects.Size(); i>0; i--)
  {
     tRects[i-1]->age++;
     if (tRects[i-1]->age > maxAge) tRects.Erase(i-1);
  }
}

// ************************************************************************************ //
// Track Rectangles
// ************************************************************************************ //

static int CmpTrktRectangles(const void *a, const void *b)
{
  if( TrktRectangles(*(unsigned*)a)->sig > TrktRectangles(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void TrackRectangles::Rank()
{
  RankGestalts(Gestalt::TRKT_RECTANGLE, CmpTrktRectangles);
}

/**
 * @brief Masking Tracked Rectangles
 */
void TrackRectangles::Mask()
{
}

bool TrackRectangles::NeedsOperate()
{ 
  return false;	/// TODO
}

void TrackRectangles::Reset(const Image *img)
{
	AgeTObjects(maxAge);
}

TrackRectangles::TrackRectangles(Config *cfg) : GestaltPrinciple(cfg)
{
	firstCall = true;
}

/**
 * @brief InformNewGestalt()
 */
void TrackRectangles::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();

  if (type == Gestalt::RECTANGLE)
	{
// 		SaveNewRectangle(idx);																			/// TODO incremental saving of tracking candidates
//		if(!firstCall) TrackRectangleIncremental(idx);							/// TODO incremental call for tracking
	}

  StopRunTime();
}

/**
 * @brief Operate()
 */
void TrackRectangles::Operate(bool incremental)
{
//   StartRunTime();

// 	cout << "TrackRectangles::Operate" << endl;

//   StopRunTime();
}

/**
 * @brief OperateNonIncremental()
 */
void TrackRectangles::OperateNonIncremental()
{
  StartRunTime();

	SaveNewRectangleNonIncremental();
	if(!firstCall)
		TrackRectangleNonIncremental();
	
	Rank();
	Mask();

	firstCall = false;
	StopRunTime();
}

/**
 * @brief Save new rectangles as tracking candidate in tRects
 * @param idx Index of new received Gestalt
 */
void TrackRectangles::SaveNewRectangle(unsigned idx)
{
	RectDef rd;

	for(unsigned i=0; i<4; i++)
	{
		rd.jcts[i] = Rectangles(idx)->jcts[i];
		rd.jctsIsct[i] = LJunctions(Rectangles(idx)->jcts[i])->isct;
	}
	rd.direction[0] = Rectangles(idx)->direction[0];
	rd.direction[1] = Rectangles(idx)->direction[1];
	rd.centerPoint = Rectangles(idx)->centerPoint;

	tRects.PushBack(new TRectangle(idx, rd));
}

/**
 * @brief Save new rectangles as tracking candidate in tRects
 */
void TrackRectangles::SaveNewRectangleNonIncremental()
{
	for(unsigned idx=0; idx<NumRectangles(); idx++)
	{
/*		if(Rectangles(idx)->masked != UNDEF_ID)																									/// TODO TODO TODO use only unmasked rectangles
		{*/
			RectDef rd;
		
			for(unsigned i=0; i<4; i++)
			{
				rd.jcts[i] = Rectangles(idx)->jcts[i];
				rd.jctsIsct[i] = LJunctions(Rectangles(idx)->jcts[i])->isct;
			}
			rd.direction[0] = Rectangles(idx)->direction[0];
			rd.direction[1] = Rectangles(idx)->direction[1];
			rd.centerPoint = Rectangles(idx)->centerPoint;
	
			tRects.PushBack(new TRectangle(idx, rd));
		}
// 	}
}

/**
 * @brief Track the rectangles non-incremental.
 * @TODO Wie funktioniert das?
 * @brief 
 */
void TrackRectangles::TrackRectangleNonIncremental()
{

bool print = false;

/// TODO 
double maxAngle = 0.4;						// maximum deviation of angle for tracking
double maxLengthDev = 30.; 				// maximum deviation of line length // TODO sollte relativ zum Umfang sein!
double refDistance = 30;					// maximum deviation between calculated motion (foe) and real motion.
// double maxDistance = 30.;			// maximum deviation of center distance

// get all rectangle candidates (tRects) which age=0 and which are not tracked (id_former_tracked == UNDEF_ID)
// find 

if(print) printf("\n########## TrackRectangleNonIncremental ###########\n");
	for(unsigned i=0; i<tRects.Size(); i++)
	{
		if (tRects[i]->age == 0 && tRects[i]->id_former_tracked == UNDEF_ID)
		{
			Array<unsigned> bestMatches;				// array position in rRects
			Array<double> bestDistances;
																					/// TODO nicht refDistance verwende: Winkel der Bewegung wäre wahrscheinlich besser 
																					/// Winkel darf nur herangezogen werden, wenn das Motion Field "ein sehr wahrscheinliches" ist.

// if(print) printf("Suche Rechteck für Kandidaten: %u\n", tRects[i]->id);

			for(unsigned j=0; j<tRects.Size(); j++)
			{
				if (tRects[j]->age == 1)// && tRects[j]->id_next_tracked == UNDEF_ID) 			/// TODO Also multiple (backward-) assignment
				{
					Vector2 centerPointN = tRects[i]->rectDef.centerPoint;
					Vector2 centerPointO = tRects[j]->rectDef.centerPoint;
		
					/// distance to calculated distance from motion field
					Vector2 motion;
					for(unsigned k=0; k<NumMotionFields(); k++)
						if (MotionFields(k)->age == 0) MotionFields(k)->GetMotion(motion, centerPointO);

					double distance = (centerPointN - centerPointO - motion).Length();
	
					/// store best matches (distance)
					if (distance < refDistance)
					{
if (print) printf("		Kandidat for %u: %u || Distance: %4.2f || motion-length: %4.2f\n", tRects[i]->id, tRects[j]->id, distance, motion.Length());
						bestMatches.PushBack(j);	
						bestDistances.PushBack(distance);
					}
				}
			}
	
// printf("Nr of matches: %u\n", bestMatches.Size());

			// find rectangle with the minimum deviation of opposing edge angles and minimum deviation of side-length
			double refMinAngle = M_PI;							/// minimum angle
			unsigned bestResult = UNDEF_ID;
			double dist = 0.;
			double lengthDeviation = 0.;
			bool rectFound = false;
			if(bestMatches.Size() > 0)
			{
				for(unsigned j=0; j<bestMatches.Size(); j++)
				{
					/// calculate differences in directions
					Vector2 dir00 = tRects[i]->rectDef.direction[0];
					Vector2 dir01 = tRects[bestMatches[j]]->rectDef.direction[0];
					Vector2 dir10 = tRects[i]->rectDef.direction[1];
					Vector2 dir11 = tRects[bestMatches[j]]->rectDef.direction[1];
					
					double a = acos(Dot(dir00, dir01));
					if (a > M_PI/2.) a = M_PI - a;
					double b = acos(Dot(dir10, dir11));
					if (b > M_PI/2.) b = M_PI - b;
					double c = acos(Dot(dir00, dir11));
					if (c > M_PI/2.) c = M_PI - c;
					double d = acos(Dot(dir01, dir10));
					if (d > M_PI/2.) d = M_PI - d;
// if(print) printf("dif-angles: %4.2f - %4.2f - %4.2f - %4.2f\n", a, b, c, d);
					double minimumAngle = Min(a+b, c+d);
// if(print) printf("		Best matches for rectangle %u: %u => angle: %4.2f\n", tRects[i]->id, tRects[bestMatches[j]]->id, minimumAngle);
	// printf("	Unterschied der Winkel: %4.2f\n", minimumAngle);

					// calculate length deviation of opposing lines
					double lineLength0 = (tRects[i]->rectDef.jctsIsct[0] - tRects[i]->rectDef.jctsIsct[1]).Length() + 
															 (tRects[i]->rectDef.jctsIsct[1] - tRects[i]->rectDef.jctsIsct[2]).Length() +
															 (tRects[i]->rectDef.jctsIsct[2] - tRects[i]->rectDef.jctsIsct[3]).Length() + 
															 (tRects[i]->rectDef.jctsIsct[3] - tRects[i]->rectDef.jctsIsct[0]).Length();

					double lineLength1 = (tRects[bestMatches[j]]->rectDef.jctsIsct[0] - tRects[bestMatches[j]]->rectDef.jctsIsct[1]).Length() + 
																(tRects[bestMatches[j]]->rectDef.jctsIsct[1] - tRects[bestMatches[j]]->rectDef.jctsIsct[2]).Length() +
																(tRects[bestMatches[j]]->rectDef.jctsIsct[2] - tRects[bestMatches[j]]->rectDef.jctsIsct[3]).Length() + 
																(tRects[bestMatches[j]]->rectDef.jctsIsct[3] - tRects[bestMatches[j]]->rectDef.jctsIsct[0]).Length();

					double lengthDev = fabs(lineLength0 - lineLength1);

if(print) printf("		minimum distance of best match %u: %4.2f\n", j, lengthDev); 

					if(minimumAngle < refMinAngle && lengthDev < maxLengthDev)
					{
						refMinAngle = minimumAngle;
						bestResult = bestMatches[j] ;
						dist = bestDistances[j];
						lengthDeviation = lengthDev;
						rectFound = true;
					}
				}

if (print) if (rectFound)
{
	printf("			Best match for rectangle %u: %u => Angle: %4.2f => dist: %4.2f\n", tRects[i]->id, tRects[bestResult]->id, refMinAngle, dist);
	printf("			Length deviation: %4.2f\n", lengthDeviation);
}

				if (rectFound && refMinAngle < maxAngle)
				{
if(print) printf("			+++++++++++ Store best match: %u\n", tRects[bestResult]->id);
					tRects[i]->FormerTracked(tRects[bestResult]->id);
					tRects[bestResult]->NextTracked(tRects[i]->id);	

					/// Get tracked rectangles
					Array<unsigned> trktRectIDs;
					Array<RectDef> trktRects;
	
					trktRectIDs.PushBack(tRects[i]->id);
					trktRects.PushBack(tRects[i]->rectDef);

/// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO
/// Was passiert, wenn zwei verschiedene Rectangles (age=0) auf das selbe vorherige Rectangle getrackt werden?
/// wird der Eintrag überschrieben? Was passiert dann? Es gibt nur einen Weg vorwärts und einen Rückwärts, oder?
/// Für die Darstellung (Draw) wird der Rückwärtsweg verwendet, für das erstellen von Hypothesen der Vorwärtsweg.
/// Eigentlich sollte id_next_tracked ein Array sein, das mehrere Wege vorgibt.

					bool tracked = true;
					unsigned nextAge = 2;
/// Hier gibt es noch einen Fehler bei der Suche durch die Schleife! => daher abstechen nach maxAge
					while(tracked) // && nextAge < maxAge)
					{
						tracked = false;
						trktRectIDs.PushBack(tRects[bestResult]->id);
						trktRects.PushBack(tRects[bestResult]->rectDef);
	
						if(tRects[bestResult]->id_former_tracked != UNDEF_ID)
						{	
if(print) printf("tracked: %u\n", tRects[bestResult]->id_former_tracked);
							for(unsigned l=0; l<tRects.Size(); l++)
							{
								if(tRects[l]->age == nextAge && tRects[l]->id == tRects[bestResult]->id_former_tracked)
								{
if(print) printf("former tracked found!\n");
									tracked = true;
									bestResult = l;
									nextAge++;
								}
							}
						}
//						else /*tracked = false*/break;
					}
	
					/// Create new TrktRectangle
					NewGestalt(new TrktRectangle(trktRectIDs, trktRects));

				}
			}
// if(print) printf("\n");
		}
	}
}


/**
 * @brief Track the new rectangle. (incremental)
 * @TODO implement function?
 */
void TrackRectangles::TrackRectangleIncremental(unsigned idx)
{
	
}
}


















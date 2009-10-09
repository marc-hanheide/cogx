/**
 * @file TrackCubes.cc
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief Tracking cubes
 **/

#include "TrackCubes.hh"
#include "MotionField.hh"
#include "TrktCube.hh"
#include "TrktFlap.hh"
#include "TrktRectangle.hh"
#include "Cube.hh"
#include "Flap.hh"
#include "Math.hh"

namespace Z
{

/**
 * @brief Constructor of class TCubes 
 */
TrackCubes::TCube::TCube(unsigned i, TCubeDef cd)
{
  id = i;
  age = 0;

	tCubeDef = cd;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void TrackCubes::TCube::FormerTracked(unsigned id)
{
  id_former_tracked = id;
}

void TrackCubes::TCube::NextTracked(unsigned id)
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
void TrackCubes::AgeTObjects(unsigned maxAge)
{
  AgeTCubes(maxAge);
}

/**
 * @brief Increment age of tCubes and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void TrackCubes::AgeTCubes(unsigned maxAge)
{
  for(unsigned i=tCubes.Size(); i>0; i--)
  {
     tCubes[i-1]->age++;
     if (tCubes[i-1]->age > maxAge) tCubes.Erase(i-1);
  }
}

// ************************************************************************************ //
// Track Cubes
// ************************************************************************************ //

static int CmpTrktCubes(const void *a, const void *b)
{
  if( TrktCubes(*(unsigned*)a)->sig > TrktCubes(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void TrackCubes::Rank()
{
  RankGestalts(Gestalt::TRKT_CUBE, CmpTrktCubes);
}

/**
 * @brief Masking Tracked Rectangles
 */
void TrackCubes::Mask()
{
}

bool TrackCubes::NeedsOperate()
{ 
  return false;	/// TODO
}

void TrackCubes::Reset(const Image *img)
{
	AgeTObjects(maxAge);
}

TrackCubes::TrackCubes(Config *cfg) : GestaltPrinciple(cfg)
{
	firstCall = true;
	hypAge = 1;								/// TODO Try hypAge > 1
	nextCubeID = 10000;
}

/**
 * @brief InformNewGestalt()
 */
void TrackCubes::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
//   StartRunTime();

//   if (type == Gestalt::CUBE)
// 	{
// 		SaveNewCube(idx);					// incremental saving
//		if(!firstCall) TrackRectangleIncremental(idx);							/// TODO inkremental call for tracking
// 	}

//   StopRunTime();
}

/**
 * @brief Operate()
 */
void TrackCubes::Operate(bool incremental)
{
//   StartRunTime();
// 	cout << "TrackRectangles::Operate" << endl;
//   StopRunTime();
}

/**
 * @brief OperateNonIncremental()
 */
void TrackCubes::OperateNonIncremental()
{
  StartRunTime();

	GetCubes();
	if(!firstCall) 
	{
		TrackCubeNonIncremental();
// 		HypothesiseCube(hypAge);
		HypothesiseCubeFromFlap(hypAge);
	}
	
	Rank();
	Mask();

	firstCall = false;
	StopRunTime();
}

/**
 * @brief Save new cubes for tracking (incremental => with masked cubes)
 * @param idx Index of the new cube.
 */
void TrackCubes::SaveNewCube(unsigned idx)
{
	TCubeDef cd;

	cd.hypothesised = false;
	cd.sig = Cubes(idx)->sig;

	for(unsigned i=0; i<4; i++)
	{
			cd.corner_points[i][0] = Cubes(idx)->corner_points[i][0];
			cd.corner_points[i][1] = Cubes(idx)->corner_points[i][1];
	}
	cd.center = Cubes(idx)->center;
	cd.radius = Cubes(idx)->radius;

	tCubes.PushBack(new TCube(idx, cd));
}

/**
 * @brief Get all cubes and save it for tracking. (Non-incremental)
 */
void TrackCubes::GetCubes()
{
	for(unsigned idx=0; idx<NumCubes(); idx++)
	{
		if (Cubes(idx)->masked < 10000 || Cubes(idx)->masked == UNDEF_ID)					/// TODO Cubes wich are masked > 10000 are geometric false cubes.
//		if (Cubes(idx)->masked == UNDEF_ID)					/// TODO Take only masked cubes
		{
			TCubeDef cd;

			cd.hypothesised = false;		
			cd.sig = Cubes(idx)->sig;

			for(unsigned j=0; j<4; j++)
			{
					cd.corner_points[j][0] = Cubes(idx)->corner_points[j][0];
					cd.corner_points[j][1] = Cubes(idx)->corner_points[j][1];
			}
			cd.center = Cubes(idx)->center;
			cd.radius = Cubes(idx)->radius;

			cd.flap = Cubes(idx)->flap;
			cd.oFlaps[0] = Cubes(idx)->oFlaps[0];
			cd.oFlaps[1] = Cubes(idx)->oFlaps[1];

			cd.rect[0] = Flaps(Cubes(idx)->flap)->rects[0];
			cd.rect[1] = Flaps(Cubes(idx)->flap)->rects[1];
			if(cd.oFlaps[0] != UNDEF_ID)
			{
				if(Flaps(cd.oFlaps[0])->rects[0] != cd.rect[0] && Flaps(cd.oFlaps[0])->rects[0] != cd.rect[1])
					cd.rect[2] = Flaps(cd.oFlaps[0])->rects[0];
				else
					cd.rect[2] = Flaps(cd.oFlaps[0])->rects[1];
			}
			else cd.rect[2] = UNDEF_ID;

			tCubes.PushBack(new TCube(idx, cd));
		}
	}
}


/**
 * @brief Track the cubes non-incremental.
 * Tracking considering only the distance of center points.
 * Allocation to hypothesized cubes multiplies 1.5*distance, to allocate non-hypothesized cubes in favour.
 */
void TrackCubes::TrackCubeNonIncremental()
{
bool print = false;

if(print) printf("TrackCubeNonIncremental\n");

// printf("Cubes for tracking: %u\n", tCubes.Size());

	for(unsigned i=0; i<tCubes.Size(); i++)
	{
		if (tCubes[i]->age == 0 && tCubes[i]->id_former_tracked == UNDEF_ID)	/// TODO TODO TODO TODO TODO TODO Tracking über mehrere Bilder???
		{
if(print) printf("Try to track cube: %u\n", tCubes[i]->id);
			Array<unsigned> bestMatches;				// array position in rRects
			Array<double> bestDistances;
			double refDistance = 40;						/// TODO Abweichung von berechneter Stelle darf 20 pixel sein!!! => sig value ausrechnen?
																					/// TODO Wie bei TrackFlaps.cc auf Winkel ändern?

			for(unsigned j=0; j<tCubes.Size(); j++)
			{
				if (tCubes[j]->age == 1) // && tCubes[j]->id_next_tracked == UNDEF_ID)			/// TODO Also multiple (backward-) assignment
				{
					Vector2 centerPointN = tCubes[i]->tCubeDef.center;
					Vector2 centerPointO = tCubes[j]->tCubeDef.center;
		
					/// distance to calculated distance from motion field
					Vector2 motion;
					for(unsigned k=0; k<NumMotionFields(); k++)
						if (MotionFields(k)->age == 0) MotionFields(k)->GetMotion(motion, centerPointO);

					double distance = (centerPointN - centerPointO - motion).Length();

					/// TODO Bevorzuge nicht hypothetisierte Würfel!!!
					if(tCubes[j]->tCubeDef.hypothesised)
					{
if(print) printf("  Recalculate distance: %4.2f	=> cube %u\n", distance, tCubes[j]->id);
						distance *= 2.;
					}

					/// store the best matches (per distance)
					if (distance < refDistance)
					{
if(print) printf("	New best match: %u 	with distance: %4.3f\n", tCubes[j]->id, distance);
						bestMatches.PushBack(j);	
						bestDistances.PushBack(distance);
					}
				}
			}
//if(print)  printf("Nr of matches: %u\n", bestMatches.Size());

			bool cubeTracked = false;
			unsigned bestResult = UNDEF_ID;
			refDistance = 20.;								/// TODO Threshold for minimum distance deviation
			for(unsigned j=0; j<bestMatches.Size(); j++)
			{
//if(print)  printf("Best matches for %u is %u => distance = %4.2f\n", i, tCubes[bestMatches[j]]->id, bestDistances[j]);
				if(bestDistances[j] < refDistance)
				{
					bestResult = bestMatches[j];
					refDistance = bestDistances[j];
					cubeTracked = true;
				}
			}

//if(print)  printf("Best result for %u is %u => distance = %4.2f\n", i, tCubes[bestResult]->id, refDistance);

			if (cubeTracked)
			{
if(print) printf("  Cube tracked with former cube: %u\n", tCubes[bestResult]->id);
				tCubes[i]->FormerTracked(tCubes[bestResult]->id);
				tCubes[bestResult]->NextTracked(tCubes[i]->id);	

				/// Get tracked cubes
				Array<unsigned> trktCubeIDs;
				Array<TCubeDef> trktCubes;

				trktCubeIDs.PushBack(tCubes[i]->id);
				trktCubes.PushBack(tCubes[i]->tCubeDef);

				bool tracked = true;
				unsigned nextAge = 2;
				while(tracked && nextAge < maxAge)
				{
					tracked = false;
					trktCubeIDs.PushBack(tCubes[bestResult]->id);
					trktCubes.PushBack(tCubes[bestResult]->tCubeDef);

					if(tCubes[bestResult]->id_former_tracked != UNDEF_ID)
					{	
						for(unsigned l=0; l<tCubes.Size(); l++)
						{
							if(tCubes[l]->age == nextAge && tCubes[l]->id == tCubes[bestResult]->id_former_tracked)
							{
								tracked = true;
								bestResult = l;
								nextAge++;
							}
						}
					}
				}
				NewGestalt(new TrktCube(trktCubeIDs, trktCubes));
			}
		}
	}
}


/**
 * @brief Generate hypotheses for cubes from a flap.
 * @param hypAge Maximum age for creating hypothesis.
 */
void TrackCubes::HypothesiseCubeFromFlap(unsigned hypAge)
{
bool print = false;

if (print) printf("\nHypothesiseCubeFromFlap\n");

	for(unsigned idx=0; idx<tCubes.Size(); idx++)
	{
		if(tCubes[idx]->age <= maxAge && tCubes[idx]->age > 0 && tCubes[idx]->id_next_tracked == UNDEF_ID)
		{

			bool isHypothesized = false;					// TODO Steht an falscher Stelle (eine klammer rauf)!!!   /// cube is already hypothesized?

if (print) printf("    Cube to hypothesize: %u\n", tCubes[idx]->id);
			Array<unsigned> flaps;

			flaps.PushBack(tCubes[idx]->tCubeDef.flap);
			if(tCubes[idx]->tCubeDef.oFlaps[0] != UNDEF_ID) 		// get the two other flaps, when they exist
			{
				flaps.PushBack(tCubes[idx]->tCubeDef.oFlaps[0]);
				flaps.PushBack(tCubes[idx]->tCubeDef.oFlaps[1]);
			}

			for(unsigned f=0; f<flaps.Size(); f++)
			{
				unsigned flap = flaps[f];
				unsigned cubeAge = tCubes[idx]->age;

if (print) printf("	HypothesiseCubeFromFlap: Cube %u (pos: %u) with flap: %u\n", tCubes[idx]->id, idx, flap);
			

				// find flap with id=flap at TrktFlaps (and with age
				for(unsigned j=0; j<NumTrktFlaps(); j++)
				{
					bool foundFlap = false;								// flap could be found?
// 					bool isHypothesized = false;					// TODO Steht an falscher Stelle (eine klammer rauf)!!!   /// cube is already hypothesized?
					unsigned position = UNDEF_ID;
	
					if(TrktFlaps(j)->tFlapIDs.Find(flap) != UNDEF_ID  && flap != UNDEF_ID)
					{
if (print) printf("		Flap %u found at TrktFlaps[%u]\n", flap, j);
						position = TrktFlaps(j)->tFlapIDs.Find(flap);			// position in tFlapIDs is also the age !!! (is this sure?)
						if(position == cubeAge)
						{
							foundFlap = true;
						}
					}

if (print) if(foundFlap)
	printf("	Flap %u found at TrktFlaps %u: id=%u	|| age=%u\n", flap, TrktFlaps(j)->tFlapIDs[0], j, TrktFlaps(j)->tFlapIDs.Find(flap));
	
					if(foundFlap && !isHypothesized)
					{
if (print) printf("      Cube hypothesized from Flap: %u\n", TrktFlaps(j)->tFlapIDs[0]);
						// Calculate the motion of the flap
						Vector2 motion = (TrktFlaps(j)->tFlaps[cubeAge]).center - (TrktFlaps(j)->tFlaps[0]).center;
	
	// printf("	center points %u: %4.2f - %4.2f\n", cubeAge, (TrktFlaps(j)->tFlaps[cubeAge]).center.x, (TrktFlaps(j)->tFlaps[cubeAge]).center.y);
	// printf("	center points %u: %4.2f - %4.2f\n", cubeAge-1, (TrktFlaps(j)->tFlaps[cubeAge-1]).center.x, (TrktFlaps(j)->tFlaps[cubeAge-1]).center.y);
	// printf("	motion between flaps nr: %u - %u: %4.2f - %4.2f\n", cubeAge, cubeAge-1, motion.x, motion.y);
	
						/// Create new moved cube and store it as hypothesized cube
						TCubeDef cd = tCubes[idx]->tCubeDef;									// overtake old cube and correct with the calculated motion
						cd.hypothesised = true;
						cd.flap = TrktFlaps(j)->tFlapIDs[position-1];
						cd.oFlaps[0] = UNDEF_ID;
						cd.oFlaps[1] = UNDEF_ID;
						
						/// TODO TODO TODO TODO  Recalculate the Cube
						/// Wieso -=motion? Stimmt das immer?
						for(unsigned k=0; k<4; k++)
						{
							cd.corner_points[k][0] -= motion;
							cd.corner_points[k][1] -= motion;
						}
						cd.center -= motion;
	
	
						unsigned id = GetCubeID();														// get new index, for referencing
						tCubes.PushBack(new TCube(id, cd));

						unsigned hypCubeId = id;
						unsigned hypCubePos = tCubes.Size()-1;								// the last cube is the hypothesized one
	
						Array<unsigned> trktCubeIDs;													// create new trktCube arrays
						Array<TCubeDef> trktCubes;
		
						trktCubeIDs.PushBack(tCubes[hypCubePos]->id);					// store hypothesized cube
						trktCubes.PushBack(tCubes[hypCubePos]->tCubeDef);
	
						tCubes[hypCubePos]->FormerTracked(tCubes[idx]->id);		// store former/next cube
						tCubes[idx]->NextTracked(tCubes[hypCubePos]->id);
	
						// Find all former cubes and store them
						bool tracked = true;
						unsigned nextAge = 2;
						unsigned trackedID = idx;
						while(tracked && nextAge < maxAge)
						{
							tracked = false;
							trktCubeIDs.PushBack(tCubes[trackedID]->id);
							trktCubes.PushBack(tCubes[trackedID]->tCubeDef);
		
							if(tCubes[trackedID]->id_former_tracked != UNDEF_ID)
							{	
								for(unsigned l=0; l<tCubes.Size(); l++)
								{
									if(tCubes[l]->age == nextAge && tCubes[l]->id == tCubes[trackedID]->id_former_tracked)
									{
										tracked = true;
										trackedID = l;
										nextAge++;
									}
								}
							}
						}
						NewGestalt(new TrktCube(trktCubeIDs, trktCubes));
						isHypothesized = true;
					}
				}
			}
		}
	}
if (print) printf("HypothesiseCubeFromFlap :: End\n");
}












/*
 * @brief Generate hypotheses for cubes from a rectangle.

void TrackCubes::HypothesiseCubeFromRectangle(unsigned idx, unsigned maxAge)
{
	/// Get first rectangle
	unsigned rect = tCubes[idx]->tCubeDef.rect[0];

	printf("HypothesiseCubeFromRectangle: Cube %u with rect	: %u\n", tCubes[idx]->id, rect);

	/// suchen nach TrktFlaps, mit der richtigen id und age = 1
	for(unsigned i=0; i<NumTrktRectangles(); i++)
	{
// 		TrktFlap *tf = TrktFlaps(i);	
//		Array<unsigned> fIDs = TrktFlaps(i).tFlapIDs;
// 		Array<unsigned> fIDs = TrktFlaps(i)->tFlapIDs;

		if(TrktRectangles(i)->tRectIDs.Find(rect) != UNDEF_ID)
		{
			printf("	Rect found at TrktRects: %u\n", i);

			/// alle i's merken

		}
	}

}
 */

/**
 * @brief Track the new rectangle.
 */
void TrackCubes::TrackCubeIncremental(unsigned idx)
{
	
}

/**
 * @brief Generate a cube ID for a hypothesized cube.
 */
unsigned TrackCubes::GetCubeID()
{
	nextCubeID++;
	return nextCubeID;
}

}


















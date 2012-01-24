/**
 * @file TrackFlaps.cc
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief Tracking flaps.
 **/

#include "TrackFlaps.hh"
#include "MotionField.hh"
#include "TrktFlap.hh"
#include "TrktRectangle.hh"
#include "Flap.hh"
#include "LJunction.hh"
#include "Math.hh"

namespace Z
{

/**
 * @brief Constructor of class TFlap 
 * @param i Id of the origin Gestalt
 * @param fd Flap properties
 */
TrackFlaps::TFlap::TFlap(unsigned i, TFlapDef fd)
{
  id = i;
  age = 0;

	flapDef = fd;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void TrackFlaps::TFlap::FormerTracked(unsigned id)
{
  id_former_tracked = id;
}

void TrackFlaps::TFlap::NextTracked(unsigned id)
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
void TrackFlaps::AgeTObjects(unsigned maxAge)
{
  AgeTFlaps(maxAge);
}

/**
 * @brief Increment age of tCubes and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void TrackFlaps::AgeTFlaps(unsigned maxAge)
{
  for(unsigned i=tFlaps.Size(); i>0; i--)
  {
     tFlaps[i-1]->age++;
     if (tFlaps[i-1]->age > maxAge) tFlaps.Erase(i-1);
  }
}

// ************************************************************************************ //
// Track Flaps
// ************************************************************************************ //

static int CmpTrktFlaps(const void *a, const void *b)
{
  if( TrktFlaps(*(unsigned*)a)->sig > TrktFlaps(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void TrackFlaps::Rank()
{
  RankGestalts(Gestalt::TRKT_FLAP, CmpTrktFlaps);
}

/**
 * @brief Masking Tracked Flaps
 */
void TrackFlaps::Mask()
{
}

bool TrackFlaps::NeedsOperate()
{ 
  return false;	/// TODO
}

void TrackFlaps::Reset(const Image *img)
{
	AgeTObjects(maxAge);
}

/**
 * @brief Constructor of class TrackFlaps
 */
TrackFlaps::TrackFlaps(Config *cfg) : GestaltPrinciple(cfg)
{
	firstCall = true;
	hypAge = 1;								///< Maximum age of Gestalt for creating hypothesis.
	nextFlapID = 10000;				///< Next ID for hypothesized flaps.
}

/**
 * @brief Inform the principle about a new Gestalt
 * @param type Type of Gestalt
 * @param idx Index of Gestalt
 */
void TrackFlaps::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
}

/**
 * @brief Operate
 */
void TrackFlaps::Operate(bool incremental)
{
}

/**
 * @brief Operate the principle non incremental
 */
void TrackFlaps::OperateNonIncremental()
{
	bool print = false;

  StartRunTime();
if(print) printf("TrackFlaps - Start\n");

// 	cout << "TrackRectangles::OperateNonIncremental" << endl;
	GetFlaps();				// Get and save flaps for tracking without the masked flaps (=> this works only non-incremental)
if(print) printf("GetFlaps - End\n");

	if(!firstCall)
	{ 
		TrackFlapNonIncremental();
if(print) printf("TrackFlapNonIncremental - End\n");

		HypothesiseFlapFromRectangle(hypAge);
if(print) printf("HypothesiseFlapFromRectangle - End\n");
	}
	Rank();
	Mask();

if(print) printf("TrackFlaps - End\n");

	firstCall = false;
	StopRunTime();
}

/**
 * @brief Save new informed flap for tracking. (incremental flap saving)
 * @TODO Unused, because of the non-incremental processing
 */
void TrackFlaps::SaveNewFlap(unsigned idx)
{
// cout << "TrackFlaps::SaveNewFlap" << endl;
	if (!Flaps(idx)->IsMasked())	
	{
// printf("Flap %u is unmasked!\n", idx);
		TFlapDef fd;
		fd.hypothesised = false;

		fd.rects[0] = Flaps(idx)->rects[0];
		fd.rects[1] = Flaps(idx)->rects[1];
		
		for(unsigned i=0; i<4; i++)
		{
			fd.innerJcts[i] = Flaps(idx)->innerJcts[i];
			fd.innerJctsIsct[i] = LJunctions(Flaps(idx)->innerJcts[i])->isct;
			fd.outerJcts[i] = Flaps(idx)->outerJcts[i];
			fd.outerJctsIsct[i] = LJunctions(Flaps(idx)->outerJcts[i])->isct;
		}

		fd.center = Flaps(idx)->center;

		fd.rectCenter[0] = Flaps(idx)->rectCenter[0];
		fd.rectCenter[1] = Flaps(idx)->rectCenter[1];
		fd.rectRadius[0] = Flaps(idx)->rectRadius[0];
		fd.rectRadius[1] = Flaps(idx)->rectRadius[1];
		
		tFlaps.PushBack(new TFlap(idx, fd));
	}
// 	else printf("Flap %u is masked!\n", idx);
}

/**
 * @brief Get all flaps and save for tracking. (non-incremental flap saving)
 */
void TrackFlaps::GetFlaps()
{
	for(unsigned idx=0; idx<NumFlaps(); idx++)
	if (true)// !Flaps(idx)->IsMasked())							/// TODO TODO TODO verwende auch die nicht maskierten Flaps
	{
		TFlapDef fd;
		fd.hypothesised = false;

		fd.rects[0] = Flaps(idx)->rects[0];
		fd.rects[1] = Flaps(idx)->rects[1];
		
		for(unsigned i=0; i<4; i++)
		{
			fd.innerJcts[i] = Flaps(idx)->innerJcts[i];
			fd.innerJctsIsct[i] = LJunctions(Flaps(idx)->innerJcts[i])->isct;
			fd.outerJcts[i] = Flaps(idx)->outerJcts[i];
			fd.outerJctsIsct[i] = LJunctions(Flaps(idx)->outerJcts[i])->isct;
		}

		fd.center = Flaps(idx)->center;

		fd.rectCenter[0] = Flaps(idx)->rectCenter[0];
		fd.rectCenter[1] = Flaps(idx)->rectCenter[1];
		fd.rectRadius[0] = Flaps(idx)->rectRadius[0];
		fd.rectRadius[1] = Flaps(idx)->rectRadius[1];
		
		tFlaps.PushBack(new TFlap(idx, fd));
	}
}


/**
 * @brief Track flap non-incremental.
 * Track to nearest 
 */
void TrackFlaps::TrackFlapNonIncremental()
{
bool print = false;

// printf("TrackFlapNonIncremental\n");
	for(unsigned i=0; i<tFlaps.Size(); i++)
	{
		if (tFlaps[i]->age == 0 && tFlaps[i]->id_former_tracked == UNDEF_ID)
		{
if(print) printf("  Track Flap %u\n", tFlaps[i]->id);
			unsigned bestMatch = UNDEF_ID;
			double refDistance = 40;						/// TODO Abweichung von berechneter Stelle darf 30 pixel sein!!! => sig value ausrechnen?

			for(unsigned j=0; j<tFlaps.Size(); j++)
			{
				if (tFlaps[j]->age == 1) // && tFlaps[j]->id_next_tracked == UNDEF_ID)			/// TODO auch age > 1 => tracking über mehrere Frames!
				{
					Vector2 rectCenterPointN0 = tFlaps[i]->flapDef.rectCenter[0];		// new rect center points
					Vector2 rectCenterPointN1 = tFlaps[i]->flapDef.rectCenter[1];
					Vector2 rectCenterPointO0 = tFlaps[j]->flapDef.rectCenter[0];		// old rect center points
					Vector2 rectCenterPointO1 = tFlaps[j]->flapDef.rectCenter[1];
		
					// distance to calculated distance from motion field
					Vector2 motion[2];
					for(unsigned k=0; k<NumMotionFields(); k++)
						if (MotionFields(k)->age == 0) 
						{
							MotionFields(k)->GetMotion(motion[0], rectCenterPointO0);
							MotionFields(k)->GetMotion(motion[1], rectCenterPointO1);
						}
// printf("Motion: %4.2f / %4.2f\n", motion[0].x, motion[0].y);

					/// Find best match: Compare rectangle center distances (recalculated with motion)
					double distance0 = (rectCenterPointN0 - rectCenterPointO0 - motion[0]).Length() + 
														 (rectCenterPointN1 - rectCenterPointO1 - motion[0]).Length();
					double distance1 = (rectCenterPointN0 - rectCenterPointO1 - motion[1]).Length() + 
														 (rectCenterPointN1 - rectCenterPointO0 - motion[1]).Length();
					double distance = Min(distance0, distance1);	

					/// TODO Bevorzuge nicht hypothesierte Flaps!!!
					if(tFlaps[j]->flapDef.hypothesised)
					{
if(print) printf("   Recalculate distance: %4.2f	=> flap %u\n", distance, tFlaps[j]->id);
						distance *= 1.5;
					}

					// store best match (distance)
					if (distance < refDistance)
					{
if(print) printf(" 		Match found %u: distance: distance: %4.2f\n", tFlaps[j]->id, distance);
// if(print) printf(" 	=> Recalculated distance: %4.2f\n", distance);
						refDistance = distance;
						bestMatch = j;

					}
				}
			}


			if (bestMatch != UNDEF_ID)
			{
if(print) printf("		Best mach for %u is %u => distance: %4.2f\n", tFlaps[i]->id, tFlaps[bestMatch]->id, refDistance); 

				Array<unsigned> trktFlapIDs;
				Array<TFlapDef> trktFlaps;

				trktFlapIDs.PushBack(tFlaps[i]->id);
				trktFlaps.PushBack(tFlaps[i]->flapDef);

				tFlaps[i]->FormerTracked(tFlaps[bestMatch]->id);
				tFlaps[bestMatch]->NextTracked(tFlaps[i]->id);	

				bool tracked = true;
				unsigned nextAge = 2;
				while(tracked && nextAge < maxAge)
				{
					tracked = false;
					trktFlapIDs.PushBack(tFlaps[bestMatch]->id);
					trktFlaps.PushBack(tFlaps[bestMatch]->flapDef);

					if(tFlaps[bestMatch]->id_former_tracked != UNDEF_ID)
					{	
						for(unsigned l=0; l<tFlaps.Size(); l++)
						{
							if(tFlaps[l]->age == nextAge && tFlaps[l]->id == tFlaps[bestMatch]->id_former_tracked)
							{
								tracked = true;
								bestMatch = l;
								nextAge++;
							}
						}
					}
				}

				NewGestalt(new TrktFlap(trktFlapIDs, trktFlaps));
			}
		}
	}
}

/**
 * @brief Track the new flap (incremental).
 * @param idx Index of flap to track.
 */
void TrackFlaps::TrackFlapIncremental(unsigned idx)
{
	
}


/**
 * @brief Generate Flap hypotheses from a rectangle.
 * How: TODO
 * @param hypAge Maximum age of Gestalts to create hypotheses.
 */
void TrackFlaps::HypothesiseFlapFromRectangle(unsigned hypAge)
{
//printf("\nHypothesizeFlapFromRectangle\n");
	for(unsigned i=0; i<tFlaps.Size(); i++)
	{
		if(tFlaps[i]->age <= hypAge && tFlaps[i]->age > 0 && tFlaps[i]->id_next_tracked == UNDEF_ID)
		{
			/// Get the rectangles
			unsigned rect0 = tFlaps[i]->flapDef.rects[0];
			unsigned rect1 = tFlaps[i]->flapDef.rects[1];
		
			unsigned flapAge = tFlaps[i]->age;

// printf("HypothesiseFlapFromRectangle: Flap %u (pos:%u) with rects: %u - %u	|| flapAge: %u\n", tFlaps[i]->id, i, rect0, rect1, tFlaps[i]->age);

			/// suchen nach TrktRectangles, mit der richtigen id und age = flapAge ???
			for(unsigned j=0; j<NumTrktRectangles(); j++)
			{
				bool isRect0 = false;
				bool isRect1 = false;
				unsigned position = UNDEF_ID;

				if(TrktRectangles(j)->tRectIDs.Find(rect0) != UNDEF_ID && rect0 != UNDEF_ID)		// rect0 != UNDEF_ID => avoid tracking of UNDEF_ID
				{
					position = TrktRectangles(j)->tRectIDs.Find(rect0);			/// position in tRectIDs is also the age !!! (or not?)
					if(position == flapAge)
					{
						isRect0 = true;
					}
				}

				if(TrktRectangles(j)->tRectIDs.Find(rect1) != UNDEF_ID && rect1 != UNDEF_ID)
				{
					position = TrktRectangles(j)->tRectIDs.Find(rect1);			/// position in tRectIDs is also the age !!! (or not?)
					if(position == flapAge)
					{
						isRect1 = true;
					}
				}

				if (isRect0 || isRect1)
				{
					isRect0 = false;
					isRect1 = false;

// 					if(isRect0)
// 						printf("	Rect %u found at TrktRects: %u	|| age=%u\n", rect0, j, TrktRectangles(j)->tRectIDs.Find(rect0));
// 					if(isRect1)
// 						printf("	Rect %u found at TrktRects: %u	|| age=%u\n", rect1, j, TrktRectangles(j)->tRectIDs.Find(rect1));
// 					printf("	New rectangle is %u\n", TrktRectangles(j)->tRectIDs[position-1]);


					/// Calculate the motion of the rectangle
					Vector2 motion = (TrktRectangles(j)->tRects[flapAge]).centerPoint - (TrktRectangles(j)->tRects[0]).centerPoint;

					TFlapDef fd = tFlaps[i]->flapDef;									// overtake old flap and correct with the calculated motion
					fd.hypothesised = true;

					if (isRect0) fd.rects[0] = TrktRectangles(j)->tRectIDs[position-1];			/// TODO position - 1 ???
					else fd.rects[0] = UNDEF_ID;
					if (isRect1) fd.rects[1] = TrktRectangles(j)->tRectIDs[position-1];
					else fd.rects[1] = UNDEF_ID;

					/// TODO TODO TODO TODO  Recalculate the Flap
					for(unsigned k=0; k<4; k++)
					{
						fd.innerJctsIsct[k] -= motion;
						fd.outerJctsIsct[k] -= motion;
					}
					fd.rectCenter[0] -= motion;
					fd.rectCenter[1] -= motion;
					fd.center -=motion;

					/// get id for the new tFlap
					unsigned idx = GetFlapID();
					tFlaps.PushBack(new TFlap(idx, fd));								// get new index for referencing

					unsigned hypFlapId = idx;
					unsigned hypFlapPos = tFlaps.Size()-1;							// the last flap is the hypothesized one

					Array<unsigned> trktFlapIDs;												// create new trktFlap arrays
					Array<TFlapDef> trktFlaps;
	
					trktFlapIDs.PushBack(tFlaps[hypFlapPos]->id);				// store hypothesized flap
					trktFlaps.PushBack(tFlaps[hypFlapPos]->flapDef);
	
					tFlaps[hypFlapPos]->FormerTracked(tFlaps[i]->id);		// store former/next flap
					tFlaps[i]->NextTracked(tFlaps[hypFlapPos]->id);

					// Find all former flaps and store it
					bool tracked = true;
					unsigned nextAge = 2;
					unsigned trackedID = i;
					while(tracked && nextAge < maxAge)
					{
						tracked = false;
						trktFlapIDs.PushBack(tFlaps[trackedID]->id);
						trktFlaps.PushBack(tFlaps[trackedID]->flapDef);
	
						if(tFlaps[trackedID]->id_former_tracked != UNDEF_ID)
						{	
							for(unsigned l=0; l<tFlaps.Size(); l++)
							{
								if(tFlaps[l]->age == nextAge && tFlaps[l]->id == tFlaps[trackedID]->id_former_tracked)
								{
									tracked = true;
									trackedID = l;
									nextAge++;
								}
							}
						}
					}
					NewGestalt(new TrktFlap(trktFlapIDs, trktFlaps));
				}
			}
		}
	}
}

/**
 * @brief Generate a flap ID for a hypothesized flap.
 */
unsigned TrackFlaps::GetFlapID()
{
	nextFlapID++;
	return nextFlapID;
}

}


















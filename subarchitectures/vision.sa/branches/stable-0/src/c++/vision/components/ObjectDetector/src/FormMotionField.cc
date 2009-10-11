/**
 * @file FormMotionField.cc
 * @author Richtsfeld Andreas
 * @date Februar 2009
 * @version 0.1
 * @brief The class caculates the motion field
 **/

#include "FormMotionField.hh"
#include "MotionField.hh"
#include "LJunction.hh"
#include "Line.hh"
#include <map>

#include "Gestalt.hh"


namespace Z
{

/**
 * @brief Constructor of class TLJct 
 */
FormMotionField::TLJct::TLJct(unsigned i, LJctDef ld)
{
  id = i;
  age = 0;

	lJctDef = ld;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void FormMotionField::TLJct::FormerTracked(unsigned id)
{
  id_former_tracked = id;
}

void FormMotionField::TLJct::NextTracked(unsigned id)
{
	id_next_tracked = id;
}

/**
 * @brief Constructor of class TMotion 
 */
FormMotionField::TMotionField::TMotionField(unsigned i, MotionDef md)
{
  id = i;
  age = 0;

	motDef = md;
}

// ************************************************************************************ //
// Age the Objects
// ************************************************************************************ //

/**
 *	@brief Increase age of all objects (step counter) and erase if one is older than maxAge
 *	@param maxAge Maximum of age befor object will be deleted
 */
void FormMotionField::AgeTObjects(unsigned maxAge)
{
  AgeTLJcts(maxAge);
	AgeMotionField(maxAge);
}

/**
 * @brief Increment age of tCubes and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void FormMotionField::AgeTLJcts(unsigned maxAge)
{
  for(unsigned i=tLJcts.Size(); i>0; i--)
  {
     tLJcts[i-1]->age++;
     if (tLJcts[i-1]->age > maxAge) tLJcts.Erase(i-1);
  }
}

/**
 * @brief Increment age of tCubes and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void FormMotionField::AgeMotionField(unsigned maxAge)
{
  for(unsigned i=motField.Size(); i>0; i--)
  {
     motField[i-1]->age++;
     if (motField[i-1]->age > maxAge) motField.Erase(i-1);
  }
}

// ************************************************************************************ //
// Form Motion Field
// ************************************************************************************ //

static int CmpMotion(const void *a, const void *b)
{
  if( MotionFieldElements(*(unsigned*)a)->sig > MotionFieldElements(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormMotionField::Rank()
{
  // TODO ARI: 
  RankGestalts(Gestalt::MOTION_FIELD_ELEMENT, CmpMotion);
}

/**
 *	@brief Mask Motion_Field_Elements, using the mean direction of all elements.
 */
void FormMotionField::Mask()
{
  for(unsigned i=0; i<NumMotionFieldElements(); i++)
  {
		double alpha = acos(Dot(MotionFieldElements(i)->dir, meanDirection));
		if(alpha > M_PI/3.) MotionFieldElements(i)->Mask(i);											/// TODO Threshold for masking poor results (+-60°)
  }
}

bool FormMotionField::NeedsOperate()
{ 
  return needsOperate;	
}

FormMotionField::FormMotionField(Config *cfg) : GestaltPrinciple(cfg)
{
  needsOperate = false;
	firstCall = true;
}

void FormMotionField::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
  StartRunTime();

  if (type == Gestalt::L_JUNCTION)
	{
		/// TODO Threshold: use only L-Junctions with line length of 10 pixels.
		if(Lines(LJunctions(idx)->line[0])->len > 10.0 && Lines(LJunctions(idx)->line[1])->len > 10.0)
		{
// cout << "Inform New Gestalt:" << idx << endl;
			SaveLJunction(idx);
// cout << "SaveLJunction end" << endl;
			if (!firstCall)
			{
				TrackLJunction(idx);
// cout << "TrackLJunctions end" << endl;
				CreateNewMotionField();
// cout << "CreateNewMotionField end" << endl;
			}
		}
// cout << "Inform New Gestalt end" << endl;
	}

	Rank();
  StopRunTime();
}

/**
 *	@brief OperateNonIncremental()
 *  called from VisionCore::ProcessImage(int runtime_ms, int ca, int co)
 */
void FormMotionField::OperateNonIncremental()
{
	StartRunTime();
	if (!firstCall) CalculateFocusOfExpansion();
	AgeTObjects(maxAge);				// age all the tracked objects
	firstCall = false;
	StopRunTime();
}


/**
 * @brief Saves the new received L-Junction for tracking
 */
void FormMotionField::SaveLJunction(unsigned idx)
{
	// Get properties of the L-Junction
	LJctDef ld;
	ld.type = Gestalt::L_JUNCTION;

	ld.line[0] = LJunctions(idx)->line[0];
	ld.line[1] = LJunctions(idx)->line[1];

	ld.dir[0] = LJunctions(idx)->dir[0];
	ld.dir[1] = LJunctions(idx)->dir[1];

	ld.isct.x = LJunctions(idx)->isct.x;
	ld.isct.y = LJunctions(idx)->isct.y;

	ld.openingAngle = LJunctions(idx)->OpeningAngle();

	// save cylinder for tracking
	tLJcts.PushBack(new TLJct(idx, ld));
}

/**
 * @brief Track the new received L-Junction with an L-Junction of a former image.
 */
void FormMotionField::TrackLJunction(unsigned idx)
{
	/// Calculate the distance to all L-Junctions from the last image and store it in the map
	static map<double, unsigned> sortedDistances;				// allocate space only once => static
	for(unsigned i=tLJcts.Size(); i>0; i--)
	{
		if (tLJcts[i-1]->age == 1 && tLJcts[i-1]->id_next_tracked == UNDEF_ID)
		{
			// Calculate distance between intersections and add to sortedDistances map
			double dx = fabs(LJunctions(idx)->isct.x - tLJcts[i-1]->lJctDef.isct.x);
			double dy = fabs(LJunctions(idx)->isct.y - tLJcts[i-1]->lJctDef.isct.y);
			double dis = sqrt(dx*dx + dy*dy);
			sortedDistances.insert(make_pair(dis, i-1));
		}
	}

	/// Try to find the best match (best match of both arm directions)
	map<double, unsigned>::iterator iter;
	double referenceAngle = M_PI;
	unsigned bestLJunction = UNDEF_ID;
	double distance = 0.;
	for( iter = sortedDistances.begin(); iter != sortedDistances.end(); ++iter ) 
	{
		if (iter->first > 30.) break;				/// TODO Threshold for maximum distance for tracking!

		// Find the best match of both arm directions
		double alpha = acos(Dot(tLJcts[iter->second]->lJctDef.dir[0], LJunctions(idx)->dir[0]));
		double beta = acos(Dot(tLJcts[iter->second]->lJctDef.dir[1], LJunctions(idx)->dir[1]));

		if((alpha + beta) < referenceAngle)
		{
			bestLJunction = iter->second;
			referenceAngle = alpha + beta;
			distance = iter->first;
		}
// 		cout << "L-Jct[" << iter->second <<"]: " << tLJcts[iter->second]->id << " alpha/beta: " << alpha << " / " << beta << endl;
	}

	/// TODO TODO TODO TODO TODO TODO TODO TODO TODO Vergrößern des Tracking Raums: d.h. age > 1
	/// Rückwärtsverfolgung der L-Junction durch das Feld. Dazu muss zuerst ein Rückwärtszeiger durch das Feld TLJct eingetragen werden
	/// Eintragen von id_next_tracked (idx) in tLJcts mit id = tLJcts[bestLJunction]->id
	/// Eintragen von id_former_tracked (tLJcts[bestLJunction]->id) in tLJcts mit id = idx
	/// Gibt es id_former_tracked von tLJcts[bestLJunction]->id mit age = 2
	/// 	Und davon wieder (mit age = 3) usw.
	/// Berechnen des Verschiebungsvektors für age > 1
	/// Verlängerten Verschiebungsvektor in LJctDef-> aufnehmen.
	/// Darstellung in MotionFieldElements implementieren
	/// In Berechnung des FOE einbeziehen ???

	// Calculate significance value
	double sig = referenceAngle * distance;

	// Save new element vector if reference difference of the angle is smaller than 0.3 rad.
	if(referenceAngle < 0.3)
	{
		// Create only a motion field element, if length is not 0.
		/// TODO Set Threshold for minimum length of motion vector
		if ((LJunctions(idx)->isct - tLJcts[bestLJunction]->lJctDef.isct).Length() > 0.1)			
			NewGestalt(new MotionFieldElement(idx, tLJcts[bestLJunction]->id, LJunctions(idx)->isct, tLJcts[bestLJunction]->lJctDef.isct, sig));
// 		else
// 			cout << "FormMotionField::TrackLJunction: Motion vector with zero length." << endl;


/// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
/// Create Tracked L-Junction => Hilft nachher für Tracking von Rectangles (Hypotheses generation ????)

// 		if ((LJunctions(idx)->isct - tLJcts[bestLJunction]->lJctDef.isct).Length() <= 0.1)
// 			NewGestalt(new MotionFieldElement(idx, tLJcts[bestLJunction]->id, LJunctions(idx)->isct, tLJcts[bestLJunction]->lJctDef.isct, sig));
// 		else
// 			cout << "FormMotionField::TrackLJunction: Motion vector with zero length." << endl;
	}

	// clear sorted distances map
	sortedDistances.clear();
}


/**
 * @brief Calculate the new motion field hypotheses.
 * @TODO Berechnet nur die meanDirection und meanLength der getrackten Junctions
 */
void FormMotionField::CreateNewMotionField()
{
	if (NumMotionFieldElements() != 0)
	{
		Vector2 fieldDirection;				// Mean direction of of the field
		double meanLength = 0.;				// Mean length of the motion field vectors
	
		fieldDirection.x = 0.; fieldDirection.y = 0.;
		for(unsigned i=0; i<NumMotionFieldElements(); i++)
		{
			fieldDirection += MotionFieldElements(i)->dir;
			meanLength += MotionFieldElements(i)->length;
		}
	
		meanLength /= NumMotionFieldElements();
		if (fieldDirection.Norm() !=0) fieldDirection.Normalise();
	}

// printf("Motion Direction: %3.2f - %3.2f	meanLength: %3.2f\n", fieldDirection.x, fieldDirection.y, meanLength);
}


/**
 * @brief Calculate the focus of expansion (FOE).
 * @TODO Calculate the mean direction of the field => wird 2x berechnet => Überprüfen
 */
void FormMotionField::CalculateFocusOfExpansion()
{
	if (NumMotionFieldElements() == 0)
	{
		printf("FormMotionField::CalculateFocusOfExpansion: No Motion Field Vectors!\n");
		return;
	}

	FOE.x = 0.; FOE.y = 0.; sigFOE = 0.; expFOE = 0;
	meanDirection.x = 0.; meanDirection.y = 0.;

	// Get all element vectors and fill the array
	Vector2 newElementVectors[NumMotionFieldElements()][2];
	for(unsigned i=0; i<NumMotionFieldElements(); i++)
	{
		newElementVectors[i][0] = MotionFieldElements(i)->point[0];
		newElementVectors[i][1] = MotionFieldElements(i)->point[1];
	}

	/// calculate mean direction and mask poor results
	/// TODO Schlechte Berechnung: einige schlechte Ergebnisse können überwiegen, da die Länge der Vektoren miteinfließt
	/// TODO Es können Vektoren mit Länge 0 auftreten! Was dann?
	for(unsigned i=0; i<NumMotionFieldElements(); i++)
	{
		Vector2 dirVector = newElementVectors[i][1] - newElementVectors[i][0];
		if(dirVector.Norm() != 0.) 
		{
			dirVector = Normalise(dirVector);
// printf("DirVector: %4.2f - %4.2f\n", dirVector.x, dirVector.y);
			meanDirection += dirVector;
		}
	}

//  printf("meanDirection: %4.2f / %4.2f\n", meanDirection.x, meanDirection.y);

	meanDirection.x = meanDirection.x / NumMotionFieldElements();
	meanDirection.y = meanDirection.y / NumMotionFieldElements();
	Mask();											// mask the bad results, using the mean direction.

//  printf("meanDirection: %4.2f / %4.2f\n", meanDirection.x, meanDirection.y);
	double angle = PolarAngle(meanDirection);
// printf("Angle: %4.3f / degree: %4.3f\n", angle, angle*180/M_PI);

// cout << "angle: " << angle << endl;

	/// calculate all intersections between element vectors
	vector<Vector2> FOE_Points;
	unsigned motionCase = 0;			// 1=forward / 2=backward / 3=right / 4=left
	for(unsigned i=0; i<NumMotionFieldElements(); i++)
	{
		for(unsigned j=i; j<NumMotionFieldElements(); j++)
		{
			/// TODO TODO TODO use only vector pairs, whith more than 50 Pixel distance
			if((newElementVectors[i][0] - newElementVectors[j][0]).Length() > 50.)
			{
				Vector2 dirVectorI, dirVectorJ; 

				dirVectorI = newElementVectors[i][1] - newElementVectors[i][0];
				if(dirVectorI.Norm() != 0.) dirVectorI = Normalise(dirVectorI);
				else cout << "Enormes Problem gefunden => Sofort handeln" << endl;

				dirVectorJ = newElementVectors[j][1] - newElementVectors[j][0];
				if(dirVectorJ.Norm() != 0.) dirVectorJ = Normalise(dirVectorJ);
				else cout << "Enormes Problem gefunden => Sofort handeln" << endl;

				/// Estimate the intersection points (FOE_Points) for the different motion cases
				/// Consider the side, on which the connection should be (with l1 and l2) 
				/// Rotate the vectors (90 degree) for left and right motion, to calculate the FOE
				if (dirVectorI != dirVectorJ)
				{	
					try
					{
						double l1, l2;
						if(angle > M_PI/6. && angle < M_PI*5./6.)									/// CASE: Motion forward
						{
							Vector2 FOE = LineIntersection(newElementVectors[i][0], dirVectorI, newElementVectors[j][0], dirVectorJ, &l1, &l2);
							if(l1 < 0. && l2 < 0.)
								FOE_Points.push_back(FOE);
							motionCase = 1;
						}
						if(angle > -M_PI*5./6. && angle < -M_PI/6.)								/// CASE: Motion backward
						{
							Vector2 FOE = LineIntersection(newElementVectors[i][0], dirVectorI, newElementVectors[j][0], dirVectorJ, &l1, &l2);
							if(l1 > 0. && l2 > 0.)
								FOE_Points.push_back(FOE);
							motionCase = 2;
						}
						if(angle > M_PI*5./6. || angle < -M_PI*5./6.)							/// CASE: Motion right
						{
							// Rotate the motion element vectors
							Vector2 eVI = Rotate(dirVectorI, -M_PI/2.);
							Vector2 eVJ = Rotate(dirVectorJ, -M_PI/2.);
							Vector2 FOE = LineIntersection(newElementVectors[i][0], eVI, newElementVectors[j][0], eVJ, &l1, &l2);
							if(l1 > 0. && l2 > 0.)
								FOE_Points.push_back(FOE);
							motionCase = 3;
						}
						if(angle > -M_PI/6. && angle < M_PI/6.)										/// CASE: Motion left
						{
							// Rotate the motion element vectors
							Vector2 eVI = Rotate(dirVectorI, M_PI/2.);
							Vector2 eVJ = Rotate(dirVectorJ, M_PI/2.);
							Vector2 FOE = LineIntersection(newElementVectors[i][0], eVI, newElementVectors[j][0], eVJ, &l1, &l2);
							if(l1 > 0. && l2 > 0.)
								FOE_Points.push_back(FOE);
							motionCase = 4;
						}
					}
					catch (Except &e)
					{
/*						cout << "FormMotionField::CalculateFocusOfExpansion: LineIntersection Exception." << endl;
						cout << "vectorI: " << newElementVectors[i][0].x << " - " << newElementVectors[i][0].y << " || " << newElementVectors[i][1].x << " - " << newElementVectors[j][1].y << endl;
						cout << "direction: " << dirVectorI.x << " - " << dirVectorI.y << endl;

						cout << "vectorJ: " << newElementVectors[j][0].x << " - " << newElementVectors[j][0].y << " || " << newElementVectors[j][1].x << " - " << newElementVectors[j][1].y << endl;
						cout << "direction: " << dirVectorJ.x << " - " << dirVectorJ.y << endl;*/
					}
				}
			}
		}
	}

// cout << "motionCase: " << motionCase << endl;

// printf("\nNumber of FOE-Points: %u\n", FOE_Points.size());

	/// Calculate the FOE point for all motion fields
	if(FOE_Points.size() > 0)
	{
		/// Get the values around the median value of x and y and calculate mean value of FOE
		map<double, Vector2> FOE_MedianPoints_x;
		for(unsigned i=0; i<=FOE_Points.size(); i++)
			FOE_MedianPoints_x[FOE_Points[i].x] = FOE_Points[i];
	
		map<double, Vector2> FOE_MedianPoints_y;
		for(unsigned i=0; i<=FOE_Points.size(); i++)
			FOE_MedianPoints_y[FOE_Points[i].y] = FOE_Points[i];
	
		// consider the median values for the x-variable
		map<double, Vector2>::iterator iter, iter_begin, iter_end;
		iter_begin = FOE_MedianPoints_x.begin();
		for(unsigned j=0; j<FOE_Points.size()/3; j++) iter_begin++;
		iter_end = FOE_MedianPoints_x.begin();
		for(unsigned j=0; j<FOE_Points.size()*2/3; j++) iter_end++;
		for( iter=iter_begin; iter != iter_end; ++iter ) 
			FOE.x += iter->second.x;
	
		// consider the median values for the y-variable
		iter_begin = FOE_MedianPoints_y.begin();
		for(unsigned j=0; j<FOE_Points.size()/3; j++) iter_begin++;
		iter_end = FOE_MedianPoints_y.begin();
		for(unsigned j=0; j<FOE_Points.size()*2/3; j++) iter_end++;
		for( iter=iter_begin; iter != iter_end; ++iter ) 
			FOE.y += iter->second.y;
	
		FOE /= FOE_Points.size()/3;
	}

	/// Calculate the FOE motion vectors in respect to the distance
	if(FOE_Points.size() > 0)
	{
		int elementCounter = 0, gElementCounter = 0;
		double disMultiplicator = 0;

		/// Nehme alle Motion Field Vectors
		for(unsigned i=0; i<NumMotionFieldElements(); i++)
		{
			if(!MotionFieldElements(i)->IsMasked())
			{
				elementCounter++;

				/// Calculate direction from FOE-point to Element vertex.
				Vector2 foeDirection = MotionFieldElements(i)->point[1] - FOE;
				if(foeDirection.y != 0) foeDirection.Normalise();
	
				/// Calculate opening angle between FOE direction and Element direction.
				double openingAngle = acos(Dot(foeDirection, MotionFieldElements(i)->dir));

				if(openingAngle < M_PI/6. && motionCase == 1)														// Motion forward
				{
					gElementCounter++;
					sigFOE += openingAngle;
					double distance = (MotionFieldElements(i)->point[1] - FOE).Length();
					if (distance != 0.) disMultiplicator += (MotionFieldElements(i)->length / distance);
				}
				if(fabs(openingAngle-M_PI) < M_PI/6. && motionCase == 2)								// Motion backward
				{
					gElementCounter++;
					sigFOE += fabs(openingAngle-M_PI);
					double distance = (MotionFieldElements(i)->point[1] - FOE).Length();
					if (distance != 0.) disMultiplicator += (MotionFieldElements(i)->length / distance);
				}
				if(fabs(openingAngle - (M_PI/2.)) < M_PI/6. && motionCase == 3)					// Motion right
				{
					gElementCounter++;
					sigFOE += fabs(openingAngle - (M_PI/2.));
					double distance = (MotionFieldElements(i)->point[1] - FOE).Length();
					if (distance != 0.) disMultiplicator += (MotionFieldElements(i)->length / distance);
				}
				if(fabs(openingAngle - (M_PI/2.)) < M_PI/6. && motionCase == 4)					// Motion left
				{
					gElementCounter++;
					sigFOE += fabs(openingAngle - (M_PI/2.));
					double distance = (MotionFieldElements(i)->point[1] - FOE).Length();
					if (distance != 0.) disMultiplicator += (MotionFieldElements(i)->length / distance);
				}
			}
		}

		/// Calculate distance multiplicator value and significance for FOE
		if (elementCounter != 0) disMultiplicator /= elementCounter;
		if (sigFOE != 0) sigFOE = elementCounter / sigFOE;

// printf("FOE: %4.2f / %4.2f\n", FOE.x, FOE.y);
// cout << "distance value: " << disMultiplicator << " || sig: " << sigFOE << endl;

		/// Set the FOE vector for all MotionField elements (only for displaying)
		for(unsigned i=0; i<NumMotionFieldElements(); i++)
			MotionFieldElements(i)->SetFOE(motionCase, FOE, disMultiplicator, sigFOE);

		/// Save motion field
		MotionDef md;
		md.motionCase = motionCase;
		md.FOE = FOE;
		md.disMul = disMultiplicator;
		md.sigFOE = sigFOE;

		motField.PushBack(new TMotionField(0, md));

		/// Create new Gestalt MotionField
		CreateGestaltMotionField();
	}
}

/**
 * @brief Create new motion field. Deliver also the old motion fields.
 */
void FormMotionField::CreateGestaltMotionField()
{
// 	NewGestalt(new MotionField(motionCase, FOE, disMultiplicator, sigFOE));
	for(unsigned i=0; i<motField.Size(); i++)
	{
		NewGestalt(new MotionField(motField[i]->motDef.motionCase, motField[i]->motDef.FOE, motField[i]->motDef.disMul, motField[i]->motDef.sigFOE, motField[i]->age));
	}

}

}



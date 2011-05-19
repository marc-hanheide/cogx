/**
 * @file ObjectTracker.cc
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Tracking of detected objects
 **/

#include <math.h>
#include "Math.hh"
#include "ObjectTracker.hh"
#include "Object.hh"
#include "VisionCore.hh"
#include "LJunction.hh"
#include "Line.hh"
#include "Ellipse.hh"
#include "Ball.hh"
#include "ExtEllipse.hh"
#include "Rectangle.hh"
#include "Flap.hh"
#include "Cube.hh"
#include "Cone.hh"
#include "Ellipse.hh"
#include "Wall.hh"
#include "Exit.hh"
#include "TrktFlap.hh"
#include "TrktCube.hh"

namespace Z
{

/**
 * @brief Constructor of class TCube 
 * @param o_id Object id
 * @param i Original ID from the Gestalt
 * @param cd Properties of the cube (class CubeDefinition)
 */
ObjectTracker::TCube::TCube(unsigned o_id, unsigned i, CubeDef cd)
{
  object_id = o_id; 
  id = i;
  age = 0;

	cubeDef = cd;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void ObjectTracker::TCube::FormerTracked(unsigned i)
{
  id_former_tracked = i;
}

void ObjectTracker::TCube::NextTracked(unsigned id)
{
	id_next_tracked = id;
}

/**
 * @brief Constructor of class TCone 
 * @param o_id Object id
 * @param i Original ID from the Gestalt
 * @param cd Properties of the cone (class ConeDefinition)
 */
ObjectTracker::TCone::TCone(unsigned o_id, unsigned i, ConeDef cd)
{
	object_id = o_id;
  id = i;
  age = 0;

	coneDef = cd;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void ObjectTracker::TCone::FormerTracked(unsigned i)
{
  id_former_tracked = i;
}

void ObjectTracker::TCone::NextTracked(unsigned id)
{
	id_next_tracked = id;
}

/**
 * @brief Constructor of class TCylinder 
 * @param o_id Object id
 * @param i Original ID from the Gestalt
 * @param cd Properties of the cylinder (class CylinderDefinition)
 */
ObjectTracker::TCylinder::TCylinder(unsigned o_id, unsigned i, CylDef cd)
{
	object_id = o_id;
  id = i;
  age = 0;

	cylDef = cd;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void ObjectTracker::TCylinder::FormerTracked(unsigned i)
{
  id_former_tracked = i;
}

void ObjectTracker::TCylinder::NextTracked(unsigned id)
{
	id_next_tracked = id;
}

/**
 * @brief Constructor of class TBall 
 * @param o_id Object id
 * @param i Original ID from the Gestalt
 * @param bd Properties of the ball (class BallDefinition)
 */
ObjectTracker::TBall::TBall(unsigned o_id, unsigned i, BalDef bd)
{
	object_id = o_id;
  id = i;
  age = 0;

	balDef = bd;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void ObjectTracker::TBall::FormerTracked(unsigned i)
{
  id_former_tracked = i;
}

void ObjectTracker::TBall::NextTracked(unsigned id)
{
	id_next_tracked = id;
}

/**
 * @brief Constructor of class TWall
 * @param o_id Object id
 * @param i Original ID from the Gestalt
 * @param wd Properties of the wall (class WallDefinition)
 */
ObjectTracker::TWall::TWall(unsigned o_id, unsigned i, WalDef wd)
{
	object_id = o_id;
  id = i;
  age = 0;

	walDef = wd;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void ObjectTracker::TWall::FormerTracked(unsigned i)
{
  id_former_tracked = i;
}

void ObjectTracker::TWall::NextTracked(unsigned id)
{
	id_next_tracked = id;
}

/**
 * @brief Constructor of class TExit 
 * @param o_id Object id
 * @param i Original ID from the Gestalt
 * @param ed Properties of the exit (class ExitDefinition)
 */
ObjectTracker::TExit::TExit(unsigned o_id, unsigned i, ExitDef ed)
{
	object_id = o_id;
  id = i;
  age = 0;

	exitDef = ed;

  id_former_tracked = UNDEF_ID;
	id_next_tracked = UNDEF_ID;
}

void ObjectTracker::TExit::FormerTracked(unsigned i)
{
  id_former_tracked = i;
}

void ObjectTracker::TExit::NextTracked(unsigned id)
{
	id_next_tracked = id;
}

/**
 *	@brief Constructor of class ObjectTracker
 */
ObjectTracker::ObjectTracker(Config *cfg) : GestaltPrinciple(cfg)
{
  firstCall = true;						// first call of ObjectTracker
	maxAge = 2;									// maximum age for tracking

	getCamParamsFromVC = true;	// get camera parameters from vision core params

  trackCubes = false;
	trackFlap2Cube = false;
  trackRect2Cube = false;
	
  trackCones = false;
  trackCylinders = false;
	trackBalls = false;

	printRect = false;
	printTrackCylinder = false;
	printTrackBall = false;
	printCyl = false;
	printCheckTrDi = false;
}

/**
 *	@brief Increase age of all objects (step counter) and erase if one is older than maxAge
 *	@param maxAge Maximum of age befor object will be deleted
 */
void ObjectTracker::AgeTObjects(unsigned maxAge)
{
  AgeTCubes(maxAge);
  AgeTCones(maxAge);
  AgeTCylinders(maxAge);
	AgeTBalls(maxAge);
	AgeTWalls(maxAge);
	AgeTExits(maxAge);
}

/**
 * @brief Increment age of tCubes and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void ObjectTracker::AgeTCubes(unsigned maxAge)
{
  for(unsigned i=tCubes.Size(); i>0; i--)
  {
     tCubes[i-1]->age++;
     if (tCubes[i-1]->age > maxAge) tCubes.Erase(i-1);
  }
}

/**
 * @brief Increment age of tCones and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void ObjectTracker::AgeTCones(unsigned maxAge)
{
  for(unsigned i=tCones.Size(); i>0; i--)
  {
     tCones[i-1]->age++;
     if (tCones[i-1]->age > maxAge) tCones.Erase(i-1);
  }
}

/**
 * @brief Increment age of tCylinders and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void ObjectTracker::AgeTCylinders(unsigned maxAge)
{
  for(unsigned i=tCylinders.Size(); i>0; i--)
  {
     tCylinders[i-1]->age++;
     if (tCylinders[i-1]->age > maxAge) tCylinders.Erase(i-1);
  }
}

/**
 * @brief Increment age of tBalls and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void ObjectTracker::AgeTBalls(unsigned maxAge)
{
  for(unsigned i=tBalls.Size(); i>0; i--)
  {
     tBalls[i-1]->age++;
     if (tBalls[i-1]->age > maxAge) tBalls.Erase(i-1);
  }
}

/**
 * @brief Increment age of tWalls and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void ObjectTracker::AgeTWalls(unsigned maxAge)
{
  for(unsigned i=tWalls.Size(); i>0; i--)
  {
     tWalls[i-1]->age++;
     if (tWalls[i-1]->age > maxAge) tWalls.Erase(i-1);
  }
}

/**
 * @brief Increment age of tExits and delete if they are older than maxAge steps
 * @param maxAge Maximum of Age befor object will be deleted
 */
void ObjectTracker::AgeTExits(unsigned maxAge)
{
  for(unsigned i=tExits.Size(); i>0; i--)
  {
     tExits[i-1]->age++;
     if (tExits[i-1]->age > maxAge) tExits.Erase(i-1);
  }
}

/**
 *	@brief Mask objects, which have the center point inside the radius of another object.
 *	The function considers only cubes, cones, cylinders and balls.
 *	TODO First come, first serve!!!
 */
/*void ObjectTracker::Mask()
{
  for(unsigned i=0; i<NumObjects(); i++)
  {
		for(unsigned j=i+1; j<NumObjects(); j++)
		{
			if(!Objects(i)->IsMasked() && !Objects(j)->IsMasked())
			{
				if(Objects(j)->IsInside(i))
				{
					Objects(j)->Mask(i);

					// find tObject and set age to maxAge. => damit sie gelöscht werden?
					if(Objects(j)->type == Gestalt::CUBE || Objects(j)->type == Gestalt::TRACKEDCUBE)
						for(unsigned k=0; k<tCubes.Size(); k++) 
							if (tCubes[k]->object_id == j) tCubes[k]->age = maxAge;
					if(Objects(j)->type == Gestalt::CONE || Objects(j)->type == Gestalt::TRACKEDCONE)
						for(unsigned k=0; k<tCones.Size(); k++) 
							if (tCones[k]->object_id == j) tCones[k]->age = maxAge;
					if(Objects(j)->type == Gestalt::CYLINDER || Objects(j)->type == Gestalt::TRACKEDCYLINDER)
						for(unsigned k=0; k<tCylinders.Size(); k++) 
							if (tCylinders[k]->object_id == j) tCylinders[k]->age = maxAge;
					if(Objects(j)->type == Gestalt::BALL || Objects(j)->type == Gestalt::TRACKEDBALL)
						for(unsigned k=0; k<tBalls.Size(); k++) 
							if (tBalls[k]->object_id == j) tBalls[k]->age = maxAge;
				}
			}
		}
  }
}*/
/// Cubes sind immer besser als Tracked Cubes
/// sonst entscheidet significance
void ObjectTracker::Mask()
{
  for(unsigned i=0; i<NumObjects(); i++)								// mask the same Gestalts with significance value
  {
		for(unsigned j=0; j<NumObjects(); j++)
		{
			if(!Objects(i)->IsMasked() && !Objects(j)->IsMasked())
			if(Objects(i)->sig < Objects(j)->sig)
			{
// 				if(Objects(j)->type == Objects(i)->type && Objects(i)->IsInside(j))
				if(Objects(i)->IsInside(j))
 				{	
					Objects(i)->Mask(j);		 
				}
			}		  
		}
  }
//   for(unsigned i=0; i<NumObjects(); i++)								// mask => first come, first serve
//   {
// 		for(unsigned j=i+1; j<NumObjects(); j++)
// 		{
// 			if(!Objects(i)->IsMasked() && !Objects(j)->IsMasked())
// 			{
// 				if(Objects(j)->IsInside(i))
// 				{
// 					Objects(j)->Mask(i);
// 				}
// 			}
// 		}
// 	}
}


/**
 *	@brief Age the masked objects, so that they can not be tracked in the next image.
 */
void ObjectTracker::AgeMasked()
{
 	for(unsigned i=0; i<NumObjects(); i++)
 		if(Objects(i)->masked != UNDEF_ID)
 		{
			for(unsigned j=0; j<tCubes.Size(); j++)						// tCubes
			{
				if(tCubes[j]->id == Objects(i)->gestalt_id)
				{
					tCubes[j]->age = maxAge;
				}
			}
			for(unsigned j=0; j<tCones.Size(); j++)						// tCones
			{
				if(tCones[j]->id == Objects(i)->gestalt_id)
				{
					tCones[j]->age = maxAge;
				}
			}
			for(unsigned j=0; j<tCylinders.Size(); j++)				// tCylinders
			{
				if(tCylinders[j]->id == Objects(i)->gestalt_id)
				{
					tCylinders[j]->age = maxAge;
				}
			}
		}
}

/**
 *	@brief InformNewGestalt()
 *	@param type Gestalt type
 *	@param idx Gestalt index
 */
void ObjectTracker::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
}

/**
 *	@brief Initialisation of the camera parameters (intrinsic/extrinsic matrix) to get 3D positions from an 2D image
 */
void ObjectTracker::InitCamModel()
{
	confFile = "subarchitectures/vision.sa/src/c++/vision/components/ObjectDetector/cfg/calibration.xml";
//	confFile = "../cfg/calibration.xml"; 			// configuration file of camera parameters for mxTools

  m_tImgSize.width = VisionCore::IW();
  m_tImgSize.height = VisionCore::IH();

	// get camera parameters from vision core
	if(getCamParamsFromVC)
	{
		VisionCore::GetCamModel(m_cCamModel);
		m_cCamModel.Save("subarchitectures/vision.sa/src/c++/vision/components/ObjectDetector/cfg/calRef.xml");
	}
	else
	{
		printf("ObjectTracker::InitCamModel: Load camera configuration from calibration.xml file.\n");
		m_cCamModel.LoadIntrinsic (confFile.data());
		m_cCamModel.LoadDistortion (confFile.data());
		m_cCamModel.LoadReferencePointsToComputeExtrinsic(confFile.data());
		m_cCamModel.Save("subarchitectures/vision.sa/src/c++/vision/components/ObjectDetector/cfg/ExtrinsicFromCalibration.xml");
	}
	m_cCamModel.PreProcessing (m_tImgSize);

}

/**
 *	@brief OperateNonIncremental()
 *  called from VisionCore::ProcessImage(int runtime_ms, int ca, int co)
 */
void ObjectTracker::OperateNonIncremental()
{
  StartRunTime();
//printf("ObjectTracker() - ");

	// TODO Initialisation of the camera model: Changing parameters!!!
  if (firstCall)
	{	
		InitCamModel();
		firstCall = false;
	}

 	maxTrackDist = 25.;															// standard maximum is 25 pixel
// 	meanTrackDist = 0.;
// 	meanTrackDir.x = 0.;
// 	meanTrackDir.y = 0.;
// 	maxDirDiff = 0.;
// 	trackDist.Clear();
// 	vTrackDist.Clear();

	maxTrackDist3D = 25.;														// standard maximum is 25 pixel
	meanTrackDist3D = 0.;
	meanTrackDir3D.x = 0.;
	meanTrackDir3D.y = 0.;
	maxDirDiff3D = 0.;
	trackDist3D.Clear();
	vTrackDist3D.Clear();


  AgeTObjects(maxAge);														// increment age of the Objects

//printf("######################################################\n");

	/// get all objects cubes and the 3D properties of the objects
	GetCubes();
	GetCones();
	GetCylinders();
	GetBalls();
	GetWalls();
	GetExits();

// printf("track2Fehler 0\n");
	// track objects with existing objects
  if(trackCubes) TrackCube();											// Try to track the cubes from tCubes
if(printCheckTrDi) printf("maxTrackDist3D after cube: %4.2f - meanTrackDist3D: %4.2f - meanTrackDir3D: %4.3f - %4.3f - maxDirDiff3D: %4.3f\n",
		maxTrackDist3D, meanTrackDist3D, meanTrackDir3D.x, meanTrackDir3D.y, maxDirDiff3D);
// printf("track2Fehler 1\n");

  if(trackCones) TrackCone();											// Try to track the cones from tCones
if(printCheckTrDi) printf("maxTrackDist3D after cube: %4.2f - meanTrackDist3D: %4.2f - meanTrackDir3D: %4.3f - %4.3f - maxDirDiff3D: %4.3f\n",
		maxTrackDist3D, meanTrackDist3D, meanTrackDir3D.x, meanTrackDir3D.y, maxDirDiff3D);
// printf("track2Fehler 2\n");

  if(trackCylinders) TrackCylinder();							// Try to track the cylinder from tCylinder
if(printCheckTrDi) printf("maxTrackDist3D after cube: %4.2f - meanTrackDist3D: %4.2f - meanTrackDir3D: %4.3f - %4.3f - maxDirDiff3D: %4.3f\n",
		maxTrackDist3D, meanTrackDist3D, meanTrackDir3D.x, meanTrackDir3D.y, maxDirDiff3D);
// printf("track2Fehler 3\n");

  if(trackBalls) TrackBall();							// Try to track the ball from tBall
// if(printCheckTrDi) printf("maxTrackDist3D after cube: %4.2f - meanTrackDist3D: %4.2f - meanTrackDir3D: %4.3f - %4.3f - maxDirDiff3D: %4.3f\n",
// 		maxTrackDist3D, meanTrackDist3D, meanTrackDir3D.x, meanTrackDir3D.y, maxDirDiff3D);
// printf("track2Fehler 3\n");

  if(trackCylinders) TrackWall();							// Try to track the wall from tWall
if(printCheckTrDi) printf("maxTrackDist3D after cube: %4.2f - meanTrackDist3D: %4.2f - meanTrackDir3D: %4.3f - %4.3f - maxDirDiff3D: %4.3f\n",
		maxTrackDist3D, meanTrackDist3D, meanTrackDir3D.x, meanTrackDir3D.y, maxDirDiff3D);
// printf("track2Fehler 4\n");

  if(trackCylinders) TrackExit();							// Try to track the exit from tExit
if(printCheckTrDi) printf("maxTrackDist3D after cube: %4.2f - meanTrackDist3D: %4.2f - meanTrackDir3D: %4.3f - %4.3f - maxDirDiff3D: %4.3f\n",
		maxTrackDist3D, meanTrackDist3D, meanTrackDir3D.x, meanTrackDir3D.y, maxDirDiff3D);
// printf("track2Fehler - no\n");

	// track objects with underlying gestalts
	if(trackFlap2Cube) TrackFlap2Cube();						// Try to track cubes from flaps
// printf("track2Fehler 11\n");
	if(trackRect2Cube) TrackRect2Cube();						// Try to track cubes from rectangles
// printf("track2Fehler 12\n");
  if(trackCones) TrackEllipse2Cone();							// Try to track cones from ellipses
// printf("track2Fehler 13\n");
	if(trackCylinders) TrackEllipse2Cylinder();			// Try to track cylinders from ellipses
// printf("track2Fehler 14\n");

	/// TODO print all tCubes
	//PrintTCubes();
	//PrintTCylinders();
	//ShowTCubes();

	Mask();
	AgeMasked();

  StopRunTime();
}


/**
 *	@brief Get all cubes, which are not masked and estimate the properties (also 3D)
 */
void ObjectTracker::GetCubes()
{
  for(unsigned i=0; i<NumCubes(); i++)
  {
		if (Cubes(i)->masked < 10000 || Cubes(i)->masked == UNDEF_ID)					/// TODO Cubes wich are masked > 10000 are geometric false cubes.
		{
			CubeDef cd;

			cd.sig = Cubes(i)->sig;

			cd.corner_points[0][0] = Cubes(i)->corner_points[0][0];
			cd.corner_points[0][1] = Cubes(i)->corner_points[0][1];
			cd.corner_points[1][0] = Cubes(i)->corner_points[1][0];
			cd.corner_points[1][1] = Cubes(i)->corner_points[1][1];
			cd.corner_points[2][0] = Cubes(i)->corner_points[2][0];
			cd.corner_points[2][1] = Cubes(i)->corner_points[2][1];
			cd.corner_points[3][0] = Cubes(i)->corner_points[3][0];
			cd.corner_points[3][1] = Cubes(i)->corner_points[3][1];

			// get the cube-properties for the cube
			GetCubeProperties(cd);

			// save cube for tracking
			TCube *c = new TCube(NumObjects(), i, cd);
			tCubes.PushBack(c);

			// create new object from cube
			NewGestalt(new Object(Gestalt::CUBE, i, cd));
		}
  }

	/// TODO get hypothesized cubes!!!
  for(unsigned i=0; i<NumTrktCubes(); i++)
  {
		if (TrktCubes(i)->masked == UNDEF_ID && TrktCubes(i)->tCubes[0].hypothesised)
		{
			CubeDef cd;

			/// TODO TODO TODO TODO Significance of tracked cube is 20% lower?
			/// Tracked cubes müssen immer besser sein, als hypothesierte cubes!!! (80% und 50%)
			if(TrktCubes(i)->tCubes[0].hypothesised) cd.sig = TrktCubes(i)->sig * 0.5;
			else cd.sig = TrktCubes(i)->sig*0.8;

			cd.corner_points[0][0] = TrktCubes(i)->tCubes[0].corner_points[0][0];
			cd.corner_points[0][1] = TrktCubes(i)->tCubes[0].corner_points[0][1];
			cd.corner_points[1][0] = TrktCubes(i)->tCubes[0].corner_points[1][0];
			cd.corner_points[1][1] = TrktCubes(i)->tCubes[0].corner_points[1][1];
			cd.corner_points[2][0] = TrktCubes(i)->tCubes[0].corner_points[2][0];
			cd.corner_points[2][1] = TrktCubes(i)->tCubes[0].corner_points[2][1];
			cd.corner_points[3][0] = TrktCubes(i)->tCubes[0].corner_points[3][0];
			cd.corner_points[3][1] = TrktCubes(i)->tCubes[0].corner_points[3][1];

			/// TODO
			// Recalculate3DPosition(cd);

			// get the cube-properties for the cube
			RecalculateCube3DProperties(i, cd);
			GetCubeProperties(cd);

			// save cube for tracking
//			TCube *c = new TCube(NumObjects(), i, cd);
//			tCubes.PushBack(c);

			// create new object from cube
			NewGestalt(new Object(Gestalt::TRACKEDCUBE, TrktCubes(i)->tCubeIDs[0], cd));
		}
	}
}



/**
 *	@brief Recalculate the 3D properties of an tracked cube
 *	Estimate all properties of a cube in respect to the camera parameters
 */
void ObjectTracker::RecalculateCube3DProperties(unsigned trktCube, CubeDef &cd)
{
bool print = false;

if (print) 	printf("######## Recalculate Cube 3D Properties ############\n");
	/// Welches Flap?
	unsigned trktFlapID = TrktCubes(trktCube)->tCubes[0].flap;		// id of the flap (posible hypothesized)
if (print) printf("  trktFlapID: %u\n", trktFlapID);
	unsigned trktFlap = UNDEF_ID;																	// Nr in TrktFlap
	for(unsigned i=0; i<NumTrktFlaps(); i++)
	{
if (print) printf("TrktFlap from flap: %u\n", TrktFlaps(i)->tFlapIDs[0]);
		if (TrktFlaps(i)->tFlapIDs[0] == trktFlapID)
		{
if (print) 			printf("Tracked Flap ID found!!!\n");
			trktFlap = i;
		}
	}
if (print) 	printf("  Flap: %u\n", trktFlap);

	/// Untersten 2 Punkte 
	map<double, unsigned> jctsIsct;
	jctsIsct[TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[0].y] = 0;
	jctsIsct[TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[2].y] = 2;
	jctsIsct[TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[0].y] = 4;
	jctsIsct[TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[1].y] = 5;
	jctsIsct[TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[2].y] = 6;
	jctsIsct[TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[3].y] = 7;

	map<double, unsigned>::iterator iter = jctsIsct.end();
	iter--;
	unsigned idx0 = (*iter).second;
	iter--;
	unsigned idx1 = (*iter).second;
	
if (print) 	printf("    Flap Iscts: %u - %u\n", idx0, idx1);

	/// wenn inner jct dabei ist, dann liegt das Flap! 
	/// wenn outer[01], flap ist links
	/// wenn outer [23], flap ist rechts
	
	Vector2 bottomPoints[2];
	if(idx0 == 0 || idx0 == 2 || idx1 == 0 || idx1 == 2)
	{
if (print) 		printf("	=> Flap liegt!!!\n");

		// Alle 6 bekannten Punkte übernehmen!
		// Alle 6 bekannten Punkte übernehmen!
		cd.corner_points[1][1] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[0];
		cd.corner_points[2][1] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[1];
		cd.corner_points[1][0] = (TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[3] + TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[1])/2;
		cd.corner_points[2][0] = (TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[2] + TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[0])/2;
		cd.corner_points[0][0] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[2];
		cd.corner_points[3][0] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[3];

		cd.corner_points[0][1] = cd.corner_points[1][1] - (cd.corner_points[1][0] - cd.corner_points[0][0]);
		cd.corner_points[3][1] = cd.corner_points[0][1] - (cd.corner_points[0][0] - cd.corner_points[3][0]);


	}
	if(idx0 == 4 || idx0 == 5 || idx1 == 4 || idx1 == 5)
	{
if (print) 		printf("	=> Flap links!!!\n");

		// Alle 6 bekannten Punkte übernehmen!
		// Alle 6 bekannten Punkte übernehmen!
		cd.corner_points[1][1] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[0];
		cd.corner_points[2][1] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[1];
		cd.corner_points[1][0] = (TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[3] + TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[1])/2;
		cd.corner_points[2][0] = (TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[2] + TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[0])/2;
		cd.corner_points[0][0] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[2];
		cd.corner_points[3][0] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[3];

		cd.corner_points[0][1] = cd.corner_points[1][1] - (cd.corner_points[1][0] - cd.corner_points[0][0]);
		cd.corner_points[3][1] = cd.corner_points[0][1] - (cd.corner_points[0][0] - cd.corner_points[3][0]);

	}
	if(idx0 == 6 || idx0 == 7 || idx1 == 6 || idx1 == 7)
	{
if (print) 		printf("	=> Flap rechts!!!\n");

		// Alle 6 bekannten Punkte übernehmen!
		cd.corner_points[1][1] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[2];
		cd.corner_points[0][1] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[3];
		cd.corner_points[1][0] = (TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[3] + TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[1])/2;
		cd.corner_points[0][0] = (TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[2] + TrktFlaps(trktFlap)->tFlaps[0].innerJctsIsct[0])/2;
		cd.corner_points[2][0] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[0];
		cd.corner_points[3][0] = TrktFlaps(trktFlap)->tFlaps[0].outerJctsIsct[1];

// 		Vector2 mean = ((cd.corner_points[1][0] - cd.corner_points[1][1]) + (cd.corner_points[1][0] - cd.corner_points[1][1]) + (cd.corner_points[1][0] - cd.corner_points[1][1]))/3.;
		cd.corner_points[2][1] = cd.corner_points[2][0] - (cd.corner_points[1][0] - cd.corner_points[1][1]);
		cd.corner_points[3][1] = cd.corner_points[0][1] - (cd.corner_points[0][0] - cd.corner_points[3][0]);
	}

	/// Umrechnung der beiden Punkte auf 3D
	Vector2 bottomPoints3D[2];

	double dAboveGround = 0.;
	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point

	tPointImg = cvPoint2D64f (cd.corner_points[1][1].x, cd.corner_points[1][1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	bottomPoints3D[0].x = tPointWorld3D.x;
	bottomPoints3D[0].y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (cd.corner_points[1][1].x, cd.corner_points[1][1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	bottomPoints3D[1].x = tPointWorld3D.x;
	bottomPoints3D[1].y = tPointWorld3D.y;

	/// Winkel zwischen den 2 Punkten?
	/// 3
}


/**
 *	@brief Estimate all properties of a cube. CubeDef must contain the corner points.
 *	@param cd Cube definition containing the properties of the defined cube
 */
void ObjectTracker::GetCubeProperties(CubeDef &cd)
{
	// cube center point
	cd.center = (cd.corner_points[0][0] + cd.corner_points[0][1] + cd.corner_points[1][1] + 
								cd.corner_points[2][1] + cd.corner_points[2][0] + cd.corner_points[3][0])/6.;

	// cube radius
	double r0 = (cd.corner_points[0][0]-cd.center).Length();
	double r1 = (cd.corner_points[0][1]-cd.center).Length();
	double r2 = (cd.corner_points[3][0]-cd.center).Length();
	double r3 = (cd.corner_points[1][1]-cd.center).Length();
	double r4 = (cd.corner_points[2][0]-cd.center).Length();
	double r5 = (cd.corner_points[2][1]-cd.center).Length();

	cd.radius = Max(r0, r1);
	cd.radius = Max(cd.radius, r2);
	cd.radius = Max(cd.radius, r3);
	cd.radius = Max(cd.radius, r4);
	cd.radius = Max(cd.radius, r5);

	// cube ground center point 2D
	cd.groundCenter = (cd.corner_points[0][1] + cd.corner_points[1][1] + cd.corner_points[2][1] + cd.corner_points[3][1])/4;

	// get 3D properties of an cube
	GetCube3DProperties(cd);
}

/**
 *	@brief Calculate the 3D properties of the cube
 *	Estimate all properties of a cube in respect to the camera parameters
 *	@param cd Cube definition structure
 */
void ObjectTracker::GetCube3DProperties(CubeDef &cd)
{
	double dAboveGround = 0.; 			// points are on ground plane

	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point

	// cube ground center point 3D
	tPointImg = cvPoint2D64f ((int) cd.groundCenter.x, (int) cd.groundCenter.y);
	m_cCamModel.Image2World (cvPoint( tPointImg), &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.groundCenter3D.x = tPointWorld3D.x;
	cd.groundCenter3D.y = tPointWorld3D.y;

	// corner point [0][1] (3D)
	tPointImg = cvPoint2D64f (cd.corner_points[0][1].x, cd.corner_points[0][1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.corner_points3D[0][1].x = tPointWorld3D.x;
	cd.corner_points3D[0][1].y = tPointWorld3D.y;

	// corner point [1][1] (3D)
	tPointImg = cvPoint2D64f (cd.corner_points[1][1].x, cd.corner_points[1][1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.corner_points3D[1][1].x = tPointWorld3D.x;
	cd.corner_points3D[1][1].y = tPointWorld3D.y;

	// corner point [2][1] (3D)
	tPointImg = cvPoint2D64f (cd.corner_points[2][1].x, cd.corner_points[2][1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.corner_points3D[2][1].x = tPointWorld3D.x;
	cd.corner_points3D[2][1].y = tPointWorld3D.y;

	// calculate dimension of the cube (3D)
	cd.length_a = (cd.corner_points3D[1][1] - cd.corner_points3D[0][1]).Norm();
	cd.length_b = (cd.corner_points3D[2][1] - cd.corner_points3D[1][1]).Norm();

	// get the orientation (3D) of the cube (also in degrees)
	cd.orientation.x = cd.corner_points3D[1][1].x - cd.groundCenter3D.x;
	cd.orientation.y = cd.corner_points3D[1][1].y - cd.groundCenter3D.y;
	cd.orientation_deg = ScaleAngle_0_pi(PolarAngle(cd.orientation));				// TODO eigentlich nicht deg sondern rad (Winkel) 

	// TODO Mean aus 31 = 01+(21-11) und 31 = 21-(11-01)
	Vector2 cp31a = cd.corner_points3D[0][1]+(cd.corner_points3D[2][1]-cd.corner_points3D[1][1]);
	Vector2 cp31b = cd.corner_points3D[2][1]-(cd.corner_points3D[1][1]-cd.corner_points3D[0][1]);
	cd.corner_points3D[3][1] = (cp31a + cp31b) / 2.;

	// corner point [1][0] (3D) (is above corner_ponit[1][1])
	tPointImg = cvPoint2D64f (cd.corner_points[1][0].x, cd.corner_points[1][0].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2YPlane (tPointWorld3D, &tPointWorld3D, cd.corner_points3D[1][1].y);
	cd.corner_points3D[1][0].x = tPointWorld3D.x;
	cd.corner_points3D[1][0].y = tPointWorld3D.y;
	cd.height = tPointWorld3D.z; 

	// corner point [0][0] (3D) (is above corner_ponit[0][1])
	tPointImg = cvPoint2D64f (cd.corner_points[0][0].x, cd.corner_points[0][0].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2YPlane (tPointWorld3D, &tPointWorld3D, cd.corner_points3D[0][1].y);
	cd.corner_points3D[0][0].x = tPointWorld3D.x;
	cd.corner_points3D[0][0].y = tPointWorld3D.y;
	cd.height += tPointWorld3D.z; 

	// corner point [2][0] (3D) (is above corner_ponit[2][1])
	tPointImg = cvPoint2D64f (cd.corner_points[2][0].x, cd.corner_points[2][0].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2YPlane (tPointWorld3D, &tPointWorld3D, cd.corner_points3D[2][1].y);
	cd.corner_points3D[2][0].x = tPointWorld3D.x;
	cd.corner_points3D[2][0].y = tPointWorld3D.y;
	cd.height += tPointWorld3D.z; 

	// corner point [3][0] (3D) (is above corner_ponit[3][1])
	tPointImg = cvPoint2D64f (cd.corner_points[3][0].x, cd.corner_points[3][0].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2YPlane (tPointWorld3D, &tPointWorld3D, cd.corner_points3D[3][1].y);
	cd.corner_points3D[3][0].x = tPointWorld3D.x;
	cd.corner_points3D[3][0].y = tPointWorld3D.y;
	cd.height += tPointWorld3D.z; 

	cd.height /= 4; // height of the cube is mean of z from the four estimated points on the top

	// cube center point
	cd.cubeCenter3D = (cd.corner_points3D[0][0] + cd.corner_points3D[1][0] + cd.corner_points3D[2][0] + cd.corner_points3D[3][0] + 
										 cd.corner_points3D[0][1] + cd.corner_points3D[1][1] + cd.corner_points3D[2][1] + cd.corner_points3D[3][1])/8.;
}

/**
 *	@brief Try to track the same cube with a cube from the last image
 */
void ObjectTracker::TrackCube()
{
/// TODO Die maximale Tracking Distanz sollte nach dem ersten getrackten Object eigentlich auch eine Richtung haben
  for(unsigned i=tCubes.Size(); i>0; i--)
  {
    if (tCubes[i-1]->age != 0) break;

    for(unsigned j=tCubes.Size(); j>0; j--)
    {
      if (tCubes[j-1]->age == 1 && tCubes[j-1]->id_next_tracked == UNDEF_ID)
      {
				double dx = tCubes[i-1]->cubeDef.center.x - tCubes[j-1]->cubeDef.center.x;
				double dy = tCubes[i-1]->cubeDef.center.y - tCubes[j-1]->cubeDef.center.y;
				double dis = sqrt(pow(dx,2) + pow(dy,2));

				// check geometry of the cube
				double lDiff = fabs(tCubes[i-1]->cubeDef.length_a - tCubes[j-1]->cubeDef.length_a) + fabs(tCubes[i-1]->cubeDef.length_b - tCubes[j-1]->cubeDef.length_b);
				
				if (dis < tCubes[i-1]->cubeDef.radius && lDiff < 0.03) 												/// TODO Threshold
				{
					Vector2 grCe = tCubes[j-1]->cubeDef.groundCenter;														// ground center

					tCubes[i-1]->FormerTracked(tCubes[j-1]->object_id);													// mark tCubes[i-1] as tracked (age=0)
					tCubes[j-1]->NextTracked(tCubes[i-1]->object_id);														// mark old tCube as tracked from tCubes[i-1] (age=1)

					Objects(tCubes[i-1]->object_id)->Tracked(tCubes[j-1]->object_id, grCe);			// mark Object as tracked

					// calculate new tracking distance
					NewTrackDist(tCubes[i-1]->cubeDef.groundCenter, tCubes[j-1]->cubeDef.groundCenter);
        }
      }
    }
  }
}

/**
 *	@brief Try to track a cube with a flap from next image
 */
void ObjectTracker::TrackFlap2Cube()
{
	// get all cubes with age=1 which could not be tracked to a cube with age=0
	for(unsigned j=tCubes.Size(); j>0; j--)
	{
		if (tCubes[j-1]->age == 1 && tCubes[j-1]->id_next_tracked == UNDEF_ID)
		{
//printf("TrackFlap2Cube(): cube %u with flap\n", tCubes[j-1]->object_id);

			// get all flaps
			for (unsigned i=0; i<NumFlaps(); i++)
			{
				if (Flaps(i)->masked == UNDEF_ID) 
				{
					/// is flap near the cube?
					Vector2 dist = Flaps(i)->center - tCubes[j-1]->cubeDef.center;
					double flapDist = dist.Norm();
	
																																							/// TODO Das einzige Kriterium für das Flap ist die Nähe zum gesuchten Cube (das sollte nicht genug sein: Größe od. dergleichen
					if (flapDist < (2*maxTrackDist))																		/// Definition of near flap: flapDist < 2*maxTrackDist
					{

						if (Flaps(i)->oCase == 1)															/// right-left
						{
							CubeDef cd;
							cd.type = Gestalt::TRACKEDCUBE;

							// get cube corner_points
							cd.corner_points[0][0] = LJunctions(Flaps(i)->outerJcts[0])->isct;
							cd.corner_points[0][1] = LJunctions(Flaps(i)->outerJcts[1])->isct;
							cd.corner_points[2][0] = LJunctions(Flaps(i)->outerJcts[2])->isct;
							cd.corner_points[2][1] = LJunctions(Flaps(i)->outerJcts[3])->isct;
							cd.corner_points[1][1] = LJunctions(Flaps(i)->innerJcts[0])->isct;
							cd.corner_points[1][0] = LJunctions(Flaps(i)->innerJcts[1])->isct;

							// calculate the two other corner points
							cd.corner_points[3][0] = cd.corner_points[0][0] - (cd.corner_points[1][0] - cd.corner_points[2][0]);
							cd.corner_points[3][1] = cd.corner_points[0][1] - (cd.corner_points[1][0] - cd.corner_points[2][0]);

							// calculate length of flap and cube edges and calculate the difference between them
							double flapA = (cd.corner_points[1][0] - cd.corner_points[2][0]).Norm();
							double flapB = (cd.corner_points[0][0] - cd.corner_points[1][0]).Norm();
							double cubeA = (tCubes[j-1]->cubeDef.corner_points[1][0] - tCubes[j-1]->cubeDef.corner_points[2][0]).Norm();
							double cubeB = (tCubes[j-1]->cubeDef.corner_points[0][0] - tCubes[j-1]->cubeDef.corner_points[1][0]).Norm();
							double dist = fabs(flapA-cubeA) * fabs(flapB-cubeB);

							// save cube for tracking
							unsigned gestaltID = SaveTrackedCube(cd);
							if (gestaltID == UNDEF_ID && dist < 3.0)									/// TODO Threshold
							{
								// get cube properties
								GetCubeProperties(cd);

								// save tracked ground center
								cd.trackedCubeGroundCenter = tCubes[j-1]->cubeDef.groundCenter;
	
								/// TODO TODO TODO TODO TODO TODO 
// 								if(CheckTrackDist(cd.groundCenter - cd.trackedCubeGroundCenter))
// 								{
									// create new object
									NewGestalt(new Object(Gestalt::TRACKEDCUBE, UNDEF_ID, cd));
		
									// mark tCube as tracked from flap
									tCubes[j-1]->NextTracked(NumObjects()-1);
									tCubes[tCubes.Size()-1]->FormerTracked(tCubes[j-1]->object_id);
	
									// mark Object as tracked
									Objects(NumObjects()-1)->Tracked(tCubes[j-1]->object_id, cd.trackedCubeGroundCenter);

									if(!CheckTrackDist(cd.groundCenter, cd.trackedCubeGroundCenter)) Objects(NumObjects()-1)->Mask(10002);
// 								}
							}
							else if (gestaltID != UNDEF_ID)
							{
								// find object-id
								unsigned objectID;
								for(unsigned i=0; i<NumObjects(); i++)
									if (Objects(i)->type == Gestalt::CUBE && Objects(i)->gestalt_id == gestaltID) objectID = i;
			
								// find tCone with object-id
								unsigned tObjectID;
								for(unsigned i=0; i<tCubes.Size(); i++)
									if(tCubes[i]->object_id == objectID) tObjectID = i;
			
								// mark tCones as tracked
								tCubes[j-1]->NextTracked(tCubes[tObjectID]->object_id);
								tCubes[tObjectID]->FormerTracked(tCubes[j-1]->object_id);
			
								// mark Object as tracked
								Vector2 trCuGrCe = tCubes[j-1]->cubeDef.groundCenter;
								Objects(objectID)->Tracked(tCubes[j-1]->object_id, trCuGrCe);
							}

						}

						if (Flaps(i)->oCase == 2)																/// left-right
						{
								//printf("ObjectTracker:	########### unpossible left - right occured ###########\n");
						}

						if (Flaps(i)->oCase == 3 || Flaps(i)->oCase == 5)				/// front-top or left-top
						{
							CubeDef cd;
							cd.type = Gestalt::TRACKEDCUBE;

							// get cube corner_points
							cd.corner_points[1][1] = LJunctions(Flaps(i)->outerJcts[0])->isct;
							cd.corner_points[2][1] = LJunctions(Flaps(i)->outerJcts[1])->isct;
							cd.corner_points[0][0] = LJunctions(Flaps(i)->outerJcts[2])->isct;
							cd.corner_points[3][0] = LJunctions(Flaps(i)->outerJcts[3])->isct;
							cd.corner_points[2][0] = LJunctions(Flaps(i)->innerJcts[0])->isct;
							cd.corner_points[1][0] = LJunctions(Flaps(i)->innerJcts[1])->isct;

							// calculate the two other corner points
							cd.corner_points[0][1] = cd.corner_points[0][0] - (cd.corner_points[2][0] - cd.corner_points[2][1]);
							cd.corner_points[3][1] = cd.corner_points[3][0] - (cd.corner_points[2][0] - cd.corner_points[2][1]);

							// calculate length of flap and cube edges and calculate the difference between them
							double flapA = (cd.corner_points[1][0] - cd.corner_points[2][0]).Norm();
							double flapB = (cd.corner_points[0][0] - cd.corner_points[1][0]).Norm();
							double cubeA = (tCubes[j-1]->cubeDef.corner_points[1][0] - tCubes[j-1]->cubeDef.corner_points[2][0]).Norm();
							double cubeB = (tCubes[j-1]->cubeDef.corner_points[0][0] - tCubes[j-1]->cubeDef.corner_points[1][0]).Norm();
							double dist = fabs(flapA-cubeA) * fabs(flapB-cubeB);

							// save cube for tracking
							unsigned gestaltID = SaveTrackedCube(cd);
							if (gestaltID == UNDEF_ID && dist < 3.0)
							{	
								// get cube properties
								GetCubeProperties(cd);

								// save tracked ground center
								cd.trackedCubeGroundCenter = tCubes[j-1]->cubeDef.groundCenter;

								/// TODO TODO TODO TODO TODO TODO 
// 								if(CheckTrackDist(cd.groundCenter - cd.trackedCubeGroundCenter))
// 								{
									// create new object
									NewGestalt(new Object(Gestalt::TRACKEDCUBE, UNDEF_ID, cd));
		
									// mark tCube as tracked from flap
									tCubes[j-1]->NextTracked(NumObjects()-1);
									tCubes[tCubes.Size()-1]->FormerTracked(tCubes[j-1]->object_id);
	
									// mark Object as tracked
									Objects(NumObjects()-1)->Tracked(tCubes[j-1]->object_id, cd.trackedCubeGroundCenter);

									if(!CheckTrackDist(cd.groundCenter, cd.trackedCubeGroundCenter)) Objects(NumObjects()-1)->Mask(10002);
// 								}
							}
							else if (gestaltID != UNDEF_ID)
							{
								// find object-id
								unsigned objectID;
								for(unsigned i=0; i<NumObjects(); i++)
									if (Objects(i)->type == Gestalt::CUBE && Objects(i)->gestalt_id == gestaltID) objectID = i;
			
								// find tCone with object-id
								unsigned tObjectID;
								for(unsigned i=0; i<tCubes.Size(); i++)
									if(tCubes[i]->object_id == objectID) tObjectID = i;
			
								// mark tCones as tracked
								tCubes[j-1]->NextTracked(tCubes[tObjectID]->object_id);
								tCubes[tObjectID]->FormerTracked(tCubes[j-1]->object_id);
			
								// mark Object as tracked
								Vector2 trCuGrCe = tCubes[j-1]->cubeDef.groundCenter;
								Objects(objectID)->Tracked(tCubes[j-1]->object_id, trCuGrCe);
							}
						}


						if (Flaps(i)->oCase == 4 || Flaps(i)->oCase == 6)					/// top-front or top-right
						{
							CubeDef cd;
							cd.type = Gestalt::TRACKEDCUBE;

							// get cube corner_points
							cd.corner_points[2][0] = LJunctions(Flaps(i)->outerJcts[0])->isct;
							cd.corner_points[3][0] = LJunctions(Flaps(i)->outerJcts[1])->isct;
							cd.corner_points[1][1] = LJunctions(Flaps(i)->outerJcts[2])->isct;
							cd.corner_points[0][1] = LJunctions(Flaps(i)->outerJcts[3])->isct;
							cd.corner_points[0][0] = LJunctions(Flaps(i)->innerJcts[0])->isct;
							cd.corner_points[1][0] = LJunctions(Flaps(i)->innerJcts[1])->isct;

							// calculate the two other corner points
							cd.corner_points[2][1] = cd.corner_points[1][1] - (cd.corner_points[1][0] - cd.corner_points[2][0]);
							cd.corner_points[3][1] = cd.corner_points[0][1] - (cd.corner_points[1][0] - cd.corner_points[2][0]);

							// calculate length of flap and cube edges and calculate the difference between them
							double flapA = (cd.corner_points[1][0] - cd.corner_points[2][0]).Norm();
							double flapB = (cd.corner_points[0][0] - cd.corner_points[1][0]).Norm();
							double cubeA = (tCubes[j-1]->cubeDef.corner_points[1][0] - tCubes[j-1]->cubeDef.corner_points[2][0]).Norm();
							double cubeB = (tCubes[j-1]->cubeDef.corner_points[0][0] - tCubes[j-1]->cubeDef.corner_points[1][0]).Norm();
							double dist = fabs(flapA-cubeA) * fabs(flapB-cubeB);

							// save cube for tracking
							unsigned gestaltID = SaveTrackedCube(cd);
							if (gestaltID == UNDEF_ID && dist < 3.0)
							{	
								// get cube properties
								GetCubeProperties(cd);

								// save tracked ground center
								cd.trackedCubeGroundCenter = tCubes[j-1]->cubeDef.groundCenter;

								/// TODO TODO TODO TODO TODO TODO 
// 								if(CheckTrackDist(cd.groundCenter - cd.trackedCubeGroundCenter))
// 								{
									// create new object
									NewGestalt(new Object(Gestalt::TRACKEDCUBE, UNDEF_ID, cd));
		
									// imark tCube as tracked from flap
									tCubes[j-1]->NextTracked(NumObjects()-1);
									tCubes[tCubes.Size()-1]->FormerTracked(tCubes[j-1]->object_id);
	
									// mark Object as tracked
									Objects(NumObjects()-1)->Tracked(tCubes[j-1]->object_id, cd.trackedCubeGroundCenter);
	
									if(!CheckTrackDist(cd.groundCenter, cd.trackedCubeGroundCenter)) Objects(NumObjects()-1)->Mask(10002);
// 								}
							}
							else if (gestaltID != UNDEF_ID)
							{
								// find object-id
								unsigned objectID;
								for(unsigned i=0; i<NumObjects(); i++)
									if (Objects(i)->type == Gestalt::CUBE && Objects(i)->gestalt_id == gestaltID) objectID = i;
			
								// find tCone with object-id
								unsigned tObjectID;
								for(unsigned i=0; i<tCubes.Size(); i++)
									if(tCubes[i]->object_id == objectID) tObjectID = i;
			
								// mark tCones as tracked
								tCubes[j-1]->NextTracked(tCubes[tObjectID]->object_id);
								tCubes[tObjectID]->FormerTracked(tCubes[j-1]->object_id);
			
								// mark Object as tracked
								Vector2 trCuGrCe = tCubes[j-1]->cubeDef.groundCenter;
								Objects(objectID)->Tracked(tCubes[j-1]->object_id, trCuGrCe);
							}
						}
					}
				}
			}
		}
	}
}


/**
 *	@brief Try to track a cube with a rectangle
 */
void ObjectTracker::TrackRect2Cube()
{
	// get all cubes which could not be tracked
	for(unsigned j=tCubes.Size(); j>0; j--)
	{
		Array<unsigned> possibleRectangles;		// all possible rectangles, who fits to one of the cube rectangles
		Array<double> posRectSig;							// significance of the possible rectangles
		Array<unsigned> rectSide;							// 0 = left, 1 = right, 2 = top rectangle

		if (tCubes[j-1]->age == 1 && tCubes[j-1]->id_next_tracked == UNDEF_ID)
		{
if(printRect) printf("\n\nCube %u will be tracked with rectangles!\n", tCubes[j-1]->object_id);

			// get all rectangles with their center points and estimate the possible rectangles "near" the old cube
			Vector2 center = tCubes[j-1]->cubeDef.center;
			for (unsigned i=0; i<NumRectangles(); i++)
			{
				double distance = (Rectangles(i)->centerPoint - center).Length();

				if(distance < (2.0 * maxTrackDist))
				{
if(printRect) printf("	Rectangle %u fits to rectangle of cube %u (d: %6.2f)\n", i, tCubes[j-1]->id, distance);

					// Save all possible rectangles
					possibleRectangles.PushBack(i);
					posRectSig.PushBack(distance);						// store distance as default value
					rectSide.PushBack(-1);
				}
			}

			// get center point of left, right and top cube rectangle
			Vector2 lCenterPoint, rCenterPoint, tCenterPoint;
			lCenterPoint = (tCubes[j-1]->cubeDef.corner_points[1][0] + tCubes[j-1]->cubeDef.corner_points[1][1] +
						tCubes[j-1]->cubeDef.corner_points[2][0] + tCubes[j-1]->cubeDef.corner_points[2][1])/4.;
			rCenterPoint = (tCubes[j-1]->cubeDef.corner_points[1][0] + tCubes[j-1]->cubeDef.corner_points[1][1] +
						tCubes[j-1]->cubeDef.corner_points[0][0] + tCubes[j-1]->cubeDef.corner_points[0][1])/4.;
			tCenterPoint = (tCubes[j-1]->cubeDef.corner_points[0][0] + tCubes[j-1]->cubeDef.corner_points[1][0] +
						tCubes[j-1]->cubeDef.corner_points[2][0] + tCubes[j-1]->cubeDef.corner_points[3][0])/4.;

			double lRadius = (tCubes[j-1]->cubeDef.corner_points[1][0] - lCenterPoint).Norm();
			lRadius = max(lRadius, (tCubes[j-1]->cubeDef.corner_points[1][1] - lCenterPoint).Norm());
			lRadius = max(lRadius, (tCubes[j-1]->cubeDef.corner_points[2][0] - lCenterPoint).Norm());
			lRadius = max(lRadius, (tCubes[j-1]->cubeDef.corner_points[2][1] - lCenterPoint).Norm());

			double rRadius = (tCubes[j-1]->cubeDef.corner_points[1][0] - rCenterPoint).Norm();
			rRadius = max(rRadius, (tCubes[j-1]->cubeDef.corner_points[1][1] - rCenterPoint).Norm());
			rRadius = max(rRadius, (tCubes[j-1]->cubeDef.corner_points[0][0] - rCenterPoint).Norm());
			rRadius = max(rRadius, (tCubes[j-1]->cubeDef.corner_points[0][1] - rCenterPoint).Norm());

			double tRadius = (tCubes[j-1]->cubeDef.corner_points[0][0] - tCenterPoint).Norm();
			tRadius = max(tRadius, (tCubes[j-1]->cubeDef.corner_points[1][0] - tCenterPoint).Norm());
			tRadius = max(tRadius, (tCubes[j-1]->cubeDef.corner_points[2][0] - tCenterPoint).Norm());
			tRadius = max(tRadius, (tCubes[j-1]->cubeDef.corner_points[3][0] - tCenterPoint).Norm());


if(printRect) printf("	maxTrackDist = %f\n", maxTrackDist);	/// TODO Noch immer ist alles von der maxTrackDistance abhängig! Ist das notwendig? Kann man die nicht ganz groß machen und mit dem besten Rechteck aussortieren

// printf("Normalise - ");
			for(unsigned k=0; k<possibleRectangles.Size(); k++)
			{
				// Get the direction (phi) of the parallel lines from the rectangle 
				double rectDir0 = Rectangles(possibleRectangles[k])->phi[0];
				double rectDir1 = Rectangles(possibleRectangles[k])->phi[1];

				Vector2 lRectDir[2];			// direction of left rectangle from cube
				lRectDir[0] = Normalise(tCubes[j-1]->cubeDef.corner_points[2][1] - tCubes[j-1]->cubeDef.corner_points[2][0]);	
				lRectDir[1] = Normalise(tCubes[j-1]->cubeDef.corner_points[1][1] - tCubes[j-1]->cubeDef.corner_points[2][1]);	

				double lDir0 = ScaleAngle_0_pi(PolarAngle(lRectDir[0]));
				double lDir1 = ScaleAngle_0_pi(PolarAngle(lRectDir[1]));

				Vector2 rRectDir[2];			// direction of right rectangle from cube
				rRectDir[0] = Normalise(tCubes[j-1]->cubeDef.corner_points[1][1] - tCubes[j-1]->cubeDef.corner_points[1][0]);	
				rRectDir[1] = Normalise(tCubes[j-1]->cubeDef.corner_points[0][1] - tCubes[j-1]->cubeDef.corner_points[1][1]);	

				double rDir0 = ScaleAngle_0_pi(PolarAngle(rRectDir[0]));
				double rDir1 = ScaleAngle_0_pi(PolarAngle(rRectDir[1]));

				Vector2 tRectDir[2];			// direction of top rectangle from cube
				tRectDir[0] = Normalise(tCubes[j-1]->cubeDef.corner_points[2][0] - tCubes[j-1]->cubeDef.corner_points[3][0]);	
				tRectDir[1] = Normalise(tCubes[j-1]->cubeDef.corner_points[1][0] - tCubes[j-1]->cubeDef.corner_points[2][0]);	

				double tDir0 = ScaleAngle_0_pi(PolarAngle(tRectDir[0]));
				double tDir1 = ScaleAngle_0_pi(PolarAngle(tRectDir[1]));


				// Calculate minimal deviation of the directions for left, right, top rectangle
				double l1 = acos(Dot(Rectangles(possibleRectangles[k])->direction[0], lRectDir[0]));
				double l2 = acos(Dot(Rectangles(possibleRectangles[k])->direction[1], lRectDir[1]));
				if (l1 > M_PI/2.) l1 = M_PI - l1;
				if (l2 > M_PI/2.) l2 = M_PI - l2;
				double lr1 = l1 + l2;

				double l3 = acos(Dot(Rectangles(possibleRectangles[k])->direction[0], lRectDir[1]));
				double l4 = acos(Dot(Rectangles(possibleRectangles[k])->direction[1], lRectDir[0]));
				if (l3 > M_PI/2.) l3 = M_PI - l3;
				if (l4 > M_PI/2.) l4 = M_PI - l4;
				double lr2 = l3 + l4;

				double r1 = acos(Dot(Rectangles(possibleRectangles[k])->direction[0], rRectDir[0]));
				double r2 = acos(Dot(Rectangles(possibleRectangles[k])->direction[1], rRectDir[1]));
				if (r1 > M_PI/2.) r1 = M_PI - r1;
				if (r2 > M_PI/2.) r2 = M_PI - r2;
				double rr1 = r1 + r2;

				double r3 = acos(Dot(Rectangles(possibleRectangles[k])->direction[0], rRectDir[1]));
				double r4 = acos(Dot(Rectangles(possibleRectangles[k])->direction[1], rRectDir[0]));
				if (r3 > M_PI/2.) r3 = M_PI - r3;
				if (r4 > M_PI/2.) r4 = M_PI - r4;
				double rr2 = r3 + r4;

				double t1 = acos(Dot(Rectangles(possibleRectangles[k])->direction[0], tRectDir[0]));
				double t2 = acos(Dot(Rectangles(possibleRectangles[k])->direction[1], tRectDir[1]));
				if (t1 > M_PI/2.) t1 = M_PI - t1;
				if (t2 > M_PI/2.) t2 = M_PI - t2;
				double tr1 = t1 + t2;

				double t3 = acos(Dot(Rectangles(possibleRectangles[k])->direction[0], tRectDir[1]));
				double t4 = acos(Dot(Rectangles(possibleRectangles[k])->direction[1], tRectDir[0]));
				if (t3 > M_PI/2.) t3 = M_PI - t3;
				if (t4 > M_PI/2.) t4 = M_PI - t4;
				double tr2 = t3 + t4;

if(printRect) printf("lr1: %4.2f - lr2: %4.2f\n", lr1, lr2);
if(printRect) printf("rr1: %4.2f - rr2: %4.2f\n", rr1, rr2);
if(printRect) printf("tr1: %4.2f - tr2: %4.2f\n", tr1, tr2);

				double lNearResult = Min(lr1, lr2);
				double rNearResult = Min(rr1, rr2);
				double tNearResult = Min(tr1, tr2);

				// offset for the xNearResult and dis, to avoid infinit results for psoRectSig
				double offset    = 3.0;			// offset for xNearResult
				double offset2   = 0.2;			// offset for dis

if(printRect) 				printf("		Rectangle dir0 - dir1: %6.4f - %6.4f\n", rectDir0, rectDir1);
if(printRect) 				printf("			left:  dir0 - dir1: %6.4f - %6.4f\n", lDir0, lDir1);
if(printRect) 				printf("			right: dir0 - dir1: %6.4f - %6.4f\n", rDir0, rDir1);
if(printRect) 				printf("			top:   dir0 - dir1: %6.4f - %6.4f\n", tDir0, tDir1);


				if(lNearResult < rNearResult && lNearResult < tNearResult)					// Left rectangle fits best
				{
					double dis = fabs(lRadius - Rectangles(possibleRectangles[k])->radius);
					posRectSig[k] =  300. / ((lNearResult + offset)*(dis + offset2));	//*posRectSig[k]);
					rectSide[k] = 0;	// left
if(printRect) 					printf("		LEFT RECTANGLE FITS BETTER: %u\n", possibleRectangles[k]);
if(printRect) 					printf("		lRadius: %4.2f  - rectRadius: %4.2f  - dis: %4.2f\n", lRadius, Rectangles(possibleRectangles[k])->radius, dis);
if(printRect) 					printf("		nearest Result: %4.2f\n", lNearResult);
if(printRect) 					printf("		posRectSig: %4.2f\n\n", posRectSig[k]);
				}
				else if (rNearResult < lNearResult && rNearResult < tNearResult)		// Right rectangle fits best
				{
					double dis = fabs(rRadius - Rectangles(possibleRectangles[k])->radius);
					posRectSig[k] =  300. / ((lNearResult + offset)*(dis + offset2)); //*posRectSig[k]);
					rectSide[k] = 1;	// right
if(printRect) 					printf("		RIGHT RECTANGLE FITS BETTER: %u\n", possibleRectangles[k]);
if(printRect) 					printf("		rRadius: %4.2f  - rectRadius: %4.2f  - dis: %4.2f\n", rRadius, Rectangles(possibleRectangles[k])->radius, dis);
if(printRect) 					printf("		nearest Result: %4.2f\n", rNearResult);
if(printRect) 					printf("		posRectSig: %4.2f\n\n", posRectSig[k]);
				}
				else if (tNearResult < lNearResult && tNearResult < rNearResult)		// Top rectangle fits best
				{
					double dis = fabs(tRadius - Rectangles(possibleRectangles[k])->radius);
					posRectSig[k] =  300. / ((lNearResult + offset)*(dis + offset2)); //*posRectSig[k]);
					rectSide[k] = 2;	// top
if(printRect) 					printf("		TOP RECTANGLE FITS BETTER: %u\n", possibleRectangles[k]);
if(printRect) 					printf("		tRadius: %4.2f  - rectRadius: %4.2f  - dis: %4.2f\n", tRadius, Rectangles(possibleRectangles[k])->radius, dis);
if(printRect) 					printf("		nearest Result: %4.2f\n", tNearResult);
if(printRect) 					printf("		posRectSig: %4.2f\n\n", posRectSig[k]);
				}
			}

// printf("end\n");

			// find out the rectangle with the highest significance
			unsigned bestRect = -1;
			double bestRectSig = 0.;
			unsigned bestRectSide = -1;
			for(unsigned k=0; k<possibleRectangles.Size(); k++)
			{
				if(bestRectSig < posRectSig[k])
				{
					bestRect = possibleRectangles[k];
					bestRectSig = posRectSig[k];
					bestRectSide = rectSide[k];
				}
			}

if(printRect) 			printf("Best rectangle is %u: %4.2f\n", bestRect, bestRectSig);

			if (bestRectSig > 5.)																									/// TODO ARI: Threshold of 10 for a good rectangle
			{
				// make new tCube, if significance is high enough
				CubeDef cd;
				cd.type = Gestalt::TRACKEDCUBE;
	
				// get cube corner_points
				if(bestRectSide == 0)					/// left rectangle
				{
					// calculate difference of center points from old to new (tracked) rectangle
					Vector2 d;
					d.x = Rectangles(bestRect)->centerPoint.x - lCenterPoint.x;
					d.y = Rectangles(bestRect)->centerPoint.y - lCenterPoint.y;
if(printRect) 	printf("dx - dy: %4.2f - %4.2f\n", d.x, d.y);
	
					// calculate all new cornerpoints
					cd.corner_points[1][0] = tCubes[j-1]->cubeDef.corner_points[1][0] + d;
					cd.corner_points[1][1] = tCubes[j-1]->cubeDef.corner_points[1][1] + d;
					cd.corner_points[2][0] = tCubes[j-1]->cubeDef.corner_points[2][0] + d;
					cd.corner_points[2][1] = tCubes[j-1]->cubeDef.corner_points[2][1] + d;
					cd.corner_points[3][0] = tCubes[j-1]->cubeDef.corner_points[3][0] + d;
					cd.corner_points[0][1] = tCubes[j-1]->cubeDef.corner_points[0][1] + d;
					cd.corner_points[0][0] = tCubes[j-1]->cubeDef.corner_points[0][0] + d;
					cd.corner_points[3][1] = tCubes[j-1]->cubeDef.corner_points[3][1] + d;
				}
				if(bestRectSide == 1)						/// right rectangle
				{
					// calculate difference of center points from old to new (tracked) rectangle
					Vector2 d;
					d.x = Rectangles(bestRect)->centerPoint.x - rCenterPoint.x;
					d.y = Rectangles(bestRect)->centerPoint.y - rCenterPoint.y;
if(printRect) 	printf("dx - dy: %4.2f - %4.2f\n", d.x, d.y);
	
					// calculate all new cornerpoints
					cd.corner_points[1][0] = tCubes[j-1]->cubeDef.corner_points[1][0] + d;
					cd.corner_points[1][1] = tCubes[j-1]->cubeDef.corner_points[1][1] + d;
					cd.corner_points[2][0] = tCubes[j-1]->cubeDef.corner_points[2][0] + d;
					cd.corner_points[2][1] = tCubes[j-1]->cubeDef.corner_points[2][1] + d;
					cd.corner_points[3][0] = tCubes[j-1]->cubeDef.corner_points[3][0] + d;
					cd.corner_points[0][1] = tCubes[j-1]->cubeDef.corner_points[0][1] + d;
					cd.corner_points[0][0] = tCubes[j-1]->cubeDef.corner_points[0][0] + d;
					cd.corner_points[3][1] = tCubes[j-1]->cubeDef.corner_points[3][1] + d;
				}
				if(bestRectSide == 2)						/// top rectangle
				{
					// calculate difference of center points from old to new (tracked) rectangle
					Vector2 d;
					d.x = Rectangles(bestRect)->centerPoint.x - tCenterPoint.x;
					d.y = Rectangles(bestRect)->centerPoint.y - tCenterPoint.y;
if(printRect) 	printf("dx - dy: %4.2f - %4.2f\n", d.x, d.y);
	
					// calculate all new cornerpoints
					cd.corner_points[1][0] = tCubes[j-1]->cubeDef.corner_points[1][0] + d;
					cd.corner_points[1][1] = tCubes[j-1]->cubeDef.corner_points[1][1] + d;
					cd.corner_points[2][0] = tCubes[j-1]->cubeDef.corner_points[2][0] + d;
					cd.corner_points[2][1] = tCubes[j-1]->cubeDef.corner_points[2][1] + d;
					cd.corner_points[3][0] = tCubes[j-1]->cubeDef.corner_points[3][0] + d;
					cd.corner_points[0][1] = tCubes[j-1]->cubeDef.corner_points[0][1] + d;
					cd.corner_points[0][0] = tCubes[j-1]->cubeDef.corner_points[0][0] + d;
					cd.corner_points[3][1] = tCubes[j-1]->cubeDef.corner_points[3][1] + d;
				}
	
				// get cube properties
				GetCubeProperties(cd);

				// save cube for tracking
				unsigned gestaltID = SaveTrackedCube(cd);
				if (gestaltID == UNDEF_ID)
				{	
if(printRect) 	printf("NEW CUBE FROM RECTANGLE\n");

					// save tracked ground center
					cd.trackedCubeGroundCenter = tCubes[j-1]->cubeDef.groundCenter;

					/// TODO TODO TODO TODO TODO TODO 
// 					if (CheckTrackDist(cd.groundCenter - cd.trackedCubeGroundCenter))
// 					{
						// create new object
						NewGestalt(new Object(Gestalt::TRACKEDCUBE, UNDEF_ID, cd));
	
						// mark tCube as tracked from flap
						tCubes[j-1]->NextTracked(NumObjects()-1);
						tCubes[tCubes.Size()-1]->FormerTracked(tCubes[j-1]->object_id);
		
						// mark Object as tracked
						Objects(NumObjects()-1)->Tracked(tCubes[j-1]->object_id, cd.trackedCubeGroundCenter);

						if(!CheckTrackDist(cd.groundCenter, cd.trackedCubeGroundCenter)) Objects(NumObjects()-1)->Mask(10002);

// 					}
				}
				else
				{
					// find object-id
					unsigned objectID;
					for(unsigned i=0; i<NumObjects(); i++)
						if (Objects(i)->type == Gestalt::CUBE && Objects(i)->gestalt_id == gestaltID) objectID = i;

					// find tCone with object-id
					unsigned tObjectID;
					for(unsigned i=0; i<tCubes.Size(); i++)
						if(tCubes[i]->object_id == objectID) tObjectID = i;

					// mark tCones as tracked
					tCubes[j-1]->NextTracked(tCubes[tObjectID]->object_id);
					tCubes[tObjectID]->FormerTracked(tCubes[j-1]->object_id);

					// mark Object as tracked
					Vector2 trCuGrCe = tCubes[j-1]->cubeDef.groundCenter;
					Objects(objectID)->Tracked(tCubes[j-1]->object_id, trCuGrCe);
				}
			}
		}
	}

if(printRect) 	printf("\n\n");
}


/**
 *	@brief SaveTrackedCube
 *	@param cd Cube definition, containint all cube properties
 *	@return Returns the id of a cube, if another cube exists at the same position (else UNDEF_ID)
 */
unsigned ObjectTracker::SaveTrackedCube(CubeDef cd)
{
	// get center point of cube and radius (2D)
	Vector2 center = (cd.corner_points[0][0] + cd.corner_points[0][1] +
			cd.corner_points[3][0] + cd.corner_points[1][1] +
			cd.corner_points[2][0] + cd.corner_points[2][1])/6.;
	
	// Only new object, if there is no cube at this position
	unsigned existingCube = CheckExistingCube(center);
	if(existingCube == UNDEF_ID)
	{
		unsigned cubeId = 1000; 			/// TODO virtual cube id for tracked cubes
	
		// save cube for tracking
		TCube *c = new TCube(NumObjects(), cubeId, cd);
		tCubes.PushBack(c);
		return UNDEF_ID;
	}
	else return existingCube;
}

/**
 *	@brief Checks if the tracked cube already exists. Returns UNDEF_ID when no cube is at this position or the id
 *	of the cube, if a cube exists.
 */
unsigned ObjectTracker::CheckExistingCube(Vector2 cp)
{
	for(unsigned i=0; i<NumCubes(); i++)
		if(Cubes(i)->masked == UNDEF_ID && (cp - Cubes(i)->center).Length() < Cubes(i)->radius) return i;
	return UNDEF_ID;
}


/**
 *	@brief Get all cones, which are not masked and estimate the properties (also 3D)
 */
void ObjectTracker::GetCones()
{
  for(unsigned i=0; i<NumCones(); i++)
  {
    if (Cones(i)->masked == UNDEF_ID)
    {
			ConeDef cd;
	
			cd.cone = i;
			cd.ellipse = Cones(i)->ellipse;
			cd.ljct = Cones(i)->ljct;

			// get properties for the cone
			GetConeProperties(cd);

			// save cones for tracking
			tCones.PushBack(new TCone(NumObjects(), i, cd));

			// make new object
      NewGestalt(new Object(Gestalt::CONE, i, cd));
		}
	}
}

/**
 *	@brief Calculate the properties of the cone with respect to the camera-parameters
 *	The cone definition (cd) must contain the ellipse and ljct.
 *	@param cd Cone definition, containing all properties of a cone
 */
void ObjectTracker::GetConeProperties(ConeDef &cd)
{
	// get ellipse parameter
	if (cd.ellipse != UNDEF_ID)
	{
		cd.x = Ellipses(cd.ellipse)->x;
		cd.y = Ellipses(cd.ellipse)->y;
		cd.a = Ellipses(cd.ellipse)->a;
		cd.b = Ellipses(cd.ellipse)->b;
		cd.phi = Ellipses(cd.ellipse)->phi;

		// get left and right vertex of the ellipse
		cd.vertex[0] = Ellipses(cd.ellipse)->vertex[0];
		cd.vertex[1] = Ellipses(cd.ellipse)->vertex[1];
	}

	// get l-junction intersection
	if (cd.ljct != UNDEF_ID)
	{
		cd.isct = LJunctions(cd.ljct)->isct;
	}

	// get center and radius of cone
	if (cd.ellipse != UNDEF_ID && cd.ljct != UNDEF_ID)
	{
		cd.center.x = (cd.x + cd.isct.x)/2.;
		cd.center.y = (cd.y + cd.isct.y)/2.;
		cd.radius = fabs((cd.center - cd.isct).Norm() + cd.b);
	}

	// get ground center
	cd.groundCenter.x = cd.x;
	cd.groundCenter.y = cd.y;

	// get cone 3D properties
	GetCone3DProperties(cd);
}

/**
 *	@brief Calculate the properties of the cone with respect to the camera-parameters
*/
void ObjectTracker::GetCone3DProperties(ConeDef &cd)
{
	double dAboveGround = 0.;				// 3D points are above ground plane?

	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point

	tPointImg = cvPoint2D64f (cd.groundCenter.x, cd.groundCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.groundCenter3D.x = tPointWorld3D.x;
	cd.groundCenter3D.y = tPointWorld3D.y;

	// Calculate radius in 3D berechnen!
	double radius2D = 0.;
	if(cd.a > cd.b) radius2D = cd.a;
	else radius2D = cd.b;
	tPointImg = cvPoint2D64f (cd.groundCenter.x + radius2D, cd.groundCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.radius3D = tPointWorld3D.x - cd.groundCenter3D.x;
}


/**
 *	@brief Try to track old cones with new ones
 */
void ObjectTracker::TrackCone()
{

  for(unsigned i=tCones.Size(); i>0; i--)
  {
    if (tCones[i-1]->age != 0) break;

    for(unsigned j=tCones.Size(); j>0; j--)
    {
      if (tCones[j-1]->age == 1)
      {
				// calculate distance between ellipse center points
				double dx = tCones[i-1]->coneDef.x - tCones[j-1]->coneDef.x;
				double dy = tCones[i-1]->coneDef.y - tCones[j-1]->coneDef.y;
				double dis = sqrt(pow(dx,2) + pow(dy,2));

				if (dis < 2.0*tCones[i-1]->coneDef.a)							/// TODO TODO TODO TODO Wie groß muss die Distanz sein? 
				{
//					double trackDist = 0;
					Vector2 groundCenterI; 
					groundCenterI.x = tCones[i-1]->coneDef.x;
					groundCenterI.y = tCones[i-1]->coneDef.y;

					Vector2 groundCenterJ;
					groundCenterJ.x = tCones[j-1]->coneDef.x;
					groundCenterJ.y = tCones[j-1]->coneDef.y;

					// mark tCones as tracked
          tCones[i-1]->FormerTracked(tCones[j-1]->object_id);
					tCones[j-1]->NextTracked(tCones[i-1]->object_id);

					// mark objects as tracked
          Objects(tCones[i-1]->object_id)->Tracked(tCones[j-1]->object_id, groundCenterJ);			// mark Object as tracked

					// calculate new tracking distance
					NewTrackDist(groundCenterI, groundCenterJ);
        }
      }
    }
  }
}


/**
 *	@brief Try to track the same cone from last image
 */
void ObjectTracker::TrackEllipse2Cone()
{
	// get all cones with age=1 which could not be tracked to a cone with age=0
	for(unsigned j=tCones.Size(); j>0; j--)
	{
		if (tCones[j-1]->age == 1 && tCones[j-1]->id_next_tracked == UNDEF_ID)
		{
			Array<unsigned> possibleEllipses;			// all possible ellipses, who may fit to the cone
			Array<double> posEllSig;							// significance of the possible ellipses
			Array<Vector2> posEllCenterDis;				// Distance between center points

			// get all ellipses
			for (unsigned i=0; i<NumEllipses(); i++)
			{
				// is the center point of the ellipse near the centerPoint of tCone?
				Vector2 distance;
				distance.x = Ellipses(i)->x - tCones[j-1]->coneDef.x;
				distance.y = Ellipses(i)->y - tCones[j-1]->coneDef.y;
				double dis = distance.Norm();

				// save ellipse as "near"
				if(dis < (2. * tCones[j-1]->coneDef.a/*maxTrackDist*/))
				{
					// Save all possible ellipses
					possibleEllipses.PushBack(i);				// store ellipses
					posEllSig.PushBack(100./dis);				// store 100/dis as significance value
					posEllCenterDis.PushBack(distance);	// store Vector distance
				}
			}

			// find the best ellipse
			double ref = 0.;
			unsigned bestEllipse;
			for(unsigned k=0; k<possibleEllipses.Size(); k++)
			{
				// get difference of size of the found ellipse to the cone-ellipse
				double dSize = fabs(tCones[j-1]->coneDef.a - Ellipses(possibleEllipses[k])->a);

				// get difference of angles
				double dPhi = fabs(tCones[j-1]->coneDef.phi - Ellipses(possibleEllipses[k])->phi);
				if (dPhi > M_PI/2.) dPhi = M_PI - dPhi;

//printf("possible Ellipses: %u	- sig: %4.2f - dSize: %4.2f - s/b: %4.2f - dPhi: %4.2f - sig: %4.2f\n", 
//					possibleEllipses[k], posEllSig[k], dSize, posEllSig[k]/dSize, dPhi, posEllSig[k]/(dSize*3*dPhi));
				if(posEllSig[k]/(dSize*3*dPhi) > ref)
				{
					bestEllipse = k;
					ref = posEllSig[k]/(dSize*3*dPhi);
				}
			}

			// create cone from best ellipse
			if (ref > 5.0) 																																					/// TODO Threshold for Tracking Ellipses to Cones
			{
				// cone definition
				ConeDef coneDef = tCones[j-1]->coneDef;

				coneDef.cone = UNDEF_ID;
				coneDef.ellipse = possibleEllipses[bestEllipse];
				coneDef.ljct = UNDEF_ID;

				// calculate vector from old to new points
				coneDef.x += posEllCenterDis[bestEllipse].x;
				coneDef.y += posEllCenterDis[bestEllipse].y;
				coneDef.isct.x += posEllCenterDis[bestEllipse].x;
				coneDef.isct.y += posEllCenterDis[bestEllipse].y;
				coneDef.vertex[0].x += posEllCenterDis[bestEllipse].x;
				coneDef.vertex[0].y += posEllCenterDis[bestEllipse].y;
				coneDef.vertex[1].x += posEllCenterDis[bestEllipse].x;
				coneDef.vertex[1].y += posEllCenterDis[bestEllipse].y;
				coneDef.center.x += posEllCenterDis[bestEllipse].x;
				coneDef.center.y += posEllCenterDis[bestEllipse].y;

				Vector2 cp;
				cp.x = coneDef.x;
				cp.y = coneDef.y;

				unsigned gestaltID = CheckExistingCone(cp);
				if(gestaltID != UNDEF_ID)
				{
					// find object-id
					unsigned objectID;
					for(unsigned i=0; i<NumObjects(); i++)
						if (Objects(i)->type == Gestalt::CONE && Objects(i)->gestalt_id == gestaltID) objectID = i;

					// find tCone with object-id
					unsigned tObjectID;
					for(unsigned i=0; i<tCones.Size(); i++)
						if(tCones[i]->object_id == objectID) tObjectID = i;

					// mark tCones as tracked
					tCones[j-1]->NextTracked(tCones[tObjectID]->object_id);
					tCones[tObjectID]->FormerTracked(tCones[j-1]->object_id);

					// mark Object as tracked
					Vector2 trCuGrCe = tCones[j-1]->coneDef.groundCenter;
					Objects(objectID)->Tracked(tCones[j-1]->object_id, trCuGrCe);
				}
				else
				{
					// copy ground center and tracked ground cente
					coneDef.groundCenter.x = coneDef.x;
					coneDef.groundCenter.y = coneDef.y;
					coneDef.trackedConeGroundCenter.x = tCones[j-1]->coneDef.groundCenter.x;
					coneDef.trackedConeGroundCenter.y = tCones[j-1]->coneDef.groundCenter.y;
	
					/// TODO TODO TODO TODO TODO TODO 
// 					if(CheckTrackDist(coneDef.groundCenter - coneDef.trackedConeGroundCenter))
// 					{
						// create new object
						NewGestalt(new Object(Gestalt::TRACKEDCONE, UNDEF_ID, coneDef));
		
						// save cone for tracking
						tCones.PushBack(new TCone(NumObjects()-1, UNDEF_ID, coneDef));
			
						// mark tCones as tracked
						tCones[j-1]->NextTracked(NumObjects()-1);
						tCones[tCones.Size()-1]->FormerTracked(tCones[j-1]->object_id);
		
						// mark Object as tracked
						Objects(NumObjects()-1)->Tracked(tCones[j-1]->object_id, coneDef.trackedConeGroundCenter);

						if(!CheckTrackDist(coneDef.groundCenter, coneDef.trackedConeGroundCenter)) Objects(NumObjects()-1)->Mask(10002);

// 					}
				}
			}
		}
	}
}

/**
 *	@brief Check if the tracked cone already exists. Returns UNDEF_ID when no cone is at this position or the id
 *	of the cone, if there is one at this position.
 *	@param cp Center point
 *	@return ID of the cone or UNDEF_ID
 */
unsigned ObjectTracker::CheckExistingCone(Vector2 cp)
{
	for(unsigned i=0; i<NumCones(); i++)
		if(Cones(i)->masked == UNDEF_ID && (cp - Cones(i)->ellipseCenter).Length() < Cones(i)->topRadius) return i;
	return UNDEF_ID;
}


/**
 *	@brief Get all cylinders, which are not masked
 */
void ObjectTracker::GetCylinders()
{
  for(unsigned i=0; i<NumCylinders(); i++)
  {
    if (Cylinders(i)->masked == UNDEF_ID)
    {
			CylDef cd;
	
			cd.cylinder = i;
			cd.ellipses[0] = ExtEllipses(Cylinders(i)->extEllipses[0])->ellipse;
			cd.ellipses[1] = ExtEllipses(Cylinders(i)->extEllipses[1])->ellipse;
			
			GetCylinderProperties(cd);

			// save cylinder for tracking
			tCylinders.PushBack(new TCylinder(NumObjects(), i, cd));

			// create new object (CYLINDER)
      NewGestalt(new Object(Gestalt::CYLINDER, i, cd));
		}
	}
}


/**
 *	@brief Calculate the properties of the cylinder
 */
void ObjectTracker::GetCylinderProperties(CylDef &cd)
{
	// get ellipse parameter of first ellipse
	if (cd.ellipses[0] != UNDEF_ID)
	{
		cd.x[0] = Ellipses(cd.ellipses[0])->x;
		cd.y[0] = Ellipses(cd.ellipses[0])->y;
		cd.a[0] = Ellipses(cd.ellipses[0])->a;
		cd.b[0] = Ellipses(cd.ellipses[0])->b;
		cd.phi[0] = Ellipses(cd.ellipses[0])->phi;

		// get left and right vertex of the ellipse
		cd.vertex[0][0] = Ellipses(cd.ellipses[0])->vertex[0];
		cd.vertex[0][1] = Ellipses(cd.ellipses[0])->vertex[1];
	}

	// get ellipse parameter of second ellipse
	if (cd.ellipses[1] != UNDEF_ID)
	{
		cd.x[1] = Ellipses(cd.ellipses[1])->x;
		cd.y[1] = Ellipses(cd.ellipses[1])->y;
		cd.a[1] = Ellipses(cd.ellipses[1])->a;
		cd.b[1] = Ellipses(cd.ellipses[1])->b;
		cd.phi[1] = Ellipses(cd.ellipses[1])->phi;

		// get left and right vertex of the ellipse
		cd.vertex[1][0] = Ellipses(cd.ellipses[1])->vertex[0];
		cd.vertex[1][1] = Ellipses(cd.ellipses[1])->vertex[1];
	}

	// TODO constraint: ground center is where y is higher (nearer the bottom of the image)
	// get ground center of cylinder
	if(cd.ellipses[0] != UNDEF_ID && cd.ellipses[1] != UNDEF_ID)
	{
		if(cd.y[0] > cd.y[1])
		{
			cd.groundCenter.x = cd.x[0];
			cd.groundCenter.y = cd.y[0];
		}
		else
		{
			cd.groundCenter.x = cd.x[1];
			cd.groundCenter.y = cd.y[1];
		}
	}

	// get equalVertex flag
	cd.equalVertex = Cylinders(cd.cylinder)->equalVertex;

	// calculate direction from first to second ellipse
	cd.dir.x = cd.x[1] - cd.x[0];
	cd.dir.y = cd.y[1] - cd.y[0];
	if (cd.dir.x != 0 && cd.dir.y != 0) cd.dir.Normalise();

	// calculate center point of cylinder
	cd.center.x = (cd.x[0] + cd.x[1])/2.;
	cd.center.y = (cd.y[0] + cd.y[1])/2.;

	// calculate radius of cylinder (center point to max. end of ellipse)
	Vector2 r0, r1;
	r0.x = cd.center.x - cd.x[0];
	r0.y = cd.center.y - cd.y[0];
	r1.x = cd.center.x - cd.x[1];
	r1.y = cd.center.y - cd.y[1];
	double rad0 = r0.Length() + cd.b[0];
	double rad1 = r1.Length() + cd.b[1];
	cd.radius = Max(rad0, rad1);

	GetCylinder3DProperties(cd);
}

/**
 *	@brief Calculate the properties of the cylinder
 *	@param cd Cylinder definition, containing all properties of the cylinder
 */
void ObjectTracker::GetCylinder3DProperties(CylDef &cd)
{
// printf("\n\n	Equal vertex: %u\n", cd.equalVertex);
// printf("	Ellipse 0: %4.2f - %4.2f - %4.2f - %4.2f - %4.2f\n", cd.x[0], cd.y[0], cd.a[0], cd.b[0], cd.phi[0]);
// printf("	Ellipse 1: %4.2f - %4.2f - %4.2f - %4.2f - %4.2f\n", cd.x[1], cd.y[1], cd.a[1], cd.b[1], cd.phi[1]);

	// calculate cylinder 3D properties
	double dAboveGround = 0.;				// 3D points are above ground plane?

	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point

	// calculate groundCenter3D
	tPointImg = cvPoint2D64f (cd.groundCenter.x, cd.groundCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.groundCenter3D.x = tPointWorld3D.x;
	cd.groundCenter3D.y = tPointWorld3D.y;

	// TODO ersetzen
	// calculate radius3D
// 	double radius2D = 0.;
// 	if(cd.a[0] > cd.b[0]) radius2D = cd.a[0];
// 	else radius2D = cd.b[0];
// 	tPointImg = cvPoint2D64f (cd.groundCenter.x + radius2D, cd.groundCenter.y);
// 	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
// 	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
// 	cd.radius3D = tPointWorld3D.x - cd.groundCenter3D.x;

	// calculate 3D vertices of both ellipses
	tPointImg = cvPoint2D64f (cd.vertex[0][0].x, cd.vertex[0][0].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.vertex3D[0][0].x = tPointWorld3D.x;
	cd.vertex3D[0][0].y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (cd.vertex[0][1].x, cd.vertex[0][1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.vertex3D[0][1].x = tPointWorld3D.x;
	cd.vertex3D[0][1].y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (cd.vertex[1][0].x, cd.vertex[1][0].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.vertex3D[1][0].x = tPointWorld3D.x;
	cd.vertex3D[1][0].y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (cd.vertex[1][1].x, cd.vertex[1][1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	cd.vertex3D[1][1].x = tPointWorld3D.x;
	cd.vertex3D[1][1].y = tPointWorld3D.y;

	// TODO one calculated ellipse on the ground plane is false => normaly the one farthest away => recalculate the vertices
	if(cd.vertex3D[1][0].y > cd.vertex3D[0][0].y)
	{
// 		printf("\n   ObjectTracker: ellipse match\n");
		cd.match = true;
// 		printf("	vertex l,r = %4.4f - %4.4f\n", cd.vertex3D[1][0].y, cd.vertex3D[0][0].y);

		// corner point [1][0] (3D) (is above corner_ponit[1][1])
		tPointImg = cvPoint2D64f (cd.vertex[1][0].x, cd.vertex[1][0].y);
		m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
		m_cCamModel.World2YPlane (tPointWorld3D, &tPointWorld3D, cd.vertex3D[0][0].y);
		cd.vertex3D[1][0].x = tPointWorld3D.x;
		cd.vertex3D[1][0].y = tPointWorld3D.y;
		cd.height = tPointWorld3D.z; 

		// corner point [1][0] (3D) (is above corner_ponit[1][1])
		tPointImg = cvPoint2D64f (cd.vertex[1][1].x, cd.vertex[1][1].y);
		m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
		m_cCamModel.World2YPlane (tPointWorld3D, &tPointWorld3D, cd.vertex3D[0][1].y);
		cd.vertex3D[1][1].x = tPointWorld3D.x;
		cd.vertex3D[1][1].y = tPointWorld3D.y;
		cd.height = (cd.height + tPointWorld3D.z)/2.;		// HACK: mean value => gerader cylinder!!! 

		// ground and top center in 3D
		cd.groundCenter3D = (cd.vertex3D[0][0] + cd.vertex3D[0][1])/2.;
		cd.topCenter3D = (cd.vertex3D[1][0] + cd.vertex3D[1][1])/2.;

		// center of the cylinder
		cd.cylinderCenter3D = (cd.groundCenter3D + cd.topCenter3D)/2.;

		// calculate radius of ground and top surface
		cd.radius3D = (cd.vertex3D[0][0] - cd.vertex3D[0][1]).Norm()/2.;
		cd.topRadius3D = (cd.vertex3D[1][0] - cd.vertex3D[1][1]).Norm()/2.;

// 		printf("	=> ground vertices: %4.4f / %4.4f - %4.4f / %4.4f\n", cd.vertex3D[0][0].x, cd.vertex3D[0][0].y, cd.vertex3D[0][1].x, cd.vertex3D[0][1].y);
// 		printf("	=>    top vertices: %4.4f / %4.4f - %4.4f / %4.4f\n", cd.vertex3D[1][0].x, cd.vertex3D[1][0].y, cd.vertex3D[1][1].x, cd.vertex3D[1][1].y);
// 		printf("	=> center point 3D: %4.4f / %4.4f / %4.4f\n", cd.cylinderCenter3D.x, cd.cylinderCenter3D.y, cd.height/2.);
// 		printf("	=> radius (ground/top): %4.4f - %4.4f\n", cd.radius3D, cd.topRadius3D);
// 		printf("	=> height: %4.4f\n", cd.height);
	}
	else
	{
// 		printf("\n   ObjectTracker: ellipse mismatch (top / bottom!)\n");
		cd.match = false;
// 		printf("	vertex l,r = %4.4f - %4.4f\n", cd.vertex3D[1][0].y, cd.vertex3D[0][0].y);

		// corner point [1][0] (3D) (is above corner_ponit[1][1])
		tPointImg = cvPoint2D64f (cd.vertex[0][0].x, cd.vertex[0][0].y);
		m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
		m_cCamModel.World2YPlane (tPointWorld3D, &tPointWorld3D, cd.vertex3D[1][0].y);
		cd.vertex3D[0][0].x = tPointWorld3D.x;
		cd.vertex3D[0][0].y = tPointWorld3D.y;
		cd.height = tPointWorld3D.z; 

		// corner point [1][0] (3D) (is above corner_ponit[1][1])
		tPointImg = cvPoint2D64f (cd.vertex[0][1].x, cd.vertex[0][1].y);
		m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
		m_cCamModel.World2YPlane (tPointWorld3D, &tPointWorld3D, cd.vertex3D[1][1].y);
		cd.vertex3D[0][1].x = tPointWorld3D.x;
		cd.vertex3D[0][1].y = tPointWorld3D.y;
		cd.height = (cd.height + tPointWorld3D.z)/2.;		// mean value => gerader cylinder!!! 

		// ground and top center in 3D
		cd.groundCenter3D = (cd.vertex3D[1][0] + cd.vertex3D[1][1])/2.;
		cd.topCenter3D = (cd.vertex3D[0][0] + cd.vertex3D[0][1])/2.;

		// center of the cylinder
		cd.cylinderCenter3D = (cd.groundCenter3D + cd.topCenter3D)/2.;

		// calculate radius of ground and top surface
		cd.radius3D = (cd.vertex3D[1][0] - cd.vertex3D[1][1]).Norm()/2.;
		cd.topRadius3D = (cd.vertex3D[0][0] - cd.vertex3D[0][1]).Norm()/2.;

// 		printf("	=> ground vertices: %4.4f / %4.4f - %4.4f / %4.4f\n", cd.vertex3D[0][0].x, cd.vertex3D[0][0].y, cd.vertex3D[0][1].x, cd.vertex3D[0][1].y);
// 		printf("	=>    top vertices: %4.4f / %4.4f - %4.4f / %4.4f\n", cd.vertex3D[1][0].x, cd.vertex3D[1][0].y, cd.vertex3D[1][1].x, cd.vertex3D[1][1].y);
// 		printf("	=> center point 3D: %4.4f / %4.4f / %4.4f\n", cd.cylinderCenter3D.x, cd.cylinderCenter3D.y, cd.height/2.);
// 		printf("	=> radius (ground/top): %4.4f - %4.4f\n", cd.radius3D, cd.topRadius3D);
// 		printf("	=> height: %4.4f\n", cd.height);
	}
}


/**
 *	@brief Try to track the same cylinder
 */
void ObjectTracker::TrackCylinder()
{
if(printTrackCylinder) printf("Track Cylinder:\n");

  for(unsigned i=tCylinders.Size(); i>0; i--)
  {
    if (tCylinders[i-1]->age != 0) break;

    for(unsigned j=tCylinders.Size(); j>0; j--)
    {
      if (tCylinders[j-1]->age == 1 && tCylinders[j-1]->id_next_tracked == UNDEF_ID)
      {
				// calculate distance between ellipse center points (ellipse 0-0 and 1-1)
				double dx0 = tCylinders[i-1]->cylDef.x[0] - tCylinders[j-1]->cylDef.x[0];
				double dy0 = tCylinders[i-1]->cylDef.y[0] - tCylinders[j-1]->cylDef.y[0];
				double dis0 = sqrt(pow(dx0,2) + pow(dy0,2));

				double dx1 = tCylinders[i-1]->cylDef.x[1] - tCylinders[j-1]->cylDef.x[1];
				double dy1 = tCylinders[i-1]->cylDef.y[1] - tCylinders[j-1]->cylDef.y[1];
				double dis1 = sqrt(pow(dx1,2) + pow(dy1,2));

				double dDis0 = fabs(dis0 - dis1);		// TODO wird nicht mehr gebraucht
				double sum0 = dis0 + dis1;

if(printTrackCylinder) printf("	Distance for cylinder %u: dis = %4.2f - %4.2f\n", tCylinders[i-1]->object_id, dis0, dis1);
if(printTrackCylinder) printf("		dDis0 = %4.2f - sum0 = %4.2f\n", dDis0, sum0);

				// calculate distance between ellipse center points (ellipse 0-1 and 1-0)
				double dxi0 = tCylinders[i-1]->cylDef.x[0] - tCylinders[j-1]->cylDef.x[1];
				double dyi0 = tCylinders[i-1]->cylDef.y[0] - tCylinders[j-1]->cylDef.y[1];
				double disi0 = sqrt(pow(dxi0,2) + pow(dyi0,2));

				double dxi1 = tCylinders[i-1]->cylDef.x[1] - tCylinders[j-1]->cylDef.x[0];
				double dyi1 = tCylinders[i-1]->cylDef.y[1] - tCylinders[j-1]->cylDef.y[0];
				double disi1 = sqrt(pow(dxi1,2) + pow(dyi1,2));

				double dDis1 = fabs(disi0 - disi1);		// TODO wird nicht mehr gebraucht
				double sum1 = disi0 + disi1;

if(printTrackCylinder) printf("	Distance for cylinder inverse %u: dis = %4.2f - %4.2f\n", tCylinders[i-1]->object_id, disi0, disi1);
if(printTrackCylinder) printf("		dDis1 = %4.2f - sum1 = %4.2f\n\n", dDis1, sum1);

				// direct connections between ellipses are always shorter than intersected connections
				if(sum0 < sum1)  // non inversed ellipses
				{
					if(dis0 < 3.0*tCylinders[i-1]->cylDef.a[0] && dis1 < 3.0*tCylinders[i-1]->cylDef.a[1])		// TODO Threshold for "near"
					{
if(printTrackCylinder) printf("NOT INVERSED IS TRUE!\n");
// 						double trackDist = 0;

						// mark tCylinders as tracked
						tCylinders[i-1]->FormerTracked(tCylinders[j-1]->object_id);			// mark tCylinder[i-1] as tracked (age=0)
						tCylinders[j-1]->NextTracked(tCylinders[i-1]->object_id);				// mark old tCylinder[j-1] as tracked from tCylinder[i-1] (age=1)

						Vector2 groundCenterI = tCylinders[i-1]->cylDef.groundCenter;
						Vector2 groundCenterJ = tCylinders[j-1]->cylDef.groundCenter;

						// mark objects as tracked
						Objects(tCylinders[i-1]->object_id)->Tracked(tCylinders[j-1]->object_id, groundCenterJ);

						// calculate new tracking distance
						NewTrackDist(groundCenterI, groundCenterJ);
					}
				}
				else							// inversed ellipses
				{
					if(disi0 < 3.0*tCylinders[i-1]->cylDef.a[0] && disi1 < 3.0*tCylinders[i-1]->cylDef.a[1])
					{
if(printTrackCylinder) printf("INVERSED IS TRUE!\n");
// 						double trackDist = 0;

						// mark tCylinders as tracked
						tCylinders[i-1]->FormerTracked(tCylinders[j-1]->object_id);			// mark tCylinder[i-1] as tracked (age=0)
						tCylinders[j-1]->NextTracked(tCylinders[i-1]->object_id);				// mark old tCylinder[j-1] as tracked from tCylinder[i-1] (age=1)

						Vector2 groundCenterI = tCylinders[i-1]->cylDef.groundCenter;
						Vector2 groundCenterJ = tCylinders[j-1]->cylDef.groundCenter;

						// mark objects as tracked
						Objects(tCylinders[i-1]->object_id)->Tracked(tCylinders[j-1]->object_id, groundCenterJ);

						// calculate new tracking distance
						NewTrackDist(groundCenterI, groundCenterJ);
					}
				}
			}
		}
	}
if(printTrackCylinder) printf("\n");
}

/**
 *	@brief Try to track a cylinder by an ellipse
 */
void ObjectTracker::TrackEllipse2Cylinder()
{
if(printCyl) printf("\n#################################################\n");
	// get all cones with age=1 which could not be tracked to a cone with age=0
	for(unsigned j=tCylinders.Size(); j>0; j--)
	{
		if (tCylinders[j-1]->age == 1 && tCylinders[j-1]->id_next_tracked == UNDEF_ID)
		{
if(printCyl) printf("-----------------------------------------------------\nTrackEllipse2Cylinder(): cylinder %u with ellipse\n", tCylinders[j-1]->object_id);

//printf("Fehler? tCylinders.Size()=%u\n", tCylinders.Size());
			Array<unsigned> possibleEllipses;								// all possible ellipses, who may fit to the cone
			Array<double> posEllSig0;												// significance of the possible ellipses
			Array<double> posEllSig1;												// significance of the possible ellipses
 			Array<Vector2> posEllCenterDis0;								// Distance between center points
			Array<Vector2> posEllCenterDis1;								// Distance between center points

 			// get all ellipses
 			for (unsigned i=0; i<NumEllipses(); i++)				// use also masked ellipses
 			{
				// is the center point of the ellipse near to one of the center-points of the cylinder-ellipses
				Vector2 distance0;
				distance0.x = Ellipses(i)->x - tCylinders[j-1]->cylDef.x[0];
				distance0.y = Ellipses(i)->y - tCylinders[j-1]->cylDef.y[0];
				double dis0 = distance0.Norm();																	// distance to first ellipse of cylinder

				Vector2 distance1;
				distance1.x = Ellipses(i)->x - tCylinders[j-1]->cylDef.x[1];
				distance1.y = Ellipses(i)->y - tCylinders[j-1]->cylDef.y[1];
				double dis1 = distance1.Norm();																	// distance to second ellipse of cylinder

				// save ellipse as "near"
				if(dis0 < (2. * tCylinders[j-1]->cylDef.a[0]) || dis0 < (2. * tCylinders[j-1]->cylDef.a[1]) ||
					 dis1 < (2. * tCylinders[j-1]->cylDef.a[0]) || dis1 < (2. * tCylinders[j-1]->cylDef.a[1]))
				{
					// Save all possible ellipses
					possibleEllipses.PushBack(i);								// store ellipses
					posEllSig0.PushBack(100./dis0);							// store 100/dis as significance value
					posEllSig1.PushBack(100./dis1);							// store 100/dis as significance value
					posEllCenterDis0.PushBack(distance0);				// store Vector distance
					posEllCenterDis1.PushBack(distance1);				// store Vector distance
				}
			}

			// find the best ellipse
			double ref0 = 0.;
			double ref1 = 0.;
			unsigned bestEllipse0 = UNDEF_ID;			// which ellipse fits best to one of the cylinder ellipses
			unsigned bestEllipse1 = UNDEF_ID;			// which ellipse fits best to one of the cylinder ellipses
			unsigned side;												// fits ellipse[0] or [1] from cylinder better?
			for(unsigned k=0; k<possibleEllipses.Size(); k++)
			{
				// get the differences of size of the found ellipse to the cylinder-ellipses
				double dSize0 = fabs(tCylinders[j-1]->cylDef.a[0] - Ellipses(possibleEllipses[k])->a);
				double dSize1 = fabs(tCylinders[j-1]->cylDef.a[1] - Ellipses(possibleEllipses[k])->a);

				// get difference of angles
				double dPhi0 = fabs(tCylinders[j-1]->cylDef.phi[0] - Ellipses(possibleEllipses[k])->phi);
				double dPhi1 = fabs(tCylinders[j-1]->cylDef.phi[1] - Ellipses(possibleEllipses[k])->phi);
				if (dPhi0 > M_PI/2.) dPhi0 = M_PI - dPhi0;
				if (dPhi1 > M_PI/2.) dPhi1 = M_PI - dPhi1;

if(printCyl) printf("possible Ellipses 0: %u	- sig0: %4.2f - dSize0: %4.2f - s/b0: %4.2f - dPhi0: %4.2f - gesamt sig0: %4.2f\n", 
							possibleEllipses[k], posEllSig0[k], dSize0, posEllSig0[k]/dSize0, dPhi0, posEllSig0[k]/(dSize0*3*dPhi0));
if(printCyl) printf("possible Ellipses 1: %u	- sig1: %4.2f - dSize1: %4.2f - s/b1: %4.2f - dPhi1: %4.2f - gesamt sig1: %4.2f\n\n", 
							possibleEllipses[k], posEllSig1[k], dSize1, posEllSig1[k]/dSize1, dPhi1, posEllSig1[k]/(dSize1*3*dPhi1));

 				if(posEllSig0[k]/(dSize0*3*dPhi0) > ref0)
 				{
 					bestEllipse0 = k;
 					ref0 = posEllSig0[k]/(dSize0*3*dPhi0);
 				}
 				if(posEllSig1[k]/(dSize1*3*dPhi1) > ref1)
 				{
 					bestEllipse1 = k;
 					ref1 = posEllSig1[k]/(dSize1*3*dPhi1);
 				}
			}

if(printCyl) printf("	Best Ellipse 0 is: %u	 - ref: %4.2f\n", possibleEllipses[bestEllipse0], ref0);
if(printCyl) printf("	Best Ellipse 1 is: %u	 - ref: %4.2f\n\n", possibleEllipses[bestEllipse1], ref1);

if(printCyl) if(Ellipses(possibleEllipses[bestEllipse0])->cylinders.Size() != 0)
if(printCyl) 	printf("extEllipse has Cylinder: %u\n", Ellipses(possibleEllipses[bestEllipse0])->cylinders[0]);

			/// TODO einschub für funktion
			unsigned bestEllipse = UNDEF_ID;
			double ref;
			if(ref0 > ref1)
			{
				bestEllipse = bestEllipse0;
				ref = ref0;
				side = 0;
			}
			else
			{
				bestEllipse = bestEllipse1;
				ref = ref1;
				side = 1;
			}


			// get extEllipse of best ellipse
			double bestRef = 0.;
			unsigned bestLine;
			Vector2 bestLineDir;
			double bestLinePhi;
			bool hasExtEllipse = true;
			unsigned extEllipse = UNDEF_ID;
			if (bestEllipse != UNDEF_ID) extEllipse = Ellipses(possibleEllipses[bestEllipse])->extEllipse;

// printf("extEllipse-Normalise - ");
			if(extEllipse != UNDEF_ID)
			{
				// get longest extLine
				double reference = 10000.;		// only reference value without meaning
				for(unsigned i=0; i<ExtEllipses(extEllipse)->extLines.Size(); i++)
				{
					Vector2 end = Lines(ExtEllipses(extEllipse)->extLines[i])->point[Other(ExtEllipses(extEllipse)->extLinesEnd[i])];
					Vector2 vertex = ExtEllipses(extEllipse)->vertex[ExtEllipses(extEllipse)->extLinesVertex[i]];
					Vector2 lineDir = Normalise(end-vertex);
					double phi = PolarAngle(lineDir);

					// Calculate angle between ellipse and extLine
					Vector2 ellDir = Ellipses(ExtEllipses(extEllipse)->ellipse)->dir;
					double oAngle = acos(Dot(ellDir, lineDir));
					if(oAngle > M_PI/2.) oAngle = M_PI - oAngle;

					// calculate line length
					double lineLength = (vertex - end).Length();

					/// TODO TODO TODO TODO get line gap
					double lineGap = ExtEllipses(extEllipse)->extLinesGap[i];
if(printCyl) printf("lineGap: %4.2f\n", lineGap);

					// calculate reference angle
					double refAngle = lineGap*100*fabs(oAngle - (M_PI/2.))/lineLength;
if(printCyl) printf("	refAngle: %4.2f - %4.2f\n", refAngle, refAngle*lineGap);
					if(refAngle < reference)
					{
						reference = refAngle;
						bestRef = fabs(oAngle - (M_PI/2.));
						bestLine = ExtEllipses(extEllipse)->extLines[i];
						bestLineDir = Normalise(lineDir);
						bestLinePhi = phi;
					}
				}
				// get longest colLine
				for(unsigned i=0; i<ExtEllipses(extEllipse)->colLines.Size(); i++)
				{
					Vector2 end = Lines(ExtEllipses(extEllipse)->colLines[i])->point[Other(ExtEllipses(extEllipse)->colLinesEnd[i])];
					Vector2 vertex = ExtEllipses(extEllipse)->vertex[ExtEllipses(extEllipse)->colLinesVertex[i]];
					Vector2 lineDir = Normalise(end-vertex);
					double phi = PolarAngle(lineDir);
			
					// Calculate angle between ellipse and colLine
					Vector2 ellDir = Ellipses(ExtEllipses(extEllipse)->ellipse)->dir;
					double oAngle = acos(Dot(ellDir, lineDir));
					if(oAngle > M_PI/2.) oAngle = M_PI - oAngle;

					// calculate line length
					double lineLength = (vertex - end).Length();

					/// TODO TODO TODO TODO get line gap
					double lineGap = ExtEllipses(extEllipse)->colLinesGap[i];
if(printCyl) printf("lineGap: %4.2f\n", lineGap);

					// calculate reference angle
					double refAngle = lineGap*100*fabs(oAngle - (M_PI/2.))/lineLength;
if(printCyl) printf("	refAngle: %4.2f - %4.2f\n", refAngle, refAngle*lineGap);
					if(refAngle < reference)
					{
						reference = refAngle;
						bestRef = fabs(oAngle - (M_PI/2.));
						bestLine = ExtEllipses(extEllipse)->colLines[i];
						bestLineDir = Normalise(lineDir);
						bestLinePhi = phi;
					}
				}

if(printCyl) printf("	Direction of best extLine or colLine %u: %4.2f\n", bestLine, bestLinePhi);
			}
			else hasExtEllipse = false;


// printf("no\n");

if(printCyl) printf("Create cylinder? => ref = %4.2f\n", ref);
			// create cylinder from best ellipse
			if (ref > 5.0) 																								// TODO Threshold for Tracking Ellipses to Cones
			{
				// cone definition
				CylDef cylDef = tCylinders[j-1]->cylDef;

				cylDef.cylinder = UNDEF_ID;		// tracked cylinder

				Vector2 cylDir; // Direction of cylinder from "side"

				Array<Vector2> posEllCenterDis;
				if(side==0)			// ellipse fits better to cylinder-ellipse[0]
				{
					posEllCenterDis = posEllCenterDis0;									// Distance is distance to first ellipse center
					cylDir.x = cylDef.x[1] - cylDef.x[0];
					cylDir.y = cylDef.y[1] - cylDef.y[0];
					cylDir.Normalise();
				}
				else						// ellipse fits better to cylinder-ellipse[1]
				{
					posEllCenterDis = posEllCenterDis1;									// Distance is distance to second ellipse center
					cylDir.x = cylDef.x[0] - cylDef.x[1];
					cylDir.y = cylDef.y[0] - cylDef.y[1];
					cylDir.Normalise();
				}

if(printCyl) printf("	Direction of cylinder: %4.2f\n",  PolarAngle(cylDir));

				// if extEllipse has lines into the other direction, we have to change the ellipse-allocation
				bool track = true;
				bool changeSide = false;
				if (hasExtEllipse) 
				{
					double alpha = acos(Dot(cylDir, bestLineDir));
if(printCyl) printf("	Öffnungswinkel: %4.3f\n", alpha);
				
					if (M_PI-alpha < 0.5)																				// TODO threshold for changing sides
					{
if(printCyl) printf("		*********** Change Side ***********\n");
						changeSide = true;
					}
					else if(alpha > 0.5) 
					{
if(printCyl) printf(" FALSE FALSE FALSE\n");
						track = false;
					}
				}

				if(track)
				{
					if(changeSide && side == 0) posEllCenterDis = posEllCenterDis1;
					if(changeSide && side == 1) posEllCenterDis = posEllCenterDis0;
	
					// add vector from old ellipse-center-point to new
					cylDef.x[0] += posEllCenterDis[bestEllipse].x;
					cylDef.y[0] += posEllCenterDis[bestEllipse].y;
					cylDef.x[1] += posEllCenterDis[bestEllipse].x;
					cylDef.y[1] += posEllCenterDis[bestEllipse].y;
					cylDef.vertex[0][0].x += posEllCenterDis[bestEllipse].x;
					cylDef.vertex[0][0].y += posEllCenterDis[bestEllipse].y;
					cylDef.vertex[0][1].x += posEllCenterDis[bestEllipse].x;
					cylDef.vertex[0][1].y += posEllCenterDis[bestEllipse].y;
					cylDef.vertex[1][0].x += posEllCenterDis[bestEllipse].x;
					cylDef.vertex[1][0].y += posEllCenterDis[bestEllipse].y;
					cylDef.vertex[1][1].x += posEllCenterDis[bestEllipse].x;
					cylDef.vertex[1][1].y += posEllCenterDis[bestEllipse].y;
					cylDef.center.x += posEllCenterDis[bestEllipse].x;
					cylDef.center.y += posEllCenterDis[bestEllipse].y;
	
					// calculate direction of first ellipse to second
					cylDef.dir.x = cylDef.x[1] - cylDef.x[0];
					cylDef.dir.y = cylDef.y[1] - cylDef.y[0];
					cylDef.dir.Normalise();
	
					// calculate center point of cylinder
					cylDef.center.x = (cylDef.x[0] + cylDef.x[1])/2.;
					cylDef.center.y = (cylDef.y[0] + cylDef.y[1])/2.;
	
					/// if cylinder already exists, mark the found cylinder as tracked
					bool existingCylinder = false;
					unsigned exCyl = CheckExistingCylinder(cylDef.center);
					if(exCyl != UNDEF_ID)
					{
if(printCyl) printf("	=> Cylinder exists already => mark as tracked and break\n");
						// mark tCylinders as tracked
						for(unsigned l=0; l<tCylinders.Size(); l++)
							if(tCylinders[l]->id == exCyl && tCylinders[l]->age == 0) 
							{
								tCylinders[l]->FormerTracked(tCylinders[j-1]->object_id);					// mark tCylinder (age=0) as tracked
								tCylinders[j-1]->NextTracked(tCylinders[l]->object_id);						// mark old tCylinder[j-1] as tracked (age=1)
							}

						Vector2 groundCenterJ;																					/// TODO TODO ground Center ist hier überhaupt nicht bestimmt!!!!
						groundCenterJ = tCylinders[j-1]->cylDef.groundCenter;

						// mark objects as tracked
						unsigned objId;
						for(unsigned l=0; l<NumObjects(); l++)
							if (Objects(l)->gestalt_id == exCyl) objId = l;
						Objects(objId)->Tracked(tCylinders[j-1]->object_id, groundCenterJ);			// mark Object as tracked

						existingCylinder = true;	// do not make a new cylinder
					}

					if(!existingCylinder)
					{
if(printCyl) printf("MAKE NEW TRACKED CYLINDER: %u with %4.2f sig\n", NumObjects(), bestRef);

						// copy ground center and tracked ground cente
						if(cylDef.y[0] > cylDef.y[1])
						{
							cylDef.groundCenter.x = cylDef.x[0];
							cylDef.groundCenter.y = cylDef.y[0];
						}
						else
						{
							cylDef.groundCenter.x = cylDef.x[1];
							cylDef.groundCenter.y = cylDef.y[1];
						}
						cylDef.trackedCylinderGroundCenter = tCylinders[j-1]->cylDef.groundCenter;

						/// TODO TODO TODO TODO TODO TODO 
// 						if(CheckTrackDist(cylDef.groundCenter - cylDef.trackedCylinderGroundCenter))
// 						{
							// get 3D properties of tracked cylinder
							GetCylinder3DProperties(cylDef);
	
							// create new object
							NewGestalt(new Object(Gestalt::TRACKEDCYLINDER, UNDEF_ID, cylDef));
	
							// save cone for tracking
							tCylinders.PushBack(new TCylinder(NumObjects()-1, UNDEF_ID, cylDef));
	
							// mark tCones as tracked
							tCylinders[j-1]->NextTracked(NumObjects()-1);
							tCylinders[tCylinders.Size()-1]->FormerTracked(tCylinders[j-1]->object_id);
	
							// mark Object as tracked
							Objects(NumObjects()-1)->Tracked(tCylinders[j-1]->object_id, tCylinders[j-1]->cylDef.groundCenter);

 							if(!CheckTrackDist(cylDef.groundCenter, cylDef.trackedCylinderGroundCenter))	Objects(NumObjects()-1)->Mask(10002);
// 						}
					}
				}
			}
		}
	}
}


/**
 *	@brief Check if the tracked cylinder already exists. Returns UNDEF_ID when no cylinder is at this position or the id
 *	of the cube, if a cube exists.
 */
unsigned ObjectTracker::CheckExistingCylinder(Vector2 cp)
{
	for(unsigned i=0; i<NumCylinders(); i++)
		if(Cylinders(i)->masked == UNDEF_ID && (cp - Cylinders(i)->center).Length() < Cylinders(i)->radius) return i;
	return UNDEF_ID;
}

/**
 *	@brief Get all balls, which are not masked and estimate the properties (also 3D)
 */
void ObjectTracker::GetBalls()
{
  for(unsigned i=0; i<NumBalls(); i++)
  {
    if (Balls(i)->masked == UNDEF_ID)
    {
			BalDef bd;

			bd.ball = i;

			GetBallProperties(bd);

			// save balls for tracking
			tBalls.PushBack(new TBall(NumObjects(), i, bd));

			// create new object (BALL)
      NewGestalt(new Object(Gestalt::BALL, i, bd));
		}
	}
}


/**
 * @brief Calculate the properties of the ball and store it at BalDef
 * @param bd Ball definition, containing all properties of the ball. bd must contain the ball id.
 */
void ObjectTracker::GetBallProperties(BalDef &bd)
{
	bd.radius = Balls(bd.ball)->radius;
	bd.center = Balls(bd.ball)->center;

	bd.groundCenter.x = bd.center.x;
	bd.groundCenter.y = bd.center.y + bd.radius;

	GetBall3DProperties(bd);
}

/**
 * @brief Calculate the 3D-properties of the ball
 * @param bd Ball definition, containing all properties of the ball
 */
void ObjectTracker::GetBall3DProperties(BalDef &bd)
{
	// calculate cylinder 3D properties
	double dAboveGround = 0.;				// 3D points are above ground plane?

	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point

	tPointImg = cvPoint2D64f (bd.groundCenter.x, bd.groundCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	bd.groundCenter3D.x = tPointWorld3D.x;
	bd.groundCenter3D.y = tPointWorld3D.y;

	// TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO TODO 
	/// calculate radius in 3D !!!
	/// mxTools sollten die Höhe zwischen 2 Punkten berechnen können, wenn ein Punkt bekannt (am Boden) und ein weiterer genau
	/// genau über diesem Punkt ist.

	/// 3D-Radius: Radius von groundCenter auf der x-Koordinate auftragen. Dies sollte ungefähr gleich dem Radius sein!
	tPointImg = cvPoint2D64f (bd.groundCenter.x + bd.radius, bd.groundCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	bd.radius3D = tPointWorld3D.x - bd.groundCenter3D.x;
}

/**
 *	@brief Try to track the ball
 */
void ObjectTracker::TrackBall()
{
if(printTrackBall) printf("Track Ball:\n");

//   for(unsigned i=tCylinders.Size(); i>0; i--)
//   {
//     if (tCylinders[i-1]->age != 0) break;
// 
//     for(unsigned j=tCylinders.Size(); j>0; j--)
//     {
//       if (tCylinders[j-1]->age == 1 && tCylinders[j-1]->id_next_tracked == UNDEF_ID)
//       {
// 				// calculate distance between ellipse center points (ellipse 0-0 and 1-1)
// 				double dx0 = tCylinders[i-1]->cylDef.x[0] - tCylinders[j-1]->cylDef.x[0];
// 				double dy0 = tCylinders[i-1]->cylDef.y[0] - tCylinders[j-1]->cylDef.y[0];
// 				double dis0 = sqrt(pow(dx0,2) + pow(dy0,2));
// 
// 				double dx1 = tCylinders[i-1]->cylDef.x[1] - tCylinders[j-1]->cylDef.x[1];
// 				double dy1 = tCylinders[i-1]->cylDef.y[1] - tCylinders[j-1]->cylDef.y[1];
// 				double dis1 = sqrt(pow(dx1,2) + pow(dy1,2));
// 
// 				double dDis0 = fabs(dis0 - dis1);		// TODO wird nicht mehr gebraucht
// 				double sum0 = dis0 + dis1;
// 
// if(printTrackCylinder) printf("	Distance for cylinder %u: dis = %4.2f - %4.2f\n", tCylinders[i-1]->object_id, dis0, dis1);
// if(printTrackCylinder) printf("		dDis0 = %4.2f - sum0 = %4.2f\n", dDis0, sum0);
// 
// 				// calculate distance between ellipse center points (ellipse 0-1 and 1-0)
// 				double dxi0 = tCylinders[i-1]->cylDef.x[0] - tCylinders[j-1]->cylDef.x[1];
// 				double dyi0 = tCylinders[i-1]->cylDef.y[0] - tCylinders[j-1]->cylDef.y[1];
// 				double disi0 = sqrt(pow(dxi0,2) + pow(dyi0,2));
// 
// 				double dxi1 = tCylinders[i-1]->cylDef.x[1] - tCylinders[j-1]->cylDef.x[0];
// 				double dyi1 = tCylinders[i-1]->cylDef.y[1] - tCylinders[j-1]->cylDef.y[0];
// 				double disi1 = sqrt(pow(dxi1,2) + pow(dyi1,2));
// 
// 				double dDis1 = fabs(disi0 - disi1);		// TODO wird nicht mehr gebraucht
// 				double sum1 = disi0 + disi1;
// 
// if(printTrackCylinder) printf("	Distance for cylinder inverse %u: dis = %4.2f - %4.2f\n", tCylinders[i-1]->object_id, disi0, disi1);
// if(printTrackCylinder) printf("		dDis1 = %4.2f - sum1 = %4.2f\n\n", dDis1, sum1);
// 
// 				// direct connections between ellipses are always shorter than intersected connections
// 				if(sum0 < sum1)  // non inversed ellipses
// 				{
// 					if(dis0 < 3.0*tCylinders[i-1]->cylDef.a[0] && dis1 < 3.0*tCylinders[i-1]->cylDef.a[1])		// TODO Threshold for "near"
// 					{
// if(printTrackCylinder) printf("NOT INVERSED IS TRUE!\n");
// // 						double trackDist = 0;
// 
// 						// mark tCylinders as tracked
// 						tCylinders[i-1]->FormerTracked(tCylinders[j-1]->object_id);			// mark tCylinder[i-1] as tracked (age=0)
// 						tCylinders[j-1]->NextTracked(tCylinders[i-1]->object_id);				// mark old tCylinder[j-1] as tracked from tCylinder[i-1] (age=1)
// 
// 						Vector2 groundCenterI = tCylinders[i-1]->cylDef.groundCenter;
// 						Vector2 groundCenterJ = tCylinders[j-1]->cylDef.groundCenter;
// 
// 						// mark objects as tracked
// 						Objects(tCylinders[i-1]->object_id)->Tracked(tCylinders[j-1]->object_id, groundCenterJ);
// 
// 						// calculate new tracking distance
// 						NewTrackDist(groundCenterI, groundCenterJ);
// 					}
// 				}
// 				else							// inversed ellipses
// 				{
// 					if(disi0 < 3.0*tCylinders[i-1]->cylDef.a[0] && disi1 < 3.0*tCylinders[i-1]->cylDef.a[1])
// 					{
// if(printTrackCylinder) printf("INVERSED IS TRUE!\n");
// // 						double trackDist = 0;
// 
// 						// mark tCylinders as tracked
// 						tCylinders[i-1]->FormerTracked(tCylinders[j-1]->object_id);			// mark tCylinder[i-1] as tracked (age=0)
// 						tCylinders[j-1]->NextTracked(tCylinders[i-1]->object_id);				// mark old tCylinder[j-1] as tracked from tCylinder[i-1] (age=1)
// 
// 						Vector2 groundCenterI = tCylinders[i-1]->cylDef.groundCenter;
// 						Vector2 groundCenterJ = tCylinders[j-1]->cylDef.groundCenter;
// 
// 						// mark objects as tracked
// 						Objects(tCylinders[i-1]->object_id)->Tracked(tCylinders[j-1]->object_id, groundCenterJ);
// 
// 						// calculate new tracking distance
// 						NewTrackDist(groundCenterI, groundCenterJ);
// 					}
// 				}
// 			}
// 		}
// 	}
}

/**
 *	@brief Get all walls, which are not masked and estimate the properties (also 3D)
 */
void ObjectTracker::GetWalls()
{
  for(unsigned i=0; i<NumWalls(); i++)
  {
		if (Walls(i)->masked == UNDEF_ID)
		{
			WalDef wd;

			wd.wall = i;

			// get the cube-properties for the cube
			GetWallProperties(wd);

			// save cube for tracking
			TWall *w = new TWall(NumObjects(), i, wd);
			tWalls.PushBack(w);

			// create new object from cube
			NewGestalt(new Object(Gestalt::WALL, i, wd));
		}
  }
}

/**
 *	@brief Calculate the properties of the wall
 *	@param wd Wall definition, containing all properties of the wall
 */
void ObjectTracker::GetWallProperties(WalDef &wd)
{
	wd.corner = Walls(wd.wall)->corner;

	wd.borderPoints[0] = Walls(wd.wall)->borderPoints[0];
	wd.borderPoints[1] = Walls(wd.wall)->borderPoints[1];

	wd.points[0] = Walls(wd.wall)->points[0];
	wd.points[1] = Walls(wd.wall)->points[1];

	wd.groundCenter = (wd.borderPoints[0] + wd.borderPoints[1])/2.;

	wd.center = wd.groundCenter;		// center = groundCenter
	wd.radius = 0.;									// TODO no radius defined


	GetWall3DProperties(wd);
}

/**
 *	@brief Get the 3D properties of walls with respect to the camera-parameter
 *	@param wd Wall definition, containing all properties of the wall
 */
void ObjectTracker::GetWall3DProperties(WalDef &wd)
{
	// calculate wall 3D properties
	double dAboveGround = 0.;				// 3D points are above ground plane?

	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point

	// calculate ground center point
	tPointImg = cvPoint2D64f (wd.groundCenter.x, wd.groundCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	wd.groundCenter3D.x = tPointWorld3D.x;
	wd.groundCenter3D.y = tPointWorld3D.y;

	// calculate border points
	tPointImg = cvPoint2D64f (wd.borderPoints[0].x, wd.borderPoints[0].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	wd.borderPoints3D[0].x = tPointWorld3D.x;
	wd.borderPoints3D[0].y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (wd.borderPoints[1].x, wd.borderPoints[1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	wd.borderPoints3D[1].x = tPointWorld3D.x;
	wd.borderPoints3D[1].y = tPointWorld3D.y;

}


/**
 *	@brief Try to track old walls with new ones
 */
void ObjectTracker::TrackWall()
{
  for(unsigned i=tWalls.Size(); i>0; i--)
  {
    if (tWalls[i-1]->age != 0) break;

    for(unsigned j=tWalls.Size(); j>0; j--)
    {
      if (tWalls[j-1]->age == 1)
      {
				// calculate distance between exit center points
				double dx = tWalls[i-1]->walDef.groundCenter.x - tWalls[j-1]->walDef.groundCenter.x;
				double dy = tWalls[i-1]->walDef.groundCenter.y - tWalls[j-1]->walDef.groundCenter.y;
				double dis = sqrt(pow(dx,2) + pow(dy,2));

				if(dis < 2*maxTrackDist)
				{
					Vector2 groundCenterI; 
					groundCenterI.x = tWalls[i-1]->walDef.groundCenter.x;
					groundCenterI.y = tWalls[i-1]->walDef.groundCenter.y;
	
					Vector2 groundCenterJ;
					groundCenterJ.x = tWalls[j-1]->walDef.groundCenter.x;
					groundCenterJ.y = tWalls[j-1]->walDef.groundCenter.y;
	
					// mark tCones as tracked
					tWalls[i-1]->FormerTracked(tWalls[j-1]->object_id);
					tWalls[j-1]->NextTracked(tWalls[i-1]->object_id);
	
					// mark objects as tracked
					Objects(tWalls[i-1]->object_id)->Tracked(tWalls[j-1]->object_id, groundCenterJ);			// mark Object as tracked
	
					// calculate new tracking distance
					//NewTrackDist(groundCenterI, groundCenterJ);	// ARI: wall nicht geeignet, da center point immer in der Mitte des Bildes
				}
      }
    }
  }
}


/**
 *	@brief Get all exits, which are not masked and estimate the properties (also 3D)
 */
void ObjectTracker::GetExits()
{
  for(unsigned i=0; i<NumExits(); i++)
  {
		if (Exits(i)->masked == UNDEF_ID)
		{
			ExitDef ed;

			ed.exit = i;

			// get the cube-properties for the cube
			GetExitProperties(ed);

			// save cube for tracking
			TExit *e = new TExit(NumObjects(), i, ed);
			tExits.PushBack(e);

			// create new object from cube
			NewGestalt(new Object(Gestalt::EXIT_, i, ed));
		}
  }
}

/**
 *	@brief Calculate the properties of the exit
 *	@param ed Exit definition, containing all properties of an exit
 */
void ObjectTracker::GetExitProperties(ExitDef &ed)
{
	ed.exitLinesStart[0] = Exits(ed.exit)->exitLinesStart[0];
	ed.exitLinesStart[1] = Exits(ed.exit)->exitLinesStart[1];

	ed.exitLinesIntersection[0] = Exits(ed.exit)->exitLinesIntersection[0];
	ed.exitLinesIntersection[1] = Exits(ed.exit)->exitLinesIntersection[1];

	ed.groundCenter = (ed.exitLinesIntersection[0] + ed.exitLinesIntersection[1])/2.;

	ed.center = ed.groundCenter;		// center = groundCenter
	ed.radius = 0.;									// TODO no radius defined

	GetExit3DProperties(ed);
}

/**
 *	@brief Get the 3D properties of exits with respect to the camera-parameter
 *	@param ed Exit definition, containing all properties of an exit
 */
void ObjectTracker::GetExit3DProperties(ExitDef &ed)
{
	// calculate exit 3D properties
	double dAboveGround = 0.;				// 3D points are above ground plane?

	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point
 
	tPointImg = cvPoint2D64f (ed.groundCenter.x, ed.groundCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	ed.groundCenter3D.x = tPointWorld3D.x;
	ed.groundCenter3D.y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (ed.exitLinesIntersection[0].x, ed.exitLinesIntersection[0].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	ed.exitLinesIntersection3D[0].x = tPointWorld3D.x;
	ed.exitLinesIntersection3D[0].y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (ed.exitLinesIntersection[1].x, ed.exitLinesIntersection[1].y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	ed.exitLinesIntersection3D[1].x = tPointWorld3D.x;
	ed.exitLinesIntersection3D[1].y = tPointWorld3D.y;

// printf("3D-intersection: %4.2f - %4.2f - %4.2f - %4.2f\n", ed.exitLinesIntersection3D[0].x,
// 		ed.exitLinesIntersection3D[0].y, ed.exitLinesIntersection3D[1].x, ed.exitLinesIntersection3D[1].y);
}

/**
 *	@brief Try to track old exits with new ones
 */
void ObjectTracker::TrackExit()
{
  for(unsigned i=tExits.Size(); i>0; i--)
  {
    if (tExits[i-1]->age != 0) break;

    for(unsigned j=tExits.Size(); j>0; j--)
    {
      if (tExits[j-1]->age == 1)
      {
				// calculate distance between exit center points
				double dx = tExits[i-1]->exitDef.groundCenter.x - tExits[j-1]->exitDef.groundCenter.x;
				double dy = tExits[i-1]->exitDef.groundCenter.y - tExits[j-1]->exitDef.groundCenter.y;
				double dis = sqrt(pow(dx,2) + pow(dy,2));

				if(dis < 2*maxTrackDist)
				{
					Vector2 groundCenterI; 
					groundCenterI.x = tExits[i-1]->exitDef.groundCenter.x;
					groundCenterI.y = tExits[i-1]->exitDef.groundCenter.y;
	
					Vector2 groundCenterJ;
					groundCenterJ.x = tExits[j-1]->exitDef.groundCenter.x;
					groundCenterJ.y = tExits[j-1]->exitDef.groundCenter.y;
	
					// mark tCones as tracked
					tExits[i-1]->FormerTracked(tExits[j-1]->object_id);
					tExits[j-1]->NextTracked(tExits[i-1]->object_id);
	
					// mark objects as tracked
					Objects(tExits[i-1]->object_id)->Tracked(tExits[j-1]->object_id, groundCenterJ);			// mark Object as tracked
	
					// calculate new tracking distance
					NewTrackDist(groundCenterI, groundCenterJ);
				}
      }
    }
  }
}

/**
 *	@brief Calculate new tracking distince
 *	@param newCenter New center point of the object
 *	@param oldCenter Old center point of the object
 */
void ObjectTracker::NewTrackDist(Vector2 newCenter, Vector2 oldCenter)
{
	// calculate maxTrackDist
	double dist = Length(newCenter - oldCenter);
	if(dist > maxTrackDist) maxTrackDist = dist;

	// calculate 3D points of old and new center points
	double dAboveGround = 0.;				// 3D points are above ground plane?
	Vector2 newCenter3D;
	Vector2 oldCenter3D;

	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point
 
	tPointImg = cvPoint2D64f (newCenter.x, newCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	newCenter3D.x = tPointWorld3D.x;
	newCenter3D.y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (oldCenter.x, oldCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	oldCenter3D.x = tPointWorld3D.x;
	oldCenter3D.y = tPointWorld3D.y;

	// calculate distance3D
	Vector2 distance3D = newCenter3D - oldCenter3D;

	// save trackDist-vector
	vTrackDist3D.PushBack(distance3D);

	// save trackDist
	double dist3D = distance3D.Length();
	trackDist3D.PushBack(dist3D);

	// calculate maxTrackDist
	if(dist3D > maxTrackDist3D) maxTrackDist3D = dist3D;
	
	// calculate meanTrackDist
	meanTrackDist3D = 0;
	for(unsigned i=0; i<trackDist3D.Size(); i++)
		meanTrackDist3D += trackDist3D[i];
	if(trackDist3D.Size() != 0) meanTrackDist3D /= trackDist3D.Size();

	// calculate meanTrackDir
	meanTrackDir3D.x = 0.;
	meanTrackDir3D.y = 0.;
	for(unsigned i=0; i<vTrackDist3D.Size(); i++)
		meanTrackDir3D += vTrackDist3D[i];
 	if (meanTrackDir3D.x != 0. || meanTrackDir3D.y != 0.) meanTrackDir3D.Normalise();

	// calculate maxDirDiff
	maxDirDiff3D = 0.;
	for(unsigned i=0; i<vTrackDist3D.Size(); i++)
	{
		double dirDiff3D = 0.;
		if (vTrackDist3D[i].x != 0) dirDiff3D = acos(Dot(Normalise(vTrackDist3D[i]), meanTrackDir3D));
		if(dirDiff3D > maxDirDiff3D) maxDirDiff3D = dirDiff3D;
	}

if(printCheckTrDi) printf("distance3D.Length: %4.3f - distance3D: %4.2f - %4.2f  - maxDirDiff: %4.2f\n", distance3D.Length(), distance3D.x, distance3D.y, maxDirDiff3D);



// 	Vector2 trDi = newCenter - oldCenter;
// 	if (trDi.x == 0. && trDi.y == 0.)
// 	{
// 		printf("ObjectTracker::NewTrackDist: zero vector\n");
// 		return;
// 	}
// 
// 	// save trackDist-vector
// 	vTrackDist.PushBack(trDi);
// 
// 	// calculate meanTrackDist
// 	meanTrackDist = 0;
// 	for(unsigned i=0; i<trackDist.Size(); i++)
// 		meanTrackDist += trackDist[i];
// 	if(trackDist.Size() != 0) meanTrackDist /= trackDist.Size();
// 
// 	// calculate meanTrackDir
// 	meanTrackDir.x = 0.;
// 	meanTrackDir.y = 0.;
// 	for(unsigned i=0; i<vTrackDist.Size(); i++)
// 		meanTrackDir += vTrackDist[i];
//  	if (meanTrackDir.x != 0. || meanTrackDir.y != 0.) meanTrackDir.Normalise();
// 
// 	// calculate maxDirDiff
// 	maxDirDiff = 0.;
// 	for(unsigned i=0; i<vTrackDist.Size(); i++)
// 	{
// 		double dirDiff = acos(Dot(Normalise(vTrackDist[i]), meanTrackDir));
// 		if(dirDiff > maxDirDiff) maxDirDiff = dirDiff;
// 	}

}

/**
 *	@brief Check, if tracking distance is possible.
 *	@param newCenter New center point of the object
 *	@param oldCenter Old center point of the object
 *	@return Returns true if tracking is possible, else false.
 */
bool ObjectTracker::CheckTrackDist(Vector2 newCenter, Vector2 oldCenter)
{
	// calculate 3D points of old and new center points
	double dAboveGround = 0.;				// 3D points are above ground plane?
	Vector2 newCenter3D;
	Vector2 oldCenter3D;

	CvPoint2D64f tPointImg;					// image point in 2D
	CvPoint3D64f tPointWorld3D;			// 3d-world point
 
	tPointImg = cvPoint2D64f (newCenter.x, newCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	newCenter3D.x = tPointWorld3D.x;
	newCenter3D.y = tPointWorld3D.y;

	tPointImg = cvPoint2D64f (oldCenter.x, oldCenter.y);
	m_cCamModel.Image2World (tPointImg, &tPointWorld3D);
	m_cCamModel.World2GroundPlane (tPointWorld3D, &tPointWorld3D, dAboveGround);
	oldCenter3D.x = tPointWorld3D.x;
	oldCenter3D.y = tPointWorld3D.y;

	// calculate distance3D
	Vector2 distance3D = newCenter3D - oldCenter3D;

if(printCheckTrDi) printf("distance3D.Length: %4.3f\n", Length(distance3D));


	if (distance3D.x == 0. && distance3D.y == 0.) 
	{
		printf("ObjectTracker::CheckTrackDist: zero vector\n");
		return -1.;
	}

	// calculate difference in length
	double lDiff = fabs(Length(distance3D) - meanTrackDist3D);

	// calculate difference in angle
	double aDiff = acos(Dot(Normalise(distance3D), meanTrackDir3D));

	// lDiff=>0 => aDiff-beschränkung größer
	double restriction = M_PI/(meanTrackDist3D*100 + 1);
if(printCheckTrDi) printf("	meanTrackDist3D: %4.3f - restriction: %4.3f\n", meanTrackDist3D, restriction);

	// difference to maxDirDiff
	double dirDiff = aDiff - maxDirDiff3D;

if(printCheckTrDi) printf("	CheckTrackDist: lDiff: %4.3f - aDiff: %4.3f - dirDiff: %4.3f\n", lDiff, aDiff, dirDiff);

	if(lDiff < meanTrackDist3D && aDiff < restriction /*&& dirDiff < 0.*/)					/// TODO Threshold for "true" track distance
	{
if(printCheckTrDi) printf("		=> true\n");
 		return true;
	}
	else 
	{
if(printCheckTrDi) printf("		=> false\n");
if(printCheckTrDi) printf("		trackDistance: %4.3f\n", Length(distance3D));
 		return false;
	}

// 	return true;
}

/**
 *	@brief PrintTCubes()
 */
void ObjectTracker::PrintTCubes()
{
	for(unsigned i=0; i<tCubes.Size(); i++)
		printf("object-id: %u - age: %u - id_former_tracked: %u - id_next_tracked: %u\n", 
				tCubes[i]->object_id, tCubes[i]->age, tCubes[i]->id_former_tracked, tCubes[i]->id_next_tracked);
	printf("-----------------------------------------------------------------------------------\n");
}

/**
 *	@brief PrintTCylinders()
 */
void ObjectTracker::PrintTCylinders()
{
	for(unsigned i=0; i<tCylinders.Size(); i++)
	{
		printf("object-id: %u - id: %u - age: %u - id_former_tracked: %u - id_next_tracked: %u\n",
					tCylinders[i]->object_id, tCylinders[i]->id, tCylinders[i]->age, tCylinders[i]->id_former_tracked, 	tCylinders[i]->id_next_tracked);
//		printf("ellipse-center: %4.2f - %4.2f  --- second: %4.2f - %4.2f\n", 
//			tCylinders[i]->cylDef.x[0], tCylinders[i]->cylDef.y[0], tCylinders[i]->cylDef.x[1], tCylinders[i]->cylDef.y[1]);
	}
	printf("-----------------------------------------------------------------------------------\n");
}

/**
 *	@brief ShowTCubes()
 */
void ObjectTracker::ShowTCubes()
{
	for(unsigned i=0; i<tCubes.Size(); i++)
	{
		if(tCubes[i]->age == 1 && tCubes[i]->id_next_tracked == UNDEF_ID)
		{		
printf("################################# Show tCubes !!!!! ##########################################\n");
			CubeDef cd;
			cd.type = Gestalt::TRACKEDCUBE;

			// get cube corner_points
			cd.corner_points[0][0] = tCubes[i]->cubeDef.corner_points[0][0];
			cd.corner_points[0][1] = tCubes[i]->cubeDef.corner_points[0][1];
			cd.corner_points[1][0] = tCubes[i]->cubeDef.corner_points[1][0];
			cd.corner_points[1][1] = tCubes[i]->cubeDef.corner_points[1][1];
			cd.corner_points[2][0] = tCubes[i]->cubeDef.corner_points[2][0];
			cd.corner_points[2][1] = tCubes[i]->cubeDef.corner_points[2][1];
			cd.corner_points[3][0] = tCubes[i]->cubeDef.corner_points[3][0];
			cd.corner_points[3][1] = tCubes[i]->cubeDef.corner_points[3][1];

			// get cube properties
			GetCubeProperties(cd);

			NewGestalt(new Object(Gestalt::TRACKEDCUBE, UNDEF_ID, cd));
		}
	}

}
}

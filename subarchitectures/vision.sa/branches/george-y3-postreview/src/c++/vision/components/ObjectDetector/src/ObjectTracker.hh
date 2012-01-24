/**
 * @file ObjectTracker.hh
 * @author Andreas Richtsfeld
 * @date April 2008
 * @version 0.1
 * @brief Tracking of detected objects
 **/

#ifndef Z_OBJECT_TRACKER_HH
#define Z_OBJECT_TRACKER_HH

#include <mxCameraModel.h>

#include "GestaltPrinciple.hh"
#include "CubeDefinition.hh"
#include "ConeDefinition.hh"
#include "CylinderDefinition.hh"
#include "BallDefinition.hh"
#include "WallDefinition.hh"
#include "ExitDefinition.hh"
#include "stdio.h"
#include "TObject.hh"


namespace Z
{

/**
 *	@brief Class ObjectTracker
 */
class ObjectTracker : public GestaltPrinciple
{
private:

/**
 *	@brief Cubes to track
 */
class TCube
{
  private:

  public:
  unsigned object_id;										// id of the object
  unsigned id;													// id of the cube																// TODO nicht immer möglich => tracked Cubes? => veraltet!!! löschen
  unsigned age;													// age of the stored TCube (in steps)
  unsigned id_former_tracked;						// id of the object from last image (age+1)
  unsigned id_next_tracked;							// id of the object from next image (age-1)

	CubeDef cubeDef;											// TODO TODO TODO Alle Infos umstellen auf CubeDef!!!!

  TCube(unsigned o_id, unsigned i, CubeDef cubDef);
  void FormerTracked(unsigned i);				// mark former cube (age+1)
	void NextTracked(unsigned id);				// mark next cube (age-1)
};

/**
 *	@brief Cones to track
 */
class TCone
{
  private:

  public:
  unsigned object_id;										// id of the object
	unsigned id;													// id of the cone																// TODO wie oben
  unsigned age;													// age of the stored TCone (in steps)
  unsigned id_former_tracked;						// id of the tracked cone
  unsigned id_next_tracked;							// id of the object from next image (age-1)

	ConeDef coneDef;											// cone properties															// TODO sollten eigentlich alle xxxProperties heißen!

  TCone(unsigned o_id, unsigned i, ConeDef cd);
  void FormerTracked(unsigned i);	// mark tracked cone (age+1)
	void NextTracked(unsigned id);				// mark as tracked (age-1)
};

/**
 *	@brief Cylinders to track
 */
class TCylinder 	// Cylinders to track
{
  private:

  public:
	unsigned object_id;										// id of the object
  unsigned id;													// id of the cylinder
  unsigned age;													// age of the stored TCylinder (in steps)
  unsigned id_former_tracked;						// id of the tracked cylinder
  unsigned id_next_tracked;							// id of the object from next image (age-1)

	CylDef cylDef;												// cylinder properties

  TCylinder(unsigned o_id, unsigned i, CylDef cd);
  void FormerTracked(unsigned i);				// mark former cylinder (age+1)
	void NextTracked(unsigned id);				// mark next cylinder (age-1)

};

/**
 *	@brief Balls to track
 */
class TBall
{
  private:

  public:
	unsigned object_id;										// id of the object
  unsigned id;													// TODO id of the wall: wie oben
  unsigned age;													// age of the stored TBall (in steps)
  unsigned id_former_tracked;						// id of the tracked ball
  unsigned id_next_tracked;							// id of the object from next image (age-1)

	BalDef balDef;												// ball properties

  TBall(unsigned o_id, unsigned i, BalDef bd);
  void FormerTracked(unsigned i);				// mark former ball (age+1)
	void NextTracked(unsigned id);				// mark next ball (age-1)
};

/**
 *	@brief Walls to track
 */
class TWall
{
  private:

  public:
	unsigned object_id;										// id of the object
  unsigned id;													// id of the wall
  unsigned age;													// age of the stored TWall (in steps)
  unsigned id_former_tracked;						// id of the tracked wall
  unsigned id_next_tracked;							// id of the object from next image (age-1)

	WalDef walDef;												// wall properties

  TWall(unsigned o_id, unsigned i, WalDef wd);
  void FormerTracked(unsigned i);				// mark former wall (age+1)
	void NextTracked(unsigned id);				// mark next wall (age-1)
};

/**
 *	@brief Exits to track
 */
class TExit
{
  private:

  public:
	unsigned object_id;										// id of the object
  unsigned id;													// id of the wall
  unsigned age;													// age of the stored TWall (in steps)
  unsigned id_former_tracked;						// id of the tracked wall
  unsigned id_next_tracked;							// id of the object from next image (age-1)

	ExitDef exitDef;											// wall properties

  TExit(unsigned o_id, unsigned i, ExitDef ed);
  void FormerTracked(unsigned i);				// mark former exits (age+1)
	void NextTracked(unsigned id);				// mark next exits (age-1)
};

	// ********** Class ObjectTracker: private ************ //
	bool printRect;												// print tracking of rectangle infos TODO delete
	bool printTrackCylinder;							// print tracking of cylinders TODO delete
	bool printTrackBall;									// print tracking of balls TODO delete
	bool printCyl;												// print tracking of cylinders from ellipses TODO delete
	bool printCheckTrDi; 									// print CheckTrackDist TODO delete

  bool firstCall;												///< First call of OperateNonIncremental: init cam model for 3D estimation
  bool trackCubes;											// enable/disable tracking
	bool trackFlap2Cube;
  bool trackRect2Cube;
  bool trackCones;
  bool trackCylinders;
	bool trackBalls;

  Array<TCube*> tCubes;									// cubes to track
  Array<TCone*> tCones;									// cones to track
  Array<TCylinder*> tCylinders;					// cylinders to track
	Array<TBall*> tBalls;									// balls to track
  Array<TWall*> tWalls;									// walls to track
	Array<TExit*> tExits;									// exits to track
	
	string confFile;											// configuration file for mxTools
  mx::CCameraModel m_cCamModel;					// camera model
  CvSize m_tImgSize;										// CV image size
	bool getCamParamsFromVC;							// camera parameters from vision core (or file)

	// Age of objects
	unsigned maxAge;											// maximum age for tracking
  void AgeTObjects(unsigned maxAge);		// Call to age all tObjects by one
  void AgeTCubes(unsigned maxAge);
  void AgeTCones(unsigned maxAge);
  void AgeTCylinders(unsigned maxAge);
	void AgeTBalls(unsigned maxAge);
  void AgeTWalls(unsigned maxAge);
  void AgeTExits(unsigned maxAge);
	void AgeMasked();

	// Tracking distances
  double maxTrackDist;									// maximum tracking distance (default = 25)
	Array<Vector2> vTrackDist3D;					// tracking distance vectors
	Array<double> trackDist3D;						// tracking distances (length)
	double meanTrackDist3D;								// mean tracking distances
  double maxTrackDist3D;								// maximum tracking distance (default = 25)
	Vector2 meanTrackDir3D;								// mean tracking direction (angle)
	double maxDirDiff3D; 									// maximum tracking direction (angle in rad)

	void NewTrackDist(Vector2 newCenter, Vector2 oldCenter);
	bool CheckTrackDist(Vector2 newCenter, Vector2 oldCenter);

  void InitCamModel();									// initialisation of camera model at start
	
	/// CUBES ///
  void GetCubes();
	void GetCubeProperties(CubeDef &cd);
	void GetCube3DProperties(CubeDef &cd);
	void RecalculateCube3DProperties(unsigned trktCube, CubeDef &cd);
	void TrackCube();
  void TrackRect2Cube();
  void TrackFlap2Cube();
  unsigned SaveTrackedCube(CubeDef cubeDef);
	unsigned CheckExistingCube(Vector2 cp);

	/// CONES ///
	void GetCones();
  void GetConeProperties(ConeDef &cd);
  void GetCone3DProperties(ConeDef &cd);
  void TrackCone();
	void TrackEllipse2Cone();
	unsigned CheckExistingCone(Vector2 cp);

	/// CYLINDERS ///
 	void GetCylinders();
  void GetCylinderProperties(CylDef &cd);
	void GetCylinder3DProperties(CylDef &cd);
	void TrackCylinder();
	void TrackEllipse2Cylinder();
	unsigned CheckExistingCylinder(Vector2 cp);

	/// BALLS ///
 	void GetBalls();
  void GetBallProperties(BalDef &bd);
	void GetBall3DProperties(BalDef &bd);
	void TrackBall();
	unsigned CheckExistingBall(Vector2 cp);

	/// WALLS ///
 	void GetWalls();
  void GetWallProperties(WalDef &wd);
  void GetWall3DProperties(WalDef &wd);
	void TrackWall();

	/// EXITS ///
 	void GetExits();
  void GetExitProperties(ExitDef &ed);
  void GetExit3DProperties(ExitDef &ed);
	void TrackExit();

	/// Show T-Objects ///
	void PrintTCubes();
	void PrintTCylinders();
	void ShowTCubes();

public: 

  ObjectTracker(Config *cfg);
	virtual void Mask();
  virtual void InformNewGestalt(Gestalt::Type type, unsigned idx);
	virtual void OperateNonIncremental();
};

}

#endif

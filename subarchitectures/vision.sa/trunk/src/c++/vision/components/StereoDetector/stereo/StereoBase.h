/**
 * @file StereoBase.h
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Base class for stereo caculation of Gestalts, with class Vertex2D, Surf2D, Vertex3D and Surf3D.
 */

#ifndef Z_STEREO_BASE_HH
#define Z_STEREO_BASE_HH

#ifdef HAVE_CAST
  #include <VisionData.hpp>
#endif

#include <vector>
#include "Vector.hh"
#include "StereoCamera.hh"
#include "Line.hh"
#include "Ellipse.hh"
#include "Closure.hh"
#include "FlapAri.hh"
#include "Cube.hh"
#include "Draw.hh"


namespace Z
{
/// TODO Was für Thresholds sind das genau: Beschreiben und alle anderen herausziehen: Surf u. Point
// These are some tuning parameters to filter out "bad" surface hypotheses.
// These might need adjusting to a specific use case.

// maximum allowed angle of one vertex normal to the mean of all vertex
// normals, e.g. 15 deg = pi/12 = 0.2618
static const double SC_MAX_NORMAL_DEVIATION = 0.50;

// minimum required cirumference of a surface in [m]
static const double SC_MIN_CIRC = 0.060;

// maximum allowed side length of a surface (wrong matches often
// tend to produce impossibly long, thing surfaces)
const double SC_MAX_LENGTH = 0.5;  // in [m]

// maximum allowed vertical deviation of line based stereo for all surface points
static const double SC_MAX_DELTA_V_SURF = 10.;

// maximum allowed vertical deviation of line based stereo for points
static const double SC_MAX_DELTA_V_POINT = 10.;

// minimum disparity (distance to point must be higher than this value)
static const double SC_MIN_DISPARITY = 0.;

// Space of interest (SOI) check.
// Sanity check for points: max. and min. distances in x,y,z-direction, relative to
// the camera
static const double SC_MIN_DIST_X = -3.;	// 3m to the left
static const double SC_MAX_DIST_X =  3.;	// 3m to the right			/// TODO TODO TODO MAX-MIN richtig?
static const double SC_MIN_DIST_Y = -3.;	// 3m up
static const double SC_MAX_DIST_Y =  3.;	// 3m down
static const double SC_MIN_DIST_Z =  0.;	// 0m away
static const double SC_MAX_DIST_Z =  4.;	// 4m away !!! (do not consider points farer than 4m away!



//----------------------------------------------------------------//
//-------------------------- Vertex2D ----------------------------// 
//----------------------------------------------------------------//
/**
 * @class Vertex2D
 * @brief A 2D vertex class to store and manipulate vs3 vertices.
 */
class Vertex2D
{
public:
  bool is_valid;					///< validation parameter
  Vector2 p;							///< point
  Vector2 pr;							///< rectified point
 
	void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
	void Draw();
};

//----------------------------------------------------------------//
//-------------------------- Surface 2D --------------------------//
//----------------------------------------------------------------//
/**
 * @class Surf2D
 * @brief A 2D surface class to store and manipulate vs3 surfaces.
 * (for closures, rectangles, flaps, cubes ...)
 */
class Surf2D
{
public:
  bool is_valid;					///< validation parameter
  vector<Vector2> p;			///< original (distorted, unrectified) points
  vector<Vector2> pr;			///< rectified points

  Surf2D() {is_valid = false;}
  Surf2D(Closure *clos) {Init(clos);}
	Surf2D(Rectangle *rectangle) {Init(rectangle);}
//	Surf2D(Cube *cube) {Init(cube, int side);}								TODO TODO /// das funktioniert mit den Seiten nicht!!!

  void Init(Closure *clos);
	void Init(Rectangle *rectangle);
	void Init(Cube *cube, int side);

  void ShiftPointsLeft(unsigned offs);
	void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
	void Draw(unsigned detail);
};

//--------------------------------------------------------------//
//-------------------------- Vertex3D --------------------------//
//--------------------------------------------------------------//
/**
 * @brief Vertex
 */
class Vertex3D
{
public:
  Vector3 p;						///< position vector
  Vector3 n;						///< normal vector

private:
	bool SanityOK();

public:
  bool Reconstruct(StereoCamera *stereo_cam, Vertex2D &left, Vertex2D &right);
	double Distance(Vertex3D point);
};

//----------------------------------------------------------------//
//-------------------------- Surf3D ------------------------------//
//----------------------------------------------------------------//
/**
 * @class Surf3D
 * @brief Class for 3D surfaces.
 */
class Surf3D
{
public:
  Array<Vertex3D> vertices;

private:
  bool NormalsOK();
  bool SizeOK();
  bool SanityOK();
  void RefineVertices();

public:
  bool Reconstruct(StereoCamera *stereo_cam, Surf2D &left, Surf2D &right, bool refine);
};

//----------------------------------------------------------------//
//-------------------------- StereoBase --------------------------//
//----------------------------------------------------------------//
/**
 * @class StereoBase
 * @brief Base class for all stereo matching classes.
 */
class StereoBase
{
public:
  enum Type
  {
    STEREO_LJUNCTION,
    STEREO_ELLIPSE,
    STEREO_CLOSURE,
    STEREO_RECTANGLE,
    STEREO_FLAP,
    STEREO_FLAP_ARI,
    STEREO_CUBE,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };						///< Type of stereo Gestalts for matching

  VisionCore *vcore[2];				///< Left and right vision core
  StereoCamera *stereo_cam;			///< Stereo camera parameters

  struct PruningParameter			///< Parameters, when pruned image will be processed at stereo core
  {
    bool pruning;				///< Pruned image delivered
    int offsetX;				///< Offset x-coordinate
    int offsetY;				///< Offset y-coordinate
    int scale;					///< Scale between original and pruned image
  };
  PruningParameter pPara;			///< Pruning parameters of an image.

private:
  bool enabled;					///< Enabled / disabled Stereo-Gestalt
  bool masking;					///< TODO 

protected:
	Type type;				///< StereoBase Type

public:
  static const char* TypeName(Type t);
  static Type EnumType(const char *type_name);

	StereoBase();
	void EnablePrinciple(bool status);
	bool IsEnabled() {return enabled;}

  double MatchingScoreSurf(Surf2D &left_surf, Surf2D &right_surf, unsigned &match_offs);			/// TODO Gehört eigentlich zu den Stereo's oder zu 2D-3D's?
  double MatchingScorePoint(Vertex2D &left_point, Vertex2D &right_point);											/// TODO Gehört eigentlich zu den Stereo's oder zu 2D-3D's?
	

	// virtual functions for the stereo classes.
	virtual int NumStereoMatches() = 0;
#ifdef HAVE_CAST_HERE
	virtual bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id) = 0;
#endif
	virtual void Draw(int side, bool masked = false) {}																					/// TODO Sollten alle pure virtual (=0) sein.
	virtual void DrawMatched(int side, bool single, int id, int detail) = 0;
	virtual void Process() = 0;
	virtual void Process(int oX, int oY, int sc) = 0;
	virtual void ClearResults() {}

};

}

#endif

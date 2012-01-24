/**
 * @file StereoTypes.h
 * @author Andreas Richtsfeld
 * @date Januray 2011
 * @version 0.1
 * @brief Base class for stereo calculated features.
 */

#ifndef Z_STEREO_TYPES_HH
#define Z_STEREO_TYPES_HH

#include "StereoCamera.h"

#include <vector>
#include "Vector.hh"
#include "Draw.hh"
#include "Line.hh"
#include "Closure.hh"
#include "Rectangle.hh"
#include "FlapAri.hh"
#include "Cube.hh"



namespace Z
{

 
/// TODO Was für Thresholds sind das genau: Beschreiben und alle anderen herausziehen: Surf u. Point
/// Gehören die hier wirklich her => Es sind auch matching thresholds in der StereoBase.h
// These are some tuning parameters to filter "bad" surface hypotheses.
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
static const double SC_MAX_DELTA_V_POINT = 5.;

// minimum disparity (distance to point must be higher than this value)
static const double SC_MIN_DISPARITY = 0.2;

// Space of interest (SOI) check.
// Sanity check for points: max. and min. distances in x,y,z-direction, relative to
// the camera
static const double SC_MIN_DIST_X = -2.;	// 3m to the left
static const double SC_MAX_DIST_X =  2.;	// 3m to the right			/// TODO TODO TODO MAX-MIN richtig?
static const double SC_MIN_DIST_Y = -2.;	// 3m up
static const double SC_MAX_DIST_Y =  2.;	// 3m down
static const double SC_MIN_DIST_Z =  0.;	// 0m away
static const double SC_MAX_DIST_Z =  3.;	// 3m away !!! (do not consider points farer than 4m away!


//----------------------------------------------------------------//
//-------------------------- HelpLine ----------------------------// 
//----------------------------------------------------------------//
/**					// TODO Wo wird HelpLine gebraucht => kann man löschen?
 * @brief HelpLine is used to prune short lines from closures or rectangles without destroying the Gestalt.
 * 
 */
struct HelpLine
{
  Vector2 p;             ///< some point on the line
  Vector2 d;             ///< direction of the line
  HelpLine(float px, float py, float dx, float dy) : p(px, py), d(dx, dy) {}
};


//----------------------------------------------------------------//
//-------------------------- Vertex2D ----------------------------// 
//----------------------------------------------------------------//
/**
 * @class Vertex2D
 * @brief A 2D vertex class to store and manipulate vs3 vertices.
 */
class Vertex2D
{
private:
  bool is_valid;         ///< validation parameter
  bool rectified_valid;  ///< rectified values are valid

public:
  Vector2 p;             ///< point
  Vector2 pr;            ///< rectified point

  void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  void Draw();
  bool IsRectified() {return rectified_valid;}
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
  bool is_valid;         ///< validation parameter
  vector<Vector2> p;     ///< original (distorted, unrectified) points
  vector<Vector2> pr;    ///< rectified points

  Surf2D() {is_valid = false;}
  Surf2D(vector<Vector2> points) {Init(points);}
  Surf2D(Closure *clos) {Init(clos);}
  Surf2D(Rectangle *rectangle) {Init(rectangle);}
												/// TODO Initialisierung sollte im StereoCube sein!
  void Init(vector<Vector2> points);
  void Init(Closure *clos);
  void Init(Rectangle *rectangle);
  void Init(Cube *cube, int side);								/// TODO sollte wegfallen: Siehe oben

  void ShiftPointsLeft(unsigned offs);
  void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  void Draw(unsigned detail);
};

//--------------------------------------------------------------//
//-------------------------- Vertex3D --------------------------//
//--------------------------------------------------------------//
/**
 * @brief Vertex in 3D
 */
class Vertex3D
{
public:
  VEC::Vector3 p;            ///< position vector
  VEC::Vector3 n;            ///< normal vector

private:
  bool SanityOK();

public:
  bool Reconstruct(cast::StereoCamera *stereo_cam, Vertex2D &left, Vertex2D &right);
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
  bool Reconstruct(cast::StereoCamera *stereo_cam, Surf2D &left, Surf2D &right, bool refine);
};


}

#endif

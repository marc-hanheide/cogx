/**
 * @file StereoBase.h
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Base class for stereo caculation of Gestalts, with class Vertex, TmpSurf and Surf3D.
 */

#ifndef Z_STEREO_BASE_HH
#define Z_STEREO_BASE_HH

#include <VisionData.hpp>
#include "StereoCamera.hh"

#include <vector>
#include "Vector2.hh"
#include "Vector3.hh"

#include "Line.hh"
#include "Ellipse.hh"
#include "Closure.hh"
#include "FlapAri.hh"
#include "Cube.hh"
#include "Draw.hh"


namespace Z
{

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



/**
 * @brief Vertex
 */
class Vertex
{
public:
  Vector3 p;						///< position
  Vector3 n;						///< TODO Normale ??? 
};

//----------------------------------------------------------------//
//-------------------------- TmpSurface --------------------------//
//----------------------------------------------------------------//
/**
 * @class TmpSurf
 * @brief Class for surfaces.
 */
class TmpSurf
{
public:
//  unsigned id;						///< ID of the surface								// TODO Which surface id???? => Es wurden clos-IDs verwendet => jetzt? Braucht man wozu?
  bool is_valid;					///< validation parameter
  vector<Vector2> p;			///< original (distorted, unrectified) points
  vector<Vector2> pr;			///< rectified points

  TmpSurf() {/*id = UNDEF_ID; */is_valid = false;}
  TmpSurf(Closure *clos) {Init(clos);}
	TmpSurf(Rectangle *rectangle) {Init(rectangle);}
//	TmpSurf(Ellipse *ellipse) {Init(ellipse);}											/// TODO wird diese Funktion aufgerufen
//	TmpSurf(Cube *cube) {Init(cube, int side);}											/// das funktioniert mit den Seiten nicht.

  void Init(Closure *clos);
	void Init(Rectangle *rectangle);
	void Init(Ellipse *ell);
	void Init(Cube *cube, int side);

  void ShiftPointsLeft(unsigned offs);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
	void Draw(RGBColor col);
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
  Array<Vertex> vertices;

private:
  bool NormalsOK();
  bool SizeOK();
  bool SanityOK();
  void RefineVertices();

public:
  bool Reconstruct(StereoCamera *stereo_cam, TmpSurf &left, TmpSurf &right, bool refine);
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
		STEREO_ELLIPSE,
		STEREO_CLOSURE,
		STEREO_RECTANGLE,
		STEREO_FLAP,
		STEREO_FLAP_ARI,
		STEREO_CUBE,
    MAX_TYPE,
    UNDEF = MAX_TYPE
  };																	///< Type of stereo Gestalts for matching

  VisionCore *vcore[2];								///< Left and right vision core
	StereoCamera *stereo_cam;						///< Stereo camera parameters

private:
	bool enabled;												///< enabled / disabled Stereo-Gestaltg

protected:
	Type type;

public:
  static const char* TypeName(Type t);
  static Type EnumType(const char *type_name);

	StereoBase();

	// functions to calculate stereo surfaces.
//   unsigned FindMatchingSurf(TmpSurf &left_surf, Array<TmpSurf> &right_surfs, unsigned l);
//   void MatchSurfaces(Array<TmpSurf> &left_surfs, Array<TmpSurf> &right_surfs, int &matches);
  double MatchingScoreSurf(TmpSurf &left_surf, TmpSurf &right_surf, unsigned &match_offs);
//   void Calculate3DSurfs(Array<TmpSurf> &left_surfs, Array<TmpSurf> &right_surfs, int &flapMatches, Array<Surf3D> &surf3ds);

	void EnablePrinciple(bool status);
	bool IsEnabled() {return enabled;}

	// virtual functions for the stereo classes.
	virtual int NumStereoMatches() = 0;
	virtual bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id) = 0;
	virtual void Draw(int side) {}
	virtual void DrawMatched(int side) {}
	virtual void Process() {}
	virtual void ClearResults() {}

};

}

#endif

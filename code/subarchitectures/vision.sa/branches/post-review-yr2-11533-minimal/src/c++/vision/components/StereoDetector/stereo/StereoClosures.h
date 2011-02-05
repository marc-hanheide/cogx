/**
 * @file StereoClosures.h
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Stereo calculation of closures.
 */

#ifndef Z_STEREO_CLOS_HH
#define Z_STEREO_CLOS_HH

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "Closure.hh"


namespace Z
{

//--------------------------------------------------------------------//
//-------------------------- TmpClosure ------------------------------//
//--------------------------------------------------------------------//
/**
 * @brief Class TmpClosure
 */
class TmpClosure
{
public:
  Surf2D surf;													///< tmp surfaces

  TmpClosure() {}
  TmpClosure(Closure *closure);
	void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
//   bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0);
  bool IsValid() {return surf.is_valid;}
};


//--------------------------------------------------------------------//
//-------------------------- Closure3D -------------------------------//
//--------------------------------------------------------------------//
/**
 * @brief Class Closure3D
 */
class Closure3D
{
public:
  Surf3D surf;													///< 3D surface
};


//--------------------------------------------------------------------//
//-------------------------- StereoClosures --------------------------//
//--------------------------------------------------------------------//
/**
 * @brief Class StereoClosures: Try to match closures from the stereo images.
 */
class StereoClosures : public StereoBase
{
private:

	Array<TmpClosure> closures[2];				///< tmp closure (left/right)
	Array<Closure3D> closure3ds;					///< 3D closures
	int closMatches;											///< Number of stereo matched closures

#ifdef HAVE_CAST
	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif
	void RecalculateCoordsystem(Closure3D &closure, Pose3 &pose);

	unsigned FindMatchingClosure(TmpClosure &left_clos, Array<TmpClosure> &right_clos, unsigned l);
	void MatchClosures(Array<TmpClosure> &left_clos, Array<TmpClosure> &right_clos, int &matches);
	void Calculate3DClosures(Array<TmpClosure> &left_clos, Array<TmpClosure> &right_clos, int &matches, Array<Closure3D> &closure3ds);
	void DrawSingleMatched(int side, int id, int detail);

public:
	StereoClosures(VisionCore *vc[2], StereoCamera *sc);
	~StereoClosures() {}

	int NumClosures2D(int side);
	int NumClosuresLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::CLOSURE);}		///< 
	int NumClosuresRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::CLOSURE);}	///< 

	const TmpClosure &Closures2D(int side, int i);
	const Closure3D &Closures(int i) {return closure3ds[i];}												///< 

	int NumStereoMatches() {return closMatches;}																		///< 
	void DrawMatched(int side, bool single, int id, int detail);

	void ClearResults();
	void Process();
	void Process(int oX, int oY, int sc);
};

}

#endif

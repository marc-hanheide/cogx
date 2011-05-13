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
#include "StereoCamera.h"
#include "Closure3D.h"
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
  Surf2D surf;                                           ///< tmp surfaces
  unsigned vs3ID;                                        /// id of the vs3 rectangle

  TmpClosure() {}
  TmpClosure(Closure *closure);
	void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
//   bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0);
  bool IsValid() {return surf.is_valid;}
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

  Array<TmpClosure> closures[2];                  ///< tmp closure (left/right)
  int closMatches;                                ///< Number of stereo matched closures

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  unsigned FindMatchingClosure(TmpClosure &left_clos, Array<TmpClosure> &right_clos, unsigned l);
  void MatchClosures(Array<TmpClosure> &left_clos, Array<TmpClosure> &right_clos, int &matches);
  void Calculate3DClosures(Array<TmpClosure> &left_clos, Array<TmpClosure> &right_clos, int &matches);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoClosures(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc);
  ~StereoClosures() {}

  int NumClosures2D(int side);
  int NumClosuresLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::CLOSURE);}		///< 
  int NumClosuresRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::CLOSURE);}	///< 

  const TmpClosure &Closures2D(int side, int i);

  int NumStereoMatches() {return closMatches;}			///< 
  void DrawMatched(int side, bool single, int id, int detail);

  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif

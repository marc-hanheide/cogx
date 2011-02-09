/**
 * @file StereoLJunctions.h
 * @author Andreas Richtsfeld
 * @date Juni 2010
 * @version 0.1
 * @brief Stereo calculation of l-junctions.
 */

#ifndef Z_STEREO_LJCTS_HH
#define Z_STEREO_LJCTS_HH

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "LJunction.hh"
#include "Vector.hh"
#include "LJunction3D.h"

namespace Z
{

/**
 * @brief Class TmpLJunction
 */
class TmpLJunction
{
public:
  Vertex2D point2D;              ///< 2D intersection point
  Vector2 dir[2];               ///< Angles of the two arms

  TmpLJunction() {}
  TmpLJunction(LJunction *ljct);
  void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  bool IsValid() {return true;}									// TODO is always valid
};


/**
 * @brief Class StereoRectangles: Try to match rectangles from the stereo images.
 */
class StereoLJunctions : public StereoBase
{
private:

  Array<TmpLJunction> ljcts[2];   ///< Tmp. l-junctions from the vision cores.
//  Array<LJunction3D> ljct3ds;     ///< 3D l-junctions
  int ljctMatches;                ///< Number of stereo matched l-junctions

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  unsigned FindMatchingLJunction(TmpLJunction &left_ljct, Array<TmpLJunction> &right_ljcts, unsigned l);
  void MatchLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, int &matches);
  void Calculate3DLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, int &matches/*, Array<LJunction3D> &ljct3ds*/);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoLJunctions(StereoCore *sco, VisionCore *vc[2], StereoCamera *sc);
  ~StereoLJunctions() {}

  int NumLJunctions2D(int side) {return vcore[side]->NumGestalts(Gestalt::L_JUNCTION);}         ///< Return number of 2D junctions
  int NumLJunctionsLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::L_JUNCTION);}             ///< Return 2D junction from left image
  int NumLJunctionsRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::L_JUNCTION);}           ///< Return 2D junction from right image

  int NumStereoMatches() {return ljctMatches;}                                                  ///< Return number of l-junction 3D matches
  void DrawMatched(int side, bool single, int id, int detail);
  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif

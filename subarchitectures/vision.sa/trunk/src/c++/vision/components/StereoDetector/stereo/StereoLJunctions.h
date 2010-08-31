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

namespace Z
{

/**
 * @brief Class TmpLJunction
 */
class TmpLJunction
{
public:
	Vertex2D point2D;						///< 2D intersection point

  TmpLJunction() {}
  TmpLJunction(LJunction *ljct);
	void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
	bool IsValid() {return true;}															// TODO is always valid
};


/**
 * @brief Class Rectangle3D
 */
class LJunction3D
{
public:
	Vertex3D point3D;						///< 3D intersection point
	
	Vector3 dir[2];						///< 3D direction of the 2 arms of the L-Junction								/// TODO Calculate
};


/**
 * @brief Class StereoRectangles: Try to match rectangles from the stereo images.
 */
class StereoLJunctions : public StereoBase
{
private:

	Array<TmpLJunction> ljcts[2];							///< Tmp. l-junctions from the vision cores.
	Array<LJunction3D> ljct3ds;								///< 3D l-junctions
	int ljctMatches;													///< Number of stereo matched l-junctions

#ifdef HAVE_CAST
	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif
	void RecalculateCoordsystem(LJunction3D &ljct, Pose3 &pose);

	unsigned FindMatchingLJunction(TmpLJunction &left_ljct, Array<TmpLJunction> &right_ljcts, unsigned l);
	void MatchLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, int &matches);
	void Calculate3DLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, int &matches, Array<LJunction3D> &ljct3ds);
	void DrawSingleMatched(int side, int id, int detail);

public:
	StereoLJunctions(VisionCore *vc[2], StereoCamera *sc);
	~StereoLJunctions() {}

  int NumLJunctions2D(int side) {return vcore[side]->NumGestalts(Gestalt::L_JUNCTION);}
	int NumLJunctionsLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::L_JUNCTION);}		///< 
	int NumLJunctionsRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::L_JUNCTION);}	///< 

//  const TmpLJuntion &LJunctions2D(int side, int i);
//  const LJunction3D &LJunctions(int i) {return ljunction3ds[i];}											///< 

	int NumStereoMatches() {return ljctMatches;}																				///< 
	void DrawMatched(int side, bool single, int id, int detail);
	void ClearResults();
	void Process();
	void Process(int oX, int oY, int sc);
};

}

#endif

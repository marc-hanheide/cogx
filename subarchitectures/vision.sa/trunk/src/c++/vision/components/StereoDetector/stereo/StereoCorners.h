/**
 * @file StereoCorners.h
 * @author Andreas Richtsfeld
 * @date January 2011
 * @version 0.1
 * @brief Stereo calculation of corners. Corners are intersections of 3 lines 
 * (logic: 2 l-junctions with one sharing arm)
 */

#ifndef Z_STEREO_CORNERS_HH
#define Z_STEREO_CORNERS_HH

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "Corner.hh"
#include "Vector.hh"

#include "Corner3D.h"

namespace Z
{

/**
 * @brief Class TmpCorner
 */
class TmpCorner
{
private:
  unsigned id;                          ///< ID of the underlying vs3 Gestalt
  bool isValid;                         ///< True, if corner is valid!

public:
  Vertex2D isct2D;                     ///< 2D intersection point of the corner
  Vector2 armDir[3];                    ///< The three arm directions of the corner in 2D 
  Vertex2D armPoint[3];                 ///< Points on the arm 10px away from intersection

  TmpCorner() {}
  TmpCorner(Corner *corner);
  void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  unsigned ID() {return id;}
  bool IsValid() {return isValid;}
};


/**
 * @brief Class StereoRectangles: Try to match rectangles from the stereo images.
 */
class StereoCorners : public StereoBase
{
private:

  Array<TmpCorner> corners[2];           ///< Tmp. corners from the vision cores.
//  Array<Corner3D> corner3ds;             ///< 3D corners				/// TODO 
  int cornerMatches;                     ///< Number of stereo matched corners

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif
  void RecalculateCoordsystem(Corner3D &ljct, Pose3 &pose);

  unsigned FindMatchingCorner(TmpCorner &left_corner, Array<TmpCorner> &right_corners, unsigned l);
  void MatchCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, int &matches);
  void Calculate3DCornerArms(Corner3D *corner, TmpCorner &left_corner, TmpCorner &right_corner);
  void Calculate3DCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, int &matches);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoCorners(StereoCore *sco, VisionCore *vc[2], StereoCamera *sc);
  ~StereoCorners() {}

  int NumCorners2D(int side) {return vcore[side]->NumGestalts(Gestalt::CORNER);}
  int NumCornersLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::CORNER);}          ///< 
  int NumCornersRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::CORNER);}        ///< 

//  const TmpLJuntion &LJunctions2D(int side, int i);
//  const LJunction3D &LJunctions(int i) {return ljunction3ds[i];}											///< 

  int NumStereoMatches() {return cornerMatches;}																				///< 
  void DrawMatched(int side, bool single, int id, int detail);
  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif

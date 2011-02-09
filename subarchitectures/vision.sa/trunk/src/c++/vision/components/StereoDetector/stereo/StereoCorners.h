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

#include <cstdio>
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
  Vertex2D isct2D;                      ///< 2D intersection point of the corner
  Vector2 armDir[3];                    ///< The three arm directions of the corner in 2D 
  Vertex2D armPoint[3];                 ///< Points on the arm 10px away from intersection point (TODO used for matching!)
  
  unsigned armMatch[3];                 ///< Which arm has the best match with another arm in the other image
  double armMatchValue[3];              ///< The opening angle of the best(second/third) match.
  
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

  Array<TmpCorner> corners[2];          ///< Tmp. corners from the vision cores.
  int cornerMatches;                    ///< Number of stereo matched corners

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  unsigned FindMatchingCorner(TmpCorner &left_corner, Array<TmpCorner> &right_corners, unsigned l);
  void MatchCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, int &matches);
  double CalculateBestArmMatches(TmpCorner &left_corner, TmpCorner &right_corner,
		unsigned &k, unsigned &l, unsigned &m, unsigned &n, unsigned &o, unsigned &p);
  void Calculate3DCornerArms(Corner3D *corner, TmpCorner &left_corner, TmpCorner &right_corner);
  void Calculate3DCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, int &matches);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoCorners(StereoCore *sco, VisionCore *vc[2], StereoCamera *sc);
  ~StereoCorners() {}

  int NumCorners2D(int side) {return vcore[side]->NumGestalts(Gestalt::CORNER);}      ///< Return number of corners in 2D
  int NumCornersLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::CORNER);}          ///< Return corners from left image
  int NumCornersRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::CORNER);}        ///< Return corners from right image

  int NumStereoMatches() {return cornerMatches;}                                      ///< Return number of corner 3D matches
  void DrawMatched(int side, bool single, int id, int detail);
  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif

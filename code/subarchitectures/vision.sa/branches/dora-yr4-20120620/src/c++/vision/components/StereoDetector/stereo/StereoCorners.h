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
#include "vs3/Corner.hh"
#include "math/Vector.hh"

#include "StereoBase.h"
#include "StereoCamera.h"
#include "Corner3D.h"

namespace Z
{

/**
 * @brief Class TmpCorner
 */
class TmpCorner
{
private:
  unsigned vs3ID;                       ///< ID of the underlying vs3 Gestalt
  bool isValid;                         ///< True, if corner is valid!

  RGBColor armColor[3][2];              ///< Color of arms: armColor[nr][LEFT/RIGHT]

public:
  unsigned armMatch[3];                 ///< Which arm has the best match with another arm in the other image			TODO später nur mehr lokal speichern, wenn nicht gebraucht!
  double armMatchValue[3];              ///< The opening angle of the best(second/third) match.
  
  Vertex2D isct2D;                      ///< 2D intersection point of the corner
  Vector2 armDir[3];                    ///< The three arm directions of the corner in 2D 
  Vertex2D armPoint[3];                 ///< Points on the arm 10px away from intersection point (TODO used for matching!)
  
  TmpCorner() {}
  TmpCorner(Corner *corner);
  void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  unsigned GetVs3ID() {return vs3ID;}
  bool IsValid() {return isValid;}
  RGBColor GetArmColor(unsigned arm, unsigned side) {return armColor[arm][side];}
};

/**
 * @brief Class TmpCorner3D
 */
class TmpCorner3D
{
private:
  bool valid;
  unsigned vs3ID[2];              ///< The vs3 IDs of the left and right ljct
  unsigned tmpID[2];              ///< The originally ID in the corner[] Array
  
  Vertex3D isct3D;                ///< Intersection point
  double sig;                     ///< Significance value of the 3D L-junction
  
public:
  TmpCorner3D();
  
  bool Reconstruct(cast::StereoCamera *stereo_cam, TmpCorner &left, TmpCorner &right, double significance2D);
  double GetSignificance() {return sig;}
  Vertex3D GetIsct3D() {return isct3D;}
  void SetValidation(bool v) {valid = v;}
  bool IsValid() {return valid;}
  void SetTmpID(unsigned l, unsigned r) {tmpID[LEFT] = l; tmpID[RIGHT] = r;}
  unsigned GetTmpID(unsigned side) {return tmpID[side];}
  unsigned GetVs3ID(unsigned side){return vs3ID[side];}
};

/**
 * @brief Class StereoRectangles: Try to match rectangles from the stereo images.
 */
class StereoCorners : public StereoBase
{
private:

  Array<TmpCorner> corners[2];          ///< Tmp. corners from the vision cores.
  Array<TmpCorner3D> corners3D;         ///< Tmp. 3D corners
  int cornerMatches;                    ///< Number of stereo matched corners

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

//   unsigned FindMatchingCorner(TmpCorner &left_corner, Array<TmpCorner> &right_corners, unsigned l);
  double CalculateBestArmMatches(TmpCorner &left_corner, TmpCorner &right_corner,
		unsigned &k, unsigned &l, unsigned &m, unsigned &n, unsigned &o, unsigned &p);
  double Calculate2DSignificance(double match, TmpCorner left_corner, TmpCorner right_corner);
  void MatchCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, std::map<double, unsigned> *match_map);
  void Calculate3DCornerArms(Corner3D *corner, TmpCorner &left_corner, TmpCorner &right_corner);
  void BackCheck(std::map<double, unsigned> *match_map, unsigned map_size);
  unsigned Calculate3DCorners(Array<TmpCorner> &left_corners, Array<TmpCorner> &right_corners, std::map<double, unsigned> *match_map);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoCorners(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc);
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

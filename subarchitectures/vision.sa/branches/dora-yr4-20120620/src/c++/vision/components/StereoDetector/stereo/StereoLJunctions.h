/**
 * @file StereoLJunctions.h
 * @author Andreas Richtsfeld
 * @date Juni 2010
 * @version 0.1
 * @brief Stereo calculation of l-junctions.
 */

#ifndef Z_STEREO_LJCTS_HH
#define Z_STEREO_LJCTS_HH

#include <map>
#include "vs3/LJunction.hh"
#include "math/Vector.hh"
#include "utils/Color.hh"

#include "StereoBase.h"
#include "StereoCamera.h"
#include "LJunction3D.h"

namespace Z
{

/**
 * @brief Class TmpLJunction
 */
class TmpLJunction
{
public:
  unsigned vs3ID;                ///< ID of the vs3 L-junction
  Vertex2D isct2D;               ///< 2D intersection point
  Vector2 dir[2];                ///< 2D Direction of the two arms
  RGBColor color[2][2];          ///< Color of the two l-jct line arms [LINE][LEFT/RIGHT]

  TmpLJunction() {}
  TmpLJunction(LJunction *ljct);
  void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  bool IsValid() {return true;}									// TODO is always valid
};


/**
 * @brief Class TmpLJunction3D
 */
class TmpLJunction3D
{
private:
  bool valid;
  unsigned vs3ID[2];              ///< The vs3 IDs of the left and right ljct
  unsigned tmpID[2];              ///< The originally ID in the ljcts[] Array
  
  Vertex3D isct3D;                ///< Intersection point
  double sig;                     ///< Significance value of the 3D L-junction
  
public:
  TmpLJunction3D();
  
  bool Reconstruct(cast::StereoCamera *stereo_cam, TmpLJunction &left, TmpLJunction &right, double significance2D);
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
class StereoLJunctions : public StereoBase
{
private:

  Array<TmpLJunction> ljcts[2];   ///< Tmp. l-junctions from the vision cores.
  Array<TmpLJunction3D> ljcts3D;  ///< Calculated tmp. 3D l-junctions
  int ljctMatches;                ///< Number of stereo matched l-junctions

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  double Calculate2DSignificance(double match, TmpLJunction left_ljct, TmpLJunction right_ljct);
  void MatchLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, std::map<double, unsigned> *match_map);
  void BackCheck(std::map<double, unsigned> *match_map, unsigned map_size);
  unsigned Calculate3DLJunctions(Array<TmpLJunction> &left_ljcts, Array<TmpLJunction> &right_ljcts, std::map<double, unsigned> *match_map);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoLJunctions(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc);
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

/**
 * @file StereoFlapsAri.h
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Stereo matching of flaps, based on rectangles.
 */

#ifndef Z_STEREO_FLAPS_ARI_HH
#define Z_STEREO_FLAPS_ARI_HH

#include "VecMath.hh"		// We use here Pose3, Vector3 from VecMath !!! Different to Pose3.h ect.

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "FlapAri.hh"


namespace Z
{

/**
 * @class TmpFlapAri
 * @brief Tmp. ARI Flap class
 */
class TmpFlapAri
{
public:
  TmpSurf surf[2];										///< tmp surfaces

  TmpFlapAri() {}
  TmpFlapAri(FlapAri *flap);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0, unsigned off1, bool swap);
  bool IsValid() {return surf[0].is_valid && surf[1].is_valid;}
};


/**
 * @class Flap3DAri
 * @brief Tmp. ARI Flap class for 3D
 */
class Flap3DAri
{
public:
  Surf3D surf[2];											///< 3D surface

  bool Reconstruct(StereoCamera *stereo_cam, TmpFlapAri &left, TmpFlapAri &right);
};


/**
 * @class StereoFlapsAri Try to match flaps(ari) from the stereo images.
 * @brief Match stereo flaps from FlapsAri.
 */
class StereoFlapsAri : public StereoBase
{
private:

  Array<TmpFlapAri> flaps[2];					///< tmp flaps
  Array<Flap3DAri> flap3ds;						///< 3D flaps
  int flapMatches;										///< stereo matched flaps

	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
	void RecalculateCoordsystem(Flap3DAri &flap, Pose3 &pose);

  double MatchingScore(TmpFlapAri &left_flap, TmpFlapAri &right_flap, unsigned &off_0, unsigned &off_1, bool &cross);
  unsigned FindMatchingFlap(TmpFlapAri &left_flap, Array<TmpFlapAri> &right_flaps, unsigned l);
  void MatchFlaps(Array<TmpFlapAri> &left_flaps, Array<TmpFlapAri> &right_flaps, int &matches);
  void Calculate3DFlaps(Array<TmpFlapAri> &left_flaps, Array<TmpFlapAri> &right_flaps, int &flapMatches, Array<Flap3DAri> &flap3ds);

public:
	StereoFlapsAri(VisionCore *vc[2], StereoCamera *sc);
	~StereoFlapsAri() {}

  int NumFlaps2D(int side);
  const TmpFlapAri &Flaps2D(int side, int i);
  int NumFlaps() {return flap3ds.Size();}
  const Flap3DAri &Flaps(int i) {return flap3ds[i];}

	int NumFlapsLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::FLAP_ARI);}
	int NumFlapsRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::FLAP_ARI);}

	int NumStereoMatches() {return flapMatches;}
	void Draw(int side);
	void DrawMatched(int side);

	void ClearResults();
	void Process();
};

}

#endif
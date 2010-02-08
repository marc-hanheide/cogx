/**
 * @file StereoFlaps.h
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.2
 * @brief Stereo matching of flaps.
 */

#ifndef Z_STEREO_FLAPS_HH
#define Z_STEREO_FLAPS_HH

#include "VecMath.hh"		// We use here Pose3, Vector3 from VecMath !!! Different to Pose3.h ect.

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "Flap.hh"


namespace Z
{

/**
 * @class TmpFlap
 */
class TmpFlap
{
public:
  TmpSurf surf[2];										///< tmp surfaces

  TmpFlap() {}
  TmpFlap(Flap *flap);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0, unsigned off1, bool swap);
  bool IsValid() {return surf[0].is_valid && surf[1].is_valid;}
};


/**
 * @class Flap3D
 */
class Flap3D
{
public:
  Surf3D surf[2];											///< 3D surface

  bool Reconstruct(StereoCamera *stereo_cam, TmpFlap &left, TmpFlap &right);
};


/**
 * @class StereoFlaps Try to match flaps from the stereo images.
 */
class StereoFlaps : public StereoBase
{
private:

  Array<TmpFlap> flaps[2];						///< tmp flaps
  Array<Flap3D> flap3ds;							///< 3D flaps
  int flapMatches;										///< stereo matched flaps

	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
	void RecalculateCoordsystem(Flap3D &flap, Pose3 &pose);

  double MatchingScore(TmpFlap &left_flap, TmpFlap &right_flap, unsigned &off_0, unsigned &off_1, bool &cross);
  unsigned FindMatchingFlap(TmpFlap &left_flap, Array<TmpFlap> &right_flaps, unsigned l);
  void MatchFlaps(Array<TmpFlap> &left_flaps, Array<TmpFlap> &right_flaps, int &matches);
  void Calculate3DFlaps(Array<TmpFlap> &left_flaps, Array<TmpFlap> &right_flaps, int &flapMatches, Array<Flap3D> &flap3ds);

public:
	StereoFlaps(VisionCore *vc[2], StereoCamera *sc);
	~StereoFlaps() {}

  int NumFlaps2D(int side);
  const TmpFlap &Flaps2D(int side, int i);
  int NumFlaps() {return flap3ds.Size();}
  const Flap3D &Flaps(int i) {return flap3ds[i];}

	int NumFlapsLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::FLAP);}
	int NumFlapsRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::FLAP);}

	int NumStereoMatches() {return flapMatches;}
	void Draw(int side);
	void DrawMatched(int side);

	void ClearResults();

	void Process();
};

}

#endif
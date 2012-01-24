/**
 * @file StereoFlapsAri.h
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Stereo matching of flaps, based on rectangles.
 */

#ifndef Z_STEREO_FLAPS_ARI_HH
#define Z_STEREO_FLAPS_ARI_HH

#include "StereoBase.h"
#include "StereoCamera.h"
#include "FlapAri.hh"
#include "Flap3D.h"


namespace Z
{

/**
 * @class TmpFlapAri
 * @brief Tmp. ARI Flap class
 */
class TmpFlapAri
{
public:
  Surf2D surf[2];                                           ///< Tmp. 2D surfaces
  unsigned id2D;

  TmpFlapAri(){surf[0].is_valid = false;}
  TmpFlapAri(FlapAri *flap);
  void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0, unsigned off1, bool swap);
  bool IsValid() {return surf[0].is_valid && surf[1].is_valid;}
};


//--------------------------------------------------------------------//
//-------------------------- StereoFlapsAri --------------------------//
//--------------------------------------------------------------------//
/**
 * @class StereoFlapsAri Try to match flaps(ari) from the stereo images.
 * @brief Match stereo flaps from FlapsAri.
 */
class StereoFlapsAri : public StereoBase
{
private:

  Array<TmpFlapAri> flaps[2];                                   ///< tmp flaps
  int flapMatches;                                              ///< stereo matched flaps

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  double MatchingScore(TmpFlapAri &left_flap, TmpFlapAri &right_flap, unsigned &off_0, unsigned &off_1, bool &cross);
  unsigned FindMatchingFlap(TmpFlapAri &left_flap, Array<TmpFlapAri> &right_flaps, unsigned l);
  void MatchFlaps(Array<TmpFlapAri> &left_flaps, Array<TmpFlapAri> &right_flaps, int &matches);
  void Calculate3DFlaps(Array<TmpFlapAri> &left_flaps, Array<TmpFlapAri> &right_flaps, int &flapMatches);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoFlapsAri(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc);
  ~StereoFlapsAri() {}

  int NumFlaps2D(int side);
  int NumFlapsLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::FLAP_ARI);}
  int NumFlapsRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::FLAP_ARI);}

  const TmpFlapAri &Flaps2D(int side, int i);

  int NumStereoMatches() {return flapMatches;}
  void DrawMatched(int side, bool single, int id, int detail);
  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif

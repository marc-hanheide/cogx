/**
 * @file StereoEllipses.h
 * @author Andreas Richtsfeld
 * @date December 2009
 * @version 0.1
 * @brief Stereo calculation of ellipses.
 */

#ifndef Z_STEREO_ELLIPSES_HH
#define Z_STEREO_ELLIPSES_HH

#include "VecMath.hh"		// We use here Pose3, Vector3 from VecMath !!! Different to Pose3.h ect.

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "Ellipse.hh"


namespace Z
{

/**
 * @brief Class TmpEllipse
 */
class TmpEllipse
{
public:
  TmpSurf surf;													///< tmp surface

  TmpEllipse() {}
  TmpEllipse(Ellipse *ellipse);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
//   bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0);
  bool IsValid() {return surf.is_valid;}
};


/**
 * @brief Class Ellipse3D
 */
class Ellipse3D
{
public:
  Surf3D surf;													///< 3D surface
};


/**
 * @brief Class StereoEllipses: Try to match ellipses from the stereo images.
 */
class StereoEllipses : public StereoBase
{
private:

	Array<TmpEllipse> ellipses[2];				///< tmp ellipse (left/right)
	Array<Ellipse3D> ellipse3ds;					///< 3D ellipses
	int ellMatches;												///< Number of stereo matched ellipses

	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
	void RecalculateCoordsystem(Ellipse3D &ellipse, Pose3 &pose);

	unsigned FindMatchingEllipse(TmpEllipse &left_ell, Array<TmpEllipse> &right_ell, unsigned l);
	void MatchEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, int &matches);
	void Calculate3DEllipses(Array<TmpEllipse> &left_ell, Array<TmpEllipse> &right_ell, int &matches, Array<Ellipse3D> &ellipse3ds);

public:
	StereoEllipses(VisionCore *vc[2], StereoCamera *sc);
	~StereoEllipses() {}

	int NumEllipses2D(int side);
	const TmpEllipse &Closures2D(int side, int i);
	int NumEllipses() {return ellipse3ds.Size();}
	const Ellipse3D &Ellipses(int i) {return ellipse3ds[i];}

	int NumEllipsesLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::ELLIPSE);}
	int NumEllipsesRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::ELLIPSE);}

	int NumStereoMatches() {return ellMatches;}
	void Draw(int side, bool masked = false);
	void DrawMatched(int side);
	void ClearResults();
	void Process();
};

}

#endif
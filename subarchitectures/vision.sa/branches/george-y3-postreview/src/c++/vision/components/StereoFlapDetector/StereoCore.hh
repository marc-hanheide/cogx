/**
 * $Id: StereoCore.hh,v 1.4 2008/11/19 23:51:33 mz Exp mz $
 *
 * @author Michael Zillich
 * @date September 2006
 *
 */

#ifndef Z_STEREO_CORE_HH
#define Z_STEREO_CORE_HH

#include "Vector2.hh"
#include "Vector3.hh"
#include "VisionCore.hh"
#include "Closure.hh"
#include "Flap.hh"
#include "StereoCamera.hh"
#include "Gestalt.hh"

namespace Z
{

// These are some tuning parameters to filter out "bad" surface hypotheses.
// These might need adjusting to a specific use case.

// maximum allowed angle of one vertex normal to the mean of all vertex
// normals, e.g. 15 deg = pi/12 = 0.2618
static const double SC_MAX_NORMAL_DEVIATION = 0.50;

// minimum required cirumference of a surface in [m]
static const double SC_MIN_CIRC = 0.060;

// maximum allowed side length of a surface (wrong matches often
// tend to produce impossibly long, thing surfaces)
const double SC_MAX_LENGTH = 0.5;  // in [m]


class Vertex
{
public:
  Vector3 p;
  Vector3 n;
};

class TmpSurf
{
public:
  unsigned id;
  vector<Vector2> p;   // original (distorted, unrectified) points
  vector<Vector2> pr;  // rectified points
  bool is_valid;

  TmpSurf() {id = UNDEF_ID; is_valid = false;}
  TmpSurf(Closure *clos) {Init(clos);}
  void Init(Closure *clos);
  void ShiftPointsLeft(unsigned offs);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
};

class Surf3D
{
public:
  Array<Vertex> vertices;

private:
  bool NormalsOK();
  bool SizeOK();
  bool SanityOK();
  void RefineVertices();

public:
  bool Reconstruct(TmpSurf &left, TmpSurf &right, StereoCamera *cam);
};

class TmpFlap
{
public:
  TmpSurf surf[2];

  TmpFlap() {}
  TmpFlap(Flap *flap);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0, unsigned off1, bool swap);
  bool IsValid() {return surf[0].is_valid && surf[1].is_valid;}
};

class Flap3D
{
public:
  Surf3D surf[2];

  bool Reconstruct(TmpFlap &left, TmpFlap &right, StereoCamera *cam);
};


class StereoCore
{
private:
  VisionCore *vcore[2];
  StereoCamera *stereo_cam;
  Array<TmpSurf> surfs[2];
  Array<Surf3D> surf3ds;
  Array<TmpFlap> flaps[2];
  Array<Flap3D> flap3ds;
  int nmatches;
	IplImage *img_l, *img_r;

  double MatchingScore(TmpSurf &left_surf, TmpSurf &right_surf, unsigned &match_offs);
//   unsigned FindMatchingSurf(TmpSurf &left_surf, Array<TmpSurf> &right_surfs, unsigned l);
//   void MatchSurfaces(Array<TmpSurf> &left_surfs, Array<TmpSurf> &right_surfs, int &matches);
  void Calculate3DSurfs(Array<TmpSurf> &left_surfs, Array<TmpSurf> &right_surfs, int &nmatches, Array<Surf3D> &surf3ds);
  double MatchingScore(TmpFlap &left_flap, TmpFlap &right_flap, unsigned &off_0, unsigned &off_1, bool &cross);
  unsigned FindMatchingFlap(TmpFlap &left_flap, Array<TmpFlap> &right_flaps, unsigned l);
  void MatchFlaps(Array<TmpFlap> &left_flaps, Array<TmpFlap> &right_flaps, int &matches);
  void Calculate3DFlaps(Array<TmpFlap> &left_flaps, Array<TmpFlap> &right_flaps, int &nmatches, Array<Flap3D> &flap3ds);

public:
  StereoCore(const string &stereocal_file) throw(Except);
  ~StereoCore();
  const StereoCamera* GetCamera() {return stereo_cam;}
  VisionCore* GetMonoCore(int side) {return vcore[side];}
  void ProcessStereoImage(/*const IplImage *left_img, const IplImage *right_img,*/ int runtime_ms);
  void ClearResults();
  int NumSurfaces2D(int side);
  const TmpSurf &Surfaces2D(int side, int i);
  int NumSurfaces() {return surf3ds.Size();}
  const Surf3D &Surfaces(int i) {return surf3ds[i];}
  int NumFlaps2D(int side);
  const TmpFlap &Flaps2D(int side, int i);
  int NumFlaps() {return flap3ds.Size();}
  const Flap3D &Flaps(int i) {return flap3ds[i];}
  int NumMatches() {return nmatches;}

	int NumFlapsLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::FLAP);}
	int NumFlapsRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::FLAP);}
	void DrawFlaps();
	void SetIplImages(IplImage *iIl, IplImage *iIr);
	void SetActiveDrawAreaSide(int side);
	void PrintResults();

};

}

#endif


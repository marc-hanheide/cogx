/**
 * @file StereoCubes.h
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Stereo matching of cubes.
 */

#ifndef Z_STEREO_CUBES_HH
#define Z_STEREO_CUBES_HH

#include "StereoBase.h"
#include "StereoCamera.h"
#include "Cube.hh"


namespace Z
{

/**
 * @brief TmpCube
 */
class TmpCube
{
public:
  Surf2D surf[3];										///< Tmp. 2D surfaces

  TmpCube() {}
  TmpCube(Cube *cube);
	void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0, unsigned off1, unsigned off2, unsigned ass0, unsigned ass1, unsigned ass2);
  bool IsValid() {return surf[0].is_valid && surf[1].is_valid && surf[2].is_valid;}
};


/**
 * @brief Cube3D
 */
class Cube3D
{
public:
  Surf3D surf_vis[3];								///< The visible 3D surfaces (ordered clockwise)			/// TODO TomGine needs the vertices counter clockwise
  Surf3D surf_hid[3];								///< The hidden 3D surfaces (ordered counter-clockwise)

  bool Reconstruct(cast::StereoCamera *stereo_cam, TmpCube &left, TmpCube &right);
};


/**
 * @brief StereoCubes: Try to match cubes from the stereo images.
 */
class StereoCubes : public StereoBase
{
private:

  Array<TmpCube> cubes[3];						///< tmp cubes
  Array<Cube3D> cube3ds;							///< 3D cubes
  int cubeMatches;										///< stereo matched cubes

#ifdef HAVE_CAST
	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif
  void RecalculateCoordsystem(Cube3D &cube, Pose3 &pose);
  double MatchingScore(TmpCube &left_cube, TmpCube &right_cube, 
  unsigned &off_0, unsigned &off_1, unsigned &off_2, 
  unsigned &assign_0, unsigned &assign_1, unsigned &assign_2);
  unsigned FindMatchingCube(TmpCube &left_cube, Array<TmpCube> &right_cubes, unsigned l);
  void MatchCubes(Array<TmpCube> &left_cubes, Array<TmpCube> &right_cubes, int &matches);
  void Calculate3DCubes(Array<TmpCube> &left_cubes, Array<TmpCube> &right_cubes, 
  int &cubeMatches, Array<Cube3D> &cube3ds);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoCubes(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc);
  ~StereoCubes() {}

  int NumCubes2D(int side);   /// Sollte NumCubes2D(int LEFT/RIGHT) nicht anders ersetzt werden "NumGestalts(type, side)", dann überall gleich???
  int NumCubesLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::CUBE);}			///< Return cubes from left stereo image in 2D
  int NumCubesRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::CUBE);}		///< Return cubes from right stereo image in 2D

  const TmpCube &Cubes2D(int side, int i);
  const Cube3D &Cubes(int i) {return cube3ds[i];}		///< Return 3D-Cube from position i.

  int NumStereoMatches() {return cubeMatches;}			///< Number of matched stereo features. TODO eigentlich hier "return cube3ds.Size();"
  void DrawMatched(int side, bool single, int id, int detail);

  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif

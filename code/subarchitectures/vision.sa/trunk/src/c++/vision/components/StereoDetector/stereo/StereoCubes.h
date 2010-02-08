/**
 * @file StereoCubes.h
 * @author Andreas Richtsfeld
 * @date Jannuar 2010
 * @version 0.1
 * @brief Stereo matching of cubes.
 */

#ifndef Z_STEREO_CUBES_HH
#define Z_STEREO_CUBES_HH

#include "VecMath.hh"		// We use here Pose3, Vector3 from VecMath !!! Different to Pose3.h ect.

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "Cube.hh"


namespace Z
{

/**
 * @brief TmpCube
 */
class TmpCube
{
public:
  TmpSurf surf[3];										///< tmp surfaces

  TmpCube() {}
  TmpCube(Cube *cube);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0, unsigned off1, bool swap);
  bool IsValid() {return surf[0].is_valid && surf[1].is_valid;}
};


/**
 * @brief Cube3D
 */
class Cube3D
{
public:
  Surf3D surf[3];											///< 3D surfaces (unordered?)

  bool Reconstruct(StereoCamera *stereo_cam, TmpCube &left, TmpCube &right);
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

	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
	void RecalculateCoordsystem(Cube3D &cube, Pose3 &pose);

  double MatchingScore(TmpCube &left_cube, TmpCube &right_cube, unsigned &off_0, unsigned &off_1, bool &cross);
  unsigned FindMatchingCube(TmpCube &left_cube, Array<TmpCube> &right_cubes, unsigned l);
  void MatchCubes(Array<TmpCube> &left_cubes, Array<TmpCube> &right_cubes, int &matches);
  void Calculate3DCubes(Array<TmpCube> &left_cubes, Array<TmpCube> &right_cubes, int &cubeMatches, Array<Cube3D> &cube3ds);

public:
	StereoCubes(VisionCore *vc[2], StereoCamera *sc);
	~StereoCubes() {}

  int NumCubes2D(int side);
					/// TODO TODO TODO TODO Sollte NumCubes2D(int LEFT/RIGHT) nicht anders ersetzt werden "NumGestalts(type, side)", dann überall gleich???
	int NumCubesLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::CUBE);}			///< Return cubes from left stereo image in 2D
	int NumCubesRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::CUBE);}		///< Return cubes from right stereo image in 2D
  const TmpCube &Cubes2D(int side, int i);

  int NumCubes() {return cube3ds.Size();}						///< TODO Gleich wie NumStereoMatches() ??? Sollte weg und durch NumStereoMatches ersetzt werden.
  const Cube3D &Cubes(int i) {return cube3ds[i];}		///< Return 3D-Cube from position i.

	int NumStereoMatches() {return cubeMatches;}			///< Number of matched stereo features. TODO eigentlich hier "return cube3ds.Size();"

	void Draw(int side);
	void DrawMatched(int side);

	void ClearResults();

	void Process();
};

}

#endif
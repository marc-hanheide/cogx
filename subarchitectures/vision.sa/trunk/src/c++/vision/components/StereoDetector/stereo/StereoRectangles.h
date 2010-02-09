/**
 * @file StereoRectangles.h
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Stereo calculation of rectangles.
 */

#ifndef Z_STEREO_RECTS_HH
#define Z_STEREO_RECTS_HH

#include "VecMath.hh"		// We use here Pose3, Vector3 from VecMath !!! Different to Pose3.h ect.

#include "StereoBase.h"
#include "StereoCamera.hh"
#include "Rectangle.hh"


namespace Z
{

/**
 * @brief Class TmpRectangle
 */
class TmpRectangle
{
public:
  TmpSurf surf;												///< tmp surfaces

  TmpRectangle() {}
  TmpRectangle(Rectangle *rectangle);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
//   bool IsAtPosition(int x, int y) const;
  void Fuddle(unsigned off0);
  bool IsValid() {return surf.is_valid;}
};


/**
 * @brief Class Rectangle3D
 */
class Rectangle3D
{
public:
  Surf3D surf;												///< 3D surface

//   bool Reconstruct(StereoCamera *stereo_cam, TmpFlap &left, TmpFlap &right);
};


/**
 * @brief Class StereoRectangles: Try to match rectangles from the stereo images.
 */
class StereoRectangles : public StereoBase
{
private:

  Array<TmpRectangle> rectangles[2];				///< tmp rectangles
	Array<Rectangle3D> rectangle3ds;					///< 3D rectangles
	int rectMatches;													///< Number of stereo matched rectangles

	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
	void RecalculateCoordsystem(Rectangle3D &rectangle, Pose3 &pose);

	unsigned FindMatchingRectangle(TmpRectangle &left_rect, Array<TmpRectangle> &right_rects, unsigned l);
	void MatchRectangles(Array<TmpRectangle> &left_rects, Array<TmpRectangle> &right_rects, int &matches);
	void Calculate3DRectangles(Array<TmpRectangle> &left_rects, Array<TmpRectangle> &right_rects, int &matches, Array<Rectangle3D> &rectangle3ds);

public:
	StereoRectangles(VisionCore *vc[2], StereoCamera *sc);
	~StereoRectangles() {}

  int NumRectangles2D(int side);
  const TmpRectangle &Rectangles2D(int side, int i);
  int NumRectangles() {return rectangle3ds.Size();}
  const Rectangle3D &Rectangles(int i) {return rectangle3ds[i];}

	int NumRectanglesLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::RECTANGLE);}
	int NumRectanglesRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::RECTANGLE);}

	int NumStereoMatches() {return rectMatches;}
	void Draw(int side, bool masked = false);
	void DrawMatched(int side);
	void ClearResults();
	void Process();
};

}

#endif
/**
 * @file StereoRectangles.h
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Stereo calculation of rectangles.
 */

#ifndef Z_STEREO_RECTS_HH
#define Z_STEREO_RECTS_HH

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
  Surf2D surf;							///< Tmp. 2D surface

  TmpRectangle() {}
  TmpRectangle(Rectangle *rectangle);
	void RePrune(int oX, int oY, int sc);
  void Rectify(StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;													/// TODO weg damit? oder braucht man noch?
  void Fuddle(unsigned off0);
  bool IsValid() {return surf.is_valid;}													/// do not show if it is not valid!
};


/**
 * @brief Class Rectangle3D
 */
class Rectangle3D
{
public:
  Surf3D surf;							///< 3D surface

//   bool Reconstruct(StereoCamera *stereo_cam, TmpFlap &left, TmpFlap &right);
};


/**
 * @brief Class StereoRectangles: Try to match rectangles from the stereo images.
 */
class StereoRectangles : public StereoBase
{
private:

  Array<TmpRectangle> rectangles[2];				///< Tmp. rectangles from the vision cores.
	Array<Rectangle3D> rectangle3ds;					///< 3D rectangles
	int rectMatches;													///< Number of stereo matched rectangles

#ifdef HAVE_CAST
	bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif
	void RecalculateCoordsystem(Rectangle3D &rectangle, Pose3 &pose);

	unsigned FindMatchingRectangle(TmpRectangle &left_rect, Array<TmpRectangle> &right_rects, unsigned l);
	void MatchRectangles(Array<TmpRectangle> &left_rects, Array<TmpRectangle> &right_rects, int &matches);
	void Calculate3DRectangles(Array<TmpRectangle> &left_rects, Array<TmpRectangle> &right_rects, int &matches, Array<Rectangle3D> &rectangle3ds);
	void DrawSingleMatched(int side, int id, int detail);

public:
	StereoRectangles(VisionCore *vc[2], StereoCamera *sc);
	~StereoRectangles() {}

  int NumRectangles2D(int side) {return vcore[side]->NumGestalts(Gestalt::RECTANGLE);};
//	int NumRectanglesLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::RECTANGLE);}		///< TODO weg
//	int NumRectanglesRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::RECTANGLE);}	///< TODO weg

  const TmpRectangle &Rectangles2D(int side, int i);
  const Rectangle3D &Rectangles(int i) {return rectangle3ds[i];}											///< 

	int NumStereoMatches() {return rectMatches;}																				///< 
	void DrawMatched(int side, bool single, int id, int detail);
	
	void ClearResults();
	void Process();
	void Process(int oX, int oY, int sc);
};

}

#endif

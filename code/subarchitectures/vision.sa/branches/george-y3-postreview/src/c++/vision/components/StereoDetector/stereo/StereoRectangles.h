/**
 * @file StereoRectangles.h
 * @author Andreas Richtsfeld
 * @date November 2009
 * @version 0.1
 * @brief Stereo calculation of rectangles.
 */

#ifndef Z_STEREO_RECTS_HH
#define Z_STEREO_RECTS_HH

#include "StereoCore.h"
#include "StereoBase.h"
#include "StereoCamera.h"
#include "Rectangle3D.h"
#include "Rectangle.hh"

namespace Z
{

/**
 * @brief Class TmpRectangle
 */
class TmpRectangle
{
public:
  Surf2D surf;                                           /// surface of the rectangle
  unsigned vs3ID;                                        /// id of the vs3 rectangle

  TmpRectangle() {}
  TmpRectangle(Rectangle *rectangle);
  void RePrune(int oX, int oY, int sc);
  void Rectify(cast::StereoCamera *stereo_cam, int side);
  void Refine();
  bool IsAtPosition(int x, int y) const;                 /// TODO weg damit? oder braucht man noch?
  void Fuddle(unsigned off0);
  bool IsValid() {return surf.is_valid;}                 /// do not show if it is not valid!
};


/**
 * @brief Class StereoRectangles: Try to match rectangles from the stereo images.
 */
class StereoRectangles : public StereoBase
{
private:

  Array<TmpRectangle> rectangles[2];                    ///< Left/Right tmp. rectangles from the vision cores.
  int rectMatches;					///< Number of stereo matched rectangles

#ifdef HAVE_CAST
  bool StereoGestalt2VisualObject(VisionData::VisualObjectPtr &obj, int id);
#endif

  unsigned FindMatchingRectangle(TmpRectangle &left_rect, Array<TmpRectangle> &right_rects, unsigned l);
  void MatchRectangles(Array<TmpRectangle> &left_rects, Array<TmpRectangle> &right_rects, int &matches);
  void Calculate3DRectangles(Array<TmpRectangle> &left_rects, Array<TmpRectangle> &right_rects, int &matches);
  void DrawSingleMatched(int side, int id, int detail);

public:
  StereoRectangles(StereoCore *sco, VisionCore *vc[2], cast::StereoCamera *sc);
  ~StereoRectangles() {}

  int NumRectangles2D(int side) {return vcore[side]->NumGestalts(Gestalt::RECTANGLE);};

  const TmpRectangle &Rectangles2D(int side, int i);			///< TODO Public? Überhaupt notwendig?

  int NumStereoMatches() {return rectMatches;}                          ///< 
  void DrawMatched(int side, bool single, int id, int detail);
	
  void ClearResults();
  void Process();
  void Process(int oX, int oY, int sc);
};

}

#endif

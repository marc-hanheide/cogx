/**
 * @file StereoCore.hh
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.2
 * @brief Managment of stereo object detection with two vs3 vision cores.
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
#include "Draw.hh"

#include "StereoBase.h"
#include "StereoEllipses.h"
#include "StereoClosures.h"
#include "StereoRectangles.h"
#include "StereoFlaps.h"
#include <VisionData.hpp>

namespace Z
{

class StereoCore
{
private:
  VisionCore *vcore[2];																	///< left and right vision core
  StereoCamera *stereo_cam;															///< stereo camera parameters and functions
	IplImage *img_l, *img_r;															///< current left and right image

  StereoBase* stereoGestalts[StereoBase::MAX_TYPE];			///< Stereo gestalt type list.

public:
  StereoCore(const string &stereocal_file) throw(Except);
  ~StereoCore();

	void InitStereoGestalts();
  const StereoCamera* GetCamera() {return stereo_cam;}
  VisionCore* GetMonoCore(int side) {return vcore[side];}

  void ClearResults();
	void SetActiveDrawAreaSide(int side);
	
	void ProcessStereoImage(int runtime_ms, IplImage *iIl, IplImage *iIr);

	/// TODO Zugriff auf Vision Core sollte besser und nicht direkt erfolgen!!!
 	int NumRectanglesLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::RECTANGLE);}
 	int NumRectanglesRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::RECTANGLE);}
	int NumFlapsLeft2D() {return vcore[LEFT]->NumGestalts(Gestalt::FLAP);}
	int NumFlapsRight2D() {return vcore[RIGHT]->NumGestalts(Gestalt::FLAP);}

	int NumStereoMatches(StereoBase::Type type) {return stereoGestalts[type]->NumStereoMatches();}
	void GetVisualObject(StereoBase::Type type, int id, VisionData::VisualObjectPtr &obj);
	void DrawStereoResults(StereoBase::Type type);

	void PrintResults();
	void PrintRectResults();
};

}

#endif


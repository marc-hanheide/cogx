/**
 * @file StereoCore.hh
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.2
 * @brief Managment of stereo object detection with two vs3 vision cores.
 */

#ifndef Z_STEREO_CORE_HH
#define Z_STEREO_CORE_HH

#include <VisionData.hpp>

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
#include "StereoFlapsAri.h"
#include "StereoCubes.h"


namespace Z
{

/**
 * @brief Class StereoCore: Management of calculations in stereo-core and both vision-cores.
 */
class StereoCore
{
private:
  VisionCore *vcore[2];																					///< left and right vision core
  StereoCamera *stereo_cam;																			///< stereo camera parameters and functions
	IplImage *img_l, *img_r;																			///< current left and right image
  StereoBase* stereoGestalts[StereoBase::MAX_TYPE];							///< Stereo gestalt type list.

	void SetActiveDrawAreaSide(int side);
	void SetImages(IplImage *iIl, IplImage *iIr);
	void InitStereoGestalts();

public:
  StereoCore(const string &stereocal_file) throw(Except);
  ~StereoCore();

  const StereoCamera* GetCamera() {return stereo_cam;}					///< Return stereo camera parameters
  VisionCore* GetMonoCore(int side) {return vcore[side];}				///< Return single vision core [LEFT/RIGHT]

  void ClearResults();
	void ProcessStereoImage(int runtime_ms, IplImage *iIl, IplImage *iIr);
	void GetVisualObject(StereoBase::Type type, int id, VisionData::VisualObjectPtr &obj);
	void DrawStereoResults(StereoBase::Type type, IplImage *iIl, IplImage *iIr, bool detected, bool matched);
	int NumStereoMatches(StereoBase::Type type) {return stereoGestalts[type]->NumStereoMatches();}

	/// TODO delete later: only for debuging first results
	void PrintResults();
	void PrintRectResults();
};

}

#endif


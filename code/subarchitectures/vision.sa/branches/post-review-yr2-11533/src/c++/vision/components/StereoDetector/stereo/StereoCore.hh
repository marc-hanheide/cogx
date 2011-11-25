/**
 * @file StereoCore.hh
 * @author Andreas Richtsfeld
 * @date October 2009
 * @version 0.2
 * @brief Managment of stereo object detection with two vs3 vision cores.
 */

#ifndef Z_STEREO_CORE_HH
#define Z_STEREO_CORE_HH

#include <stdexcept>

#include "Vector.hh"
#include "VisionCore.hh"
#include "StereoCamera.hh"
#include "Gestalt.hh"
#include "Draw.hh"
#include "Math.hh"

#include "StereoBase.h"
#include "StereoLJunctions.h"
#include "StereoEllipses.h"
#include "StereoClosures.h"
#include "StereoRectangles.h"
#include "StereoFlaps.h"
#include "StereoFlapsAri.h"
#include "StereoCubes.h"

#include "Closure.hh"
#include "Flap.hh"
#include "Corner.hh"

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

	struct PruningParameter																				///< Parameters, when pruned image will be processed at stereo core
	{
		bool pruning;						///< Pruned image delivered
		int offsetX;						///< Offset x-coordinate
		int offsetY;						///< Offset y-coordinate
		int scale;							///< Scale between original and pruned image
	};
	PruningParameter *pPara;																			///< Pruning parameters of an image.

	void SetActiveDrawAreaSide(int side);
	void SetImages(IplImage *iIl, IplImage *iIr);
	void InitStereoGestalts();
	const StereoCamera* GetCamera() {return stereo_cam;}					///< Return stereo camera parameters

public:
  StereoCore(const string &stereocal_file) throw(std::runtime_error);
  ~StereoCore();

  VisionCore* GetMonoCore(int side) {return vcore[side];}										///< Return single vision core [LEFT/RIGHT]
	StereoBase* GetStereoGestalts(int type) {return stereoGestalts[type];}		///< Return the stereo results

  void ClearResults();
	void ProcessStereoImage(int runtime_ms, float ca, float co, IplImage *iIl, IplImage *iIr);
	void ProcessStereoImage(int runtime_ms, float ca, float co, IplImage *iIl, IplImage *iIr, int oX, int oY, int sc);
#ifdef HAVE_CAST
	bool GetVisualObject(StereoBase::Type type, int id, VisionData::VisualObjectPtr &obj);
#endif
	const char* GetGestaltListInfo();

	int NumMonoGestalts(Gestalt::Type type, int side) {return vcore[side]->Gestalts(type).Size();}
	bool DrawMonoResults(Gestalt::Type type, IplImage *iIl, IplImage *iIr, bool masked, bool single,
											 int singleSide = 0, int id = 0, int detail = 0);
	int NumStereoMatches(StereoBase::Type type) {return stereoGestalts[type]->NumStereoMatches();}
	void DrawStereoResults(StereoBase::Type type, IplImage *iIl, IplImage *iIr, bool showAllStereoMatched, 
												 bool single, int id = 0, int detail = 0);
	void DrawROI(int side, CvRect roi, int roiScale, IplImage *iIl, IplImage *iIr);
	void DrawPrunedROI(int side, int offsetX, int offsetY, IplImage *iIl, IplImage *iIr);

	void PrintVCoreStatistics();
	unsigned PickGestaltAt(int side, Gestalt::Type type, int x, int y, unsigned start_after, bool reject_masked);
	
	/// TODO delete later: only for debuging first results
	void PrintResults();
	void PrintJunctions2File();
	void PrintCorners2File();
};

}

#endif

/**
 * @file Reasoner.h
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Reasoning about detected planes and detected visual objects.
 */

#ifndef SD_REASONER_H
#define SD_REASONER_H

#include <stdio.h>
#include <vector>
#include <VisionData.hpp>

#include "Vector.hh"
#include "StereoCore.h"
#include "Plane.h"
#include "Object.h"

#include "StereoBase.h"
#include "StereoEllipses.h"

namespace Z
{

/**
 * @class Reasoner
 * @brief Try to explain the scene, based on detected stereo data.
 */
class Reasoner
{
private:
	bool havePlane;			///< True, if we have a dominant plane
	Plane *plane;			///< The dominant plane
	
	StereoCore *score;		///< Actual stereo core
	int maxSG;			///< Maximum value for the stereoGestalt array
	int sgCounter;			///< Counter for the stereoGestalt array
	StereoBase* stereoGestalts[StereoBase::MAX_TYPE][3];
	bool gAF;							///< True, if gestalt array is filled.
	
	Array<VisionData::VisualObjectPtr> filteredObjs;		///< actual filtered visual objects
	Array<Object*> objects;						///< stored objects
	
	void FilterGestalt(StereoBase::Type type);
	bool HackFilter(VisionData::VisualObjectPtr &obj);
	void CreateObjects();
	
public:
	Reasoner();
	void ProcessConvexHull(Vector3 pos, double rad, std::vector<Vector3> p);
	bool GetPlane(VisionData::VisualObjectPtr &obj);
	bool Process(StereoCore *sc);
	void GetResults(Array<VisionData::VisualObjectPtr> &objs, bool unprojected);
};

}

#endif

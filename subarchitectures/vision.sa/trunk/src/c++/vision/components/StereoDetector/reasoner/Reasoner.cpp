/**
 * @file Reasoner.cpp
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Reasoning about detected planes and detected visual objects.
 */

#include "Reasoner.h"


namespace Z
{

/**
 * @brief Constructor of reasoner class.
 */
Reasoner::Reasoner()
{
	printf("Reasoner::Reasoner: New reasoner created!\n");
	
	havePlane = false; 		// have a detected dominant plane
}


/**
 * @brief Process a new convex hull and decide, if it is a dominant plane for the scene.
 * @param pos Postion of center from convex hull.
 * @param rad Radius of convex hull
 * @param p Convex hull points
 */
bool Reasoner::ProcessConvexHull(Vector3 pos, double rad, std::vector<Vector3> p)
{
	plane = new Z::Plane(pos, rad, p);
// 	if(plane->GetVisualObject(obj))
// 	{
// // antiquated
// // 		char obj_label[32];
// // 		sprintf(obj_label, "Plane");
// // 		obj->label = obj_label;
// 
// 		// add visual object to working memory
// 		planeID = newDataID();
// 		addToWorkingMemory(planeID, obj);
// 
// 		log("Wrote new plane as visual object to working memory: %s", planeID.c_str());

		havePlane = true;
		return true;
// 	}
// 	else return false;
}

/**
 * @brief Get the (dominant) plane as visual object.
 * @param obj Dominant plane (convex hull mesh) as visual object.
 */
bool Reasoner::GetPlane(VisionData::VisualObjectPtr &obj)
{
	if(havePlane)
	{
		plane->GetVisualObject(obj);
		return true;
	}
	else return false;
}

/**
 * @brief Process data of new frame
 * @param sc Stereo core
 */
bool Reasoner::Process(StereoCore *sc)
{
	printf("Reasoner::Process: not yet implemented!\n");
	
	
	return true;
}


/**
 * @brief Get the results from the reasoner
 */
void Reasoner::GetResults()
{
	printf("Reasoner::GetResults: not yet implemented!\n");

}




}





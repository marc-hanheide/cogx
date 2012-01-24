/**
 * @file StereoDetectorReasoner.cpp
 * @author Andreas Richtsfeld
 * @date August 2010
 * @version 0.1
 * @brief Reasoning about detected planes and detected visual objects.
 */


#include <cast/architecture/ChangeFilterFactory.hpp>
#include "StereoDetectorReasoner.h"

/**
 * @brief The function called to create a new instance of our component.
 */
extern "C" {
	cast::CASTComponentPtr newComponent() {
    return new cast::StereoDetectorReasoner();
  }
}

namespace cast
{

/**
 *	@brief Destructor of class StereoDetectorReasoner
 */
StereoDetectorReasoner::~StereoDetectorReasoner()
{}


/**
 *	@brief Called by the framework to configure the component.
 *	@param _config Config
 */
void StereoDetectorReasoner::configure(const std::map<std::string,std::string> & _config)
{
	havePlane = false;
}

/**
 *	@brief Called by the framework after configuration, before run loop.
 */
void StereoDetectorReasoner::start()
{
	// add change filter for ProtoObject changes
	addChangeFilter(createLocalTypeFilter<ConvexHull>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetectorReasoner>(this, &StereoDetectorReasoner::receiveConvexHull));
	addChangeFilter(createLocalTypeFilter<ConvexHull>(cdl::DELETE),
      new MemberFunctionChangeReceiver<StereoDetectorReasoner>(this, &StereoDetectorReasoner::deleteConvexHull));
			
	// add change filter for ProtoObject changes
	addChangeFilter(createLocalTypeFilter<ReasonerObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetectorReasoner>(this, &StereoDetectorReasoner::receiveReasonerObject));

	// add change filter for SDReasonerCommand changes
	addChangeFilter(createLocalTypeFilter<SDReasonerCommand>(cdl::ADD),
      new MemberFunctionChangeReceiver<StereoDetectorReasoner>(this, &StereoDetectorReasoner::receiveSDReasonerCommand));
}


/**
 *	@brief Called by the framework to start component run loop.
 */
void StereoDetectorReasoner::runComponent()
{
}

/**
 *	@brief Receive a new created convex hull.
 *	@param wmc Working memory change.
 */
void StereoDetectorReasoner::receiveConvexHull(const cdl::WorkingMemoryChange & _wmc)
{
	// delete old plane, if we got new one
	if(havePlane)
	{
		printf("StereoDetectorReasoner::receiveConvexHull: TODO: we have a plane => update!\n");
	}

	// convert plane to visual object and write to working memory	
	ConvexHullPtr chPtr = getMemoryEntry<VisionData::ConvexHull>(_wmc.address);
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;

	// Create a visual object for the plane as mesh and recalculate point sequence 
	// of convex hull in respect to the center.
	obj->pose = chPtr->center;												// pose of plane is the center point
	Vector3 p;
	std::vector<Vector3> points;
	for(unsigned i=0; i< chPtr->PointsSeq.size(); i++)
	{
		p.x = chPtr->PointsSeq[i].x - obj->pose.pos.x;	// shift the plane in respect to the pose
		p.y = chPtr->PointsSeq[i].y - obj->pose.pos.y;
		p.z = chPtr->PointsSeq[i].z - obj->pose.pos.z;
		points.push_back(p);
	}
	
	// center position of the plane
	Vector3 pos;
	pos.x = obj->pose.pos.x;
	pos.y = obj->pose.pos.y;
	pos.z = obj->pose.pos.z;
	double radius = chPtr->radius;
	
	plane = new Z::Plane(pos, radius, points);
	if(plane->GetVisualObject(obj))
	{
// antiquated
// 		char obj_label[32];
// 		sprintf(obj_label, "Plane");
// 		obj->label = obj_label;

		// add visual object to working memory
		planeID = newDataID();
		addToWorkingMemory(planeID, obj);

		log("Wrote new plane as visual object to working memory: %s", planeID.c_str());
	}
	havePlane = true;
}


/**
 *	@brief Delete a convex hull.
 *	@param wmc Working memory change.
 */
void StereoDetectorReasoner::deleteConvexHull(const cdl::WorkingMemoryChange & _wmc)
{
	printf("StereoDetectorReasoner::deleteConvexHull: not yet implemented!\n");
}


/**
 *	@brief Receive a object from the stereo detector.
 *	@param wmc Working memory change.
 */
void StereoDetectorReasoner::receiveReasonerObject(const cdl::WorkingMemoryChange & _wmc)
{
	log("StereoDetectorReasoner::receiveReasonerObject");
	
	ReasonerObjectPtr roPtr = getMemoryEntry<VisionData::ReasonerObject>(_wmc.address);
	VisionData::VisualObjectPtr obj = roPtr->obj;
	
	Vector3 pos;
	pos.x = obj->pose.pos.x;
	pos.y = obj->pose.pos.y;
	pos.z = obj->pose.pos.z;
	
	Z::Object *o = new Z::Object(pos, obj->model->vertices, obj->model->faces);
	o->ProjectToPlane(plane);
// 	SDRObjects objects;
// 	objects.obj.push_back(o);
	
	if(o->GetVisualObjectProjected(obj))		// GetVisualObject prjected to ground plane
	{
		std::string objectID = newDataID();
		objectIDs.push_back(objectID);
		addToWorkingMemory(objectID, obj);

		log("Wrote new plane as projected visual object to working memory: %s", objectID.c_str());
	}
	else																		// GetVisualObject
	{
		o->GetVisualObject(obj);
		std::string objectID = newDataID();
		objectIDs.push_back(objectID);
		addToWorkingMemory(objectID, obj);

		log("Wrote new plane as visual object to working memory: %s", objectID.c_str());
	}
}



/**
 *	@brief Receive a object from the stereo detector.
 *	@param wmc Working memory change.
 */
void StereoDetectorReasoner::receiveSDReasonerCommand(const cdl::WorkingMemoryChange & _wmc)
{
	log("StereoDetectorReasoner::receiveSDReasonerCommand: not yet implemented!");
}

}









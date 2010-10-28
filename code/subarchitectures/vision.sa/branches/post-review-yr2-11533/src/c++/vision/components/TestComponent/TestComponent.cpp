/**
 * @file TestComponent.cpp
 * @author Andreas Richtsfeld
 * @date March 2010
 * @version 0.1
 * @brief Testing WM problems
 */


#include <cast/architecture/ChangeFilterFactory.hpp>
#include "TestComponent.h"

using namespace std;
using namespace VisionData;

/**
 * @brief The function called to create a new instance of our component.
 */
extern "C" {
	cast::CASTComponentPtr newComponent() {
    return new cast::TestComponent();
  }
}

namespace cast
{

/**
 *	@brief Destructor of class StereoDetector
 */
TestComponent::~TestComponent()
{}


/**
 *	@brief Called by the framework to configure the component.
 *	@param _config Config
 */
void TestComponent::configure(const map<string,string> & _config)
{}

/**
 *	@brief Called by the framework after configuration, before run loop.
 */
void TestComponent::start()
{}


/**
 *	@brief Called by the framework to start component run loop.
 */
void TestComponent::runComponent()
{
	while(1)
	{
		log("Starting test routine:");
		cvWaitKey(1000);

		// write visual objects
		log("write visual objects to wm");
		for(unsigned i=0; i<2; i++)
			WriteNewVisualObject();

		cvWaitKey(1000);

		log("delete all visual objects");
		DeleteVisualObjectsFromWM();

		cvWaitKey(1000);

		// write visual objects
		log("\nwrite and delete all visual objects");
		for(unsigned i=0; i<2; i++)
			WriteNewVisualObject();
		DeleteVisualObjectsFromWM();
	}
}


/**
 * @brief Write visual objects to the working memory.
 */
void TestComponent::WriteNewVisualObject()
{
	VisionData::VisualObjectPtr obj = new VisionData::VisualObject;
	obj->model = new VisionData::GeometryModel;

	for(int i=0; i<5; i++)
	{
		CreateVisualObject(obj, i);		// create new dummy visual object

		// store object id and add visual object to working memory
		std::string objectID = newDataID();
		objectIDs.push_back(objectID);
		addToWorkingMemory(objectID, obj);
		log("Add new visual object to working memory: %s - %s", obj->identLabels[0].c_str(), objectID.c_str());

//		cvWaitKey(250);	/// TODO HACK TODO HACK TODO HACK TODO HACK
	}
}


/**
 * @brief Create a filled dummy visual object.
 */
bool TestComponent::CreateVisualObject(VisionData::VisualObjectPtr &obj, int id)
{
	// add center point to the model
	cogx::Math::Pose3 cogxPose;
	cogxPose.pos.x = 100; // pose.pos.x;
	cogxPose.pos.y = 100; // pose.pos.y;
	cogxPose.pos.z = 0; // pose.pos.z;
	obj->pose = cogxPose;

	// create vertices (relative to the 3D center point)
	VisionData::Vertex v0;
	v0.pos.x = 0; // rectangle.surf.vertices[i].p.x;
	v0.pos.y = 0; // rectangle.surf.vertices[i].p.y;
	v0.pos.z = 0; // rectangle.surf.vertices[i].p.z;
	obj->model->vertices.push_back(v0);

	VisionData::Vertex v1;
	v1.pos.x = 50; // rectangle.surf.vertices[i].p.x;
	v1.pos.y = 0; // rectangle.surf.vertices[i].p.y;
	v1.pos.z = 0; // rectangle.surf.vertices[i].p.z;
	obj->model->vertices.push_back(v1);

	VisionData::Vertex v2;
	v2.pos.x = 50; // rectangle.surf.vertices[i].p.x;
	v2.pos.y = 50; // rectangle.surf.vertices[i].p.y;
	v2.pos.z = 0; // rectangle.surf.vertices[i].p.z;
	obj->model->vertices.push_back(v2);

	VisionData::Vertex v3;
	v3.pos.x = 0; // rectangle.surf.vertices[i].p.x;
	v3.pos.y = 50; // rectangle.surf.vertices[i].p.y;
	v3.pos.z = 0; // rectangle.surf.vertices[i].p.z;
	obj->model->vertices.push_back(v3);

	// add faces to the vision model
	VisionData::Face f;
	f.vertices.push_back(0);
	f.vertices.push_back(1);
	f.vertices.push_back(2);
	f.vertices.push_back(3);
	obj->model->faces.push_back(f);
	f.vertices.clear();

	obj->detectionConfidence = 1.0;

	// label object with incremtal raising number
	static unsigned numStereoObjects = 0;
	char obj_label[32];
	sprintf(obj_label, "Stereo object %d", numStereoObjects);
	obj->identLabels.push_back(obj_label);
	obj->identDistrib.push_back(1.0);
	numStereoObjects++;

	return true;
}

/**
 * @brief Delete all visual objects from the working memory.
 * The IDs are stored in the vector "objectIDs".
 */
void TestComponent::DeleteVisualObjectsFromWM()
{
	for(unsigned i=0; i<objectIDs.size(); i++)
		deleteFromWorkingMemory(objectIDs[i]);
	objectIDs.clear();
}

}









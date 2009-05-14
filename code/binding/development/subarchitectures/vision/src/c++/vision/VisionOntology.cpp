#include "VisionOntology.h"
#include <vision/idl/Vision.hh>

using namespace std; using namespace cast;
/*
const string VisionOntology::CAMERA_TYPE = "Visual:Camera";
const string VisionOntology::HEAD_TYPE = "Visual:Head";
const string VisionOntology::IMAGE_FRAME_TYPE = "Visual:ImageFrame";
const string VisionOntology::SCENE_CHANGED_TYPE = "Visual:SceneChanged";
const string VisionOntology::ROI_TYPE = "Visual:MatlabROI";
const string VisionOntology::SCENE_OBJECT_TYPE = "Visual:SceneObject";
const string VisionOntology::SURFACE_TYPE = "Visual:Surface";
const string VisionOntology::ATTENDED_OBJECT_LIST_TYPE = "Visual:AttendedObjectList";
*/


/**
 * \fn VisionOntology::VisionOntology()
 * \brief Initializes the mapping between VisionOntology's types (string) to 
 *        the framework datatypes.
 */
VisionOntology::VisionOntology() : CASTOntology() {
    addObjectType<Vision::Camera>(); 
    addObjectType<Vision::Head>(); 
    addObjectType<Vision::ImageFrame>();
    addObjectType<Vision::SceneChanged>();
    addObjectType<Vision::ROI>();
    addObjectType<Vision::SceneObject>();
    addObjectType<Vision::Surface>();
    addObjectType<Vision::AttendedObjectList>();
    addObjectType<Vision::LearnInstruction>();
    addObjectType<Vision::RecognisedAttributes>();
}


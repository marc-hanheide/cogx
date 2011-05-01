/**
 * @author Team RED
 * @date 2011
 * @brief Tools for Manipulation Planner
 */

#include "Tools.h"

namespace cast
{

using namespace std;


bool calculateGripperPosition(VisionData::VisualObject &obj, cogx::Math::Pose3 &gripperPose)
{

foo
}

  
  
}




//   class VisualObject {
//     // 3D position and orientation, in the robot ego coordinate system.
//     cogx::Math::Pose3 pose;
// 
//     // pose covariance matrix
//     // PoseVar3 var;
// 
//     // working memory IDs of the surface patches that belong to this object
//     IdSeq patchIds;
// 
//     // a value between 0 and 1 indicating confidence in the pose
//     // in future: poseConfidence
//     double detectionConfidence;
// 
//     // In case an exact pose can not be established (e.g. because the object
//     // detector model does not allow pose calculation) just provide a simple
//     // bounding volume.
//     // The position of the boundingSphere is relative to the object coord. sys.
//     cogx::Math::Sphere3 boundingSphere;
// 
//     // The time when the object was last observed, in any view
//     cast::cdl::CASTTime time;
// 
//     // ID of component with latest access to this WM entry
//     string componentID;
// 
//     // List of views of the object from different cameras
//     VisualObjectViewSeq views;
// 
//     // Geometric representation in 3D space
//     GeometryModel model;
// 
//     // The name with which we refer to the object linguistically
//     // TODO: label and labelConfidence should be removed as they are now replaced
//     // by the more generic identLabels and identDistrib
//     //string label;
//     // a value bwtween 0 and 1 indicating confidence in the label (i.e.
//     // typically confidence of the recognition process that produced the label)
//     //double labelConfidence;
// 
//     // a value between 0 and 1
//     double salience;
// 
//     // Distribution of identity labels (from object recognizer)
//     StringSeq identLabels;
//     DoubleSeq identDistrib;
//     double identGain;
//     double identAmbiguity;
//     // TODO: (maybe) gains same as with colors
// 
//     StringSeq colorLabels;
//     DoubleSeq colorDistrib;
//     DoubleSeq colorGains;
//     // colorGain = max(colorGains) ... for now;
//     double colorGain;
//     double colorAmbiguity;
// 
//     StringSeq shapeLabels;
//     DoubleSeq shapeDistrib;
//     DoubleSeq shapeGains;
//     // shapeGain = max(shapeGains) ... for now;
//     double shapeGain;
//     double shapeAmbiguity;
// 
//     // Object affordance
//     string affordance;
// 
//     // Source proto object
//     string protoObjectID;
// 
//     // HACK: Points in 2D
//     DoubleSeq points2D;
//   };

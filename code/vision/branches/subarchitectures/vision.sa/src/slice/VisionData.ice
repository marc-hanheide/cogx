#ifndef VISION_DATA_ICE
#define VISION_DATA_ICE

#include <CDL.ice>
#include <Math.ice>

module VisionData {

  struct VisualObjectView {
    // 2D bounding box in the image
    cogx::Math::Rect2 boundingBox;
    
    // a value between 0 and 1 indicating confidence in the pose
    double detectionConfidence;

    // ID of the viewing camera
    int camId;
  };

  sequence<VisualObjectView> VisualObjectViewSeq;

  sequence<int> IntSeq;

  struct Vertex {
    cogx::Math::Vector3 pos;
    cogx::Math::Vector3 normal;
    cogx::Math::Vector2 texCoord;
  };

  sequence<Vertex> VertexSeq;

  struct Face {
    IntSeq vertices;
  };

  sequence<Face> FaceSeq;

  class GeometryModel {
    VertexSeq vertices;
    FaceSeq faces;
    
    string label;
  };

  class VisualObject {
    // 3D position and orientation, in the robot ego coordinate system.
    cogx::Math::Pose3 pose;

    // a value between 0 and 1 indicating confidence in the pose
    double detectionConfidence;

    // In case an exact pose can not be established (e.g. because the object
    // detector model does not allow pose calculation) just provide a simple
    // bounding volume.
    // The position of the boundingSphere is relative to the object coord. sys.
    cogx::Math::Sphere3 boundingSphere;

    // The time when the object was last observed, in any view
    cast::cdl::CASTTime time;

    // List of views of the object from different cameras
    VisualObjectViewSeq views;
    
    // Geometric representation in 3D space
    GeometryModel model;

    // The name with which we refer to the object linguistically
    string label;
  };

  sequence<string> StringSeq;

  class DetectionCommand {
    StringSeq labels;
  };
  
  /** Commands for Object Tracker
   *  @author Thomas Mörwald
   */
  enum TrackingCommandType{ START, STOP, RELEASEMODELS };
  class TrackingCommand {
    TrackingCommandType cmd;
  };
};

#endif


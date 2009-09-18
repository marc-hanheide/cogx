#ifndef VISION_DATA_ICE
#define VISION_DATA_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>
#include <Video.ice>

module VisionData {

  sequence<cogx::Math::Vector3> Vector3Seq;

  sequence<string> IdSeq;

  // A planar surface patch
  // An oriented disk of a certain size, which lies in the local x-y plane.
  class SurfacePatch {
    // note: the z-axis of the local coordinate system is the outwards pointing
    // normal vector
    cogx::Math::Pose3 pose;

    // pose covariance matrix
    // PoseVar3 var;

    // size (diameter) of patch
    double size;

    // image patch on the surface (warped onto the surface from one of the
    // cameras)
    Video::Image image;

    // all the features we need (SIFT, MSER, ...)
    //Feature1 feature1;
    //Feature2 feature2;
  };

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
  };

  class VisualObject {
    // 3D position and orientation, in the robot ego coordinate system.
    cogx::Math::Pose3 pose;

    // pose covariance matrix
    // PoseVar3 var;

    // working memory IDs of the surface patches that belong to this object
    IdSeq patchIds;

    // a value between 0 and 1 indicating confidence in the pose
    // in future: poseConfidence
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

    // a value bwtween 0 and 1 indicating confidence in the label (i.e.
    // typically confidence of the recognition process that produced the label)
    double labelConfidence;
  };

  sequence<string> StringSeq;
  sequence<double> DoubleSeq;

  class DetectionCommand {
    StringSeq labels;
  };
  
  class DominantPlane {
  	cogx::Math::Plane3 plane;
  };
  
  /** Commands for Object Tracker
   *  @author Thomas Mörwald
   */
  enum TrackingCommandType{ START, STOP, RELEASEMODELS };
  class TrackingCommand {
    TrackingCommandType cmd;
  };

  /** 
   * @brief Object Detector Commands
   * @author Andreas Richtsfeld
   */
  enum ObjectDetectionCommandType{ DSTART, DSTOP};
  class ObjectDetectionCommand {
    ObjectDetectionCommandType cmd;
  };

  /**
   * Space of interest.
   * Coarsely described by a bounding sphere.
   * Somewhat finer (at least for some objects) described by a bounding box.
   * Later on we might even add a bounding convex hull as a yet finer
   * characterisation.
   */
  class SOI {
    cogx::Math::Sphere3 boundingSphere;
    cogx::Math::Box3 boundingBox;
    // time the SOI was last changed
    cast::cdl::CASTTime time;
    // This is a temporary solution only: provide the 3D points that gave rise
    // to this SOI, iff the SOI was created by plane pop-out.
    Vector3Seq points;   // frontground points
    Vector3Seq BGpoints; //background points
    Vector3Seq EQpoints; //equivocal points which either belongs to fg or bg
  };

  /**
   * 2D rectangular image region of interest.
   */
  class ROI {
    cogx::Math::Rect2 rect;
    // time the ROI was last changed
    cast::cdl::CASTTime time;
  };

  class ObjectRecognitionMatch {
    // ID of the SOI that was processed
    string soiId;

    // IDs of the internal representation of objects
    StringSeq objectId;

    // Probability of object match. Same order as objectId.
    DoubleSeq probability;
  };
  sequence<ObjectRecognitionMatch> ObjectRecognitionMatchSeq;

  class ObjectRecognitionTask {
    // REQUEST:
    IdSeq soiIds;

    // RESPONSE
    ObjectRecognitionMatchSeq matches;
  };
  
  class SegmentMask {
  	int width;
    int height;
    IntSeq data;
  };

  /**
   * Proto Object
   */
  class ProtoObject {
    IdSeq SOIList;
    
    // 2D image patch
    Video::Image image;
    
    // Segmentation mask;
    SegmentMask mask;
    
    // time the object was last changed
    cast::cdl::CASTTime time;
  };
};

#endif


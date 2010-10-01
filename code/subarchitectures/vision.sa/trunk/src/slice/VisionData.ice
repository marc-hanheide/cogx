#ifndef VISION_DATA_ICE
#define VISION_DATA_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>
#include <Video.ice>

module VisionData {

  sequence<cogx::Math::Vector3> Vector3Seq;
  sequence<string> StringSeq;
  sequence<double> DoubleSeq;

  /**
   * @brief A convex hull discribes
   * @author Kai Zhou
   */
  struct OneObj {
    Vector3Seq pPlane;
	// 3D vector sequence, describing the plane ???
    Vector3Seq pTop;
	// 3D vector sequence, describing the object top surface ???
  };

sequence<OneObj> ObjSeq;

  /**
   * @brief A convex hull describes ...???
   * @author Kai Zhou
   */
  class ConvexHull {
    cogx::Math::Pose3 center;
	// Pose of the center of the convex hull

    Vector3Seq PointsSeq;
	// points forming convex hull (stereo based) in camera coordinates
    cast::cdl::CASTTime time;
	// cast time

    double radius;
	// distance between center and farthest point from center
    double density;
	// ~ number of points in volume
    ObjSeq Objects;
	// Objects on this plane
    cogx::Math::Plane3 plane;
	// The estimated plane
 };

  sequence<string> IdSeq;

  // RGB color
  // NOTE: bytes in ICE are -128..127! So you will need to cast to an unsigned
  // char in your code.
  struct ColorRGB {
    byte r;
    byte g;
    byte b;
  };

  // A 3D point with a color, as e.g. returned by stereo
  struct SurfacePoint {
    cogx::Math::Vector3 p;
    VisionData::ColorRGB c;
  };

  sequence<SurfacePoint> SurfacePointSeq;

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

  /**
   * @brief ???
   * @author ???
   */
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
  sequence<bool> BoolSeq;

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

    // ID of component with latest access to this WM entry
    string componentID;

    // List of views of the object from different cameras
    VisualObjectViewSeq views;

    // Geometric representation in 3D space
    GeometryModel model;

    // The name with which we refer to the object linguistically
    // TODO: label and labelConfidence should be removed as they are now replaced
    // by the more generic identLabels and identDistrib
    //string label;
    // a value bwtween 0 and 1 indicating confidence in the label (i.e.
    // typically confidence of the recognition process that produced the label)
    //double labelConfidence;

    // a value between 0 and 1
    double salience;

    // Distribution of identity labels (from object recognizer)
    StringSeq identLabels;
    DoubleSeq identDistrib;
    double identGain;
    double identAmbiguity;

    StringSeq colorLabels;
    DoubleSeq colorDistrib;
    double colorGain;
    double colorAmbiguity;

    StringSeq shapeLabels;
    DoubleSeq shapeDistrib;
    double shapeGain;
    double shapeAmbiguity;

    // Source proto object
    string protoObjectID;
  };

  class DetectionCommand {
    StringSeq labels;
  };

  /**
   * @brief Dominant plane from PlanePopUp component.
   * @author Kai Zhou
   */
  class DominantPlane {
    cogx::Math::Plane3 plane;
  };

  /**
   * @brief Commands for Object Tracker
   * @author Thomas Mörwald
   */
  enum TrackingCommandType{ START, STOP, ADDMODEL, REMOVEMODEL, OVERWRITE, LOCK, UNLOCK, GETPOINT3D, RELEASEMODELS, SCREENSHOT };
  class TrackingCommand {
    TrackingCommandType cmd;
    string visualObjectID;
	// for ADDMODEL, REMOVEMODEL, LOCK, UNLOCK, GETPOINT3D
    VertexSeq points;
	// GETPOINT3D (Input: vec2 texCoord; Output: vec3 pos, vec3 normal)
    BoolSeq pointOnModel;
	// pointOnModel[i] is true if points[i] hits the VisualObject
  };

  /**
   * @brief Commands for ObjectRecognizer3D
   * @author Thomas Mörwald
   */
  enum Recognizer3DCommandType{ RECSTOP, RECLEARN, RECOGNIZE };
  class Recognizer3DCommand{
  	Recognizer3DCommandType cmd;
  	string label;
  	string visualObjectID;
  	double confidence;
  };

  /**
   * @brief Object Detector Commands
   * @author Andreas Richtsfeld
   */
  enum ObjectDetectionCommandType{ DSTART, DSTOP, DSINGLE };
  class ObjectDetectionCommand {
    ObjectDetectionCommandType cmd;
  };

  /**
   * @brief Stereo Flap Detector Commands
   * @author Andreas Richtsfeld
   */
  enum StereoFlapDetectionCommandType{ SFSTART, SFSTOP, SFSINGLE };
  class StereoFlapDetectionCommand {
    StereoFlapDetectionCommandType cmd;
  };

  /**
   * @brief Stereo Detector Commands
   * @author Andreas Richtsfeld
   */
  enum StereoDetectionCommandType{ SDSTART, SDSTOP, SDSINGLE, SDSINGLEHR };
  class StereoDetectionCommand {
    StereoDetectionCommandType cmd;
  };

  /**
   * @brief Stereo Detector Reasoner Commands
   * @author Andreas Richtsfeld
   */
  enum SDReasonerCommandType{ NEWFRAME };
  class SDReasonerCommand {
    SDReasonerCommandType cmd;
  };

  /**
   * @brief Objects for the reasoner component.
   * @author Andreas Richtsfeld
   */
  class ReasonerObject {
    VisualObject obj;
	// visual object from stereo detector
    int frameNr;
	// frame number
  };

  /**
   * @brief Space of interest.
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
    SurfacePointSeq points;
	// frontground points
    SurfacePointSeq BGpoints;
	// background points
    SurfacePointSeq EQpoints;
	// equivocal points which either belongs to fg or bg
    int status;
  };

  /**
   * 2D rectangular image Region Of Interest (ROI).
   */
  class ROI {
    cogx::Math::Rect2 rect;
    cast::cdl::CASTTime time;
	// time the ROI was last changed
  };

  class VisualLearnerRecognitionTask {
    // REQUEST:
    string protoObjectId;

    // RESPONSE
    StringSeq labels;
    IntSeq labelConcepts;
    DoubleSeq distribution;
    DoubleSeq gain;

    // ASYNC DATA
    string visualObjectId;
  };

  class VisualLearningTask {

    string visualObjectId;
    string beliefId;

    // color, shape, type; maybe also shape3D
    string concept;
    StringSeq labels;
    DoubleSeq weights;
  };


  struct SegmentMask {
    int width;
    int height;
    Video::ByteSeq data;
  };

  /**
   * Relative Angle and Scale (RAS) - a coarse 3D shape descriptor.
   * It encodes turning angles between adjacent planar patches of object surface.
   * Angles range from 0..2 pi, where angles > pi represent concavities.
   * (note: relative scale has been taken out for now)
   * @author Hannes Prankl (prankl@acin.tuwien.ac.at)
   * @author Michael Zillich (zillich@acin.tuwien.ac.at)
   */
  struct RASShapeDescriptor {
    // normalised histogram of angles
    // bin size is 2 pi / size
    DoubleSeq angleHistogram;
  };

  /**
   * Proto Object
   */
  class ProtoObject {

    // List of source SOIs
    IdSeq SOIList;

    // 2D image patch
    Video::Image image;

    // Segmentation mask;
    SegmentMask mask;

    // List of surface 3D points
    SurfacePointSeq points;
    
    // RAS shape descriptor
    RASShapeDescriptor rasShapeDesc;

    // time the object was last changed
    cast::cdl::CASTTime time;
  };

 /**
  * Person detected in camera a laser.
  * @author Nick Hawes
  */
  class Person {
    // angle and distance from robot
    double angle;
    double distance;
    // 3d position of person relative to the robot
    double locX;
    double locZ;
    // possible direction of movement in last update
    double deltaX;
    double deltaZ;
  };

 /**
  * Command to trigger people detection
  * @author Nick Hawes
  */
  class PeopleDetectionCommand {
  	//nothing involved here
  };

  /**
  *
  * Model foregrounded for use by recognisers etc.
  * @author Nick Hawes
  */
  class ForegroundedModel {
    string model;
  };

};

#endif


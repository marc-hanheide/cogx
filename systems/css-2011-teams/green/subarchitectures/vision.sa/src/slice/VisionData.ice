#ifndef VISION_DATA_ICE
#define VISION_DATA_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>
#include <Video.ice>
#include <PointCloud.ice>


module VisionData {

  sequence<cogx::Math::Vector3> Vector3Seq;
  sequence<string> StringSeq;
  sequence<double> DoubleSeq;
  sequence<cast::cdl::WorkingMemoryAddress> WorkingMemoryAddressSeq;
  
	enum CommandCompletion {
		FAILED,
		SUCCEEDED,
		COMPINIT
	};
	
	enum CommandStatus {
		NEW,
		CHANGED,
		PENDING,
		FINISHED,
		COMMANDFAILED
	};

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


  /**
   * A planar surface patch
   * @author Michael Zillich
   */
  struct SurfacePatch {
    // note: the z-axis of the local coordinate system is the outwards pointing
    // normal vector
    cogx::Math::Pose3 pose;

    // 3D points used to estimate this patch
    PointCloud::SurfacePointSeq points;
  };
  
  sequence<SurfacePatch> SurfacePatchSeq;

  /**
   * @brief Data related to one (of possibly several) view on an object.
   * @author Michael Zillich
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
    // TODO: (maybe) gains same as with colors

    StringSeq colorLabels;
    DoubleSeq colorDistrib;
    DoubleSeq colorGains;
    // colorGain = max(colorGains) ... for now;
    double colorGain;
    double colorAmbiguity;

    StringSeq shapeLabels;
    DoubleSeq shapeDistrib;
    DoubleSeq shapeGains;
    // shapeGain = max(shapeGains) ... for now;
    double shapeGain;
    double shapeAmbiguity;

    // Object affordance
    string affordance;

    // Source proto object
    string protoObjectID;

    // HACK: Points in 2D
    DoubleSeq points2D;
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
  enum TrackingCommandType{ START, STOP, ADDMODEL, ADDMODELFROMFILE, REMOVEMODEL, OVERWRITE, LOCK, UNLOCK, GETPOINT3D, RELEASEMODELS, SCREENSHOT };
  class TrackingCommand {
    TrackingCommandType cmd;
    string visualObjectID;
	// for ADDMODEL, REMOVEMODEL, LOCK, UNLOCK, GETPOINT3D
    VertexSeq points;
	// GETPOINT3D (Input: vec2 texCoord; Output: vec3 pos, vec3 normal)
    BoolSeq pointOnModel;
	// pointOnModel[i] is true if points[i] hits the VisualObject
    string plyfile;
  };

  /**
   * @brief Commands for ObjectRecognizer3D
   * @author Thomas Mörwald
   */
  enum Recognizer3DCommandType{ RECSTOP, RECLEARN, RECOGNIZE, LOOKGRASP };
  class Recognizer3DCommand{
  	Recognizer3DCommandType cmd;
  	string label;
  	string visualObjectID;
  	double confidence;
  };
  
  /**
   * @brief Commands for ObjectRecognizer3DDriver (also miss-using as grasp driver)
   * @author Thomas Mörwald
   */
   class LookForObjectCommand{
   		WorkingMemoryAddressSeq foundVisualObjects;
   		double pan;
   		double tilt;
   		CommandCompletion comp;
   		CommandStatus status;
   		
   };
   class GraspForObjectCommand{
   		string label;
		CommandCompletion comp;
   		CommandStatus status;
   };
   class DropObjectCommand{
		CommandCompletion comp;
   		CommandStatus status;
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
    VisualObject obj;   // visual object from stereo detector
    int frameNr;        // frame number
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
    // frontground points
    PointCloud::SurfacePointSeq points;
    // background points
    PointCloud::SurfacePointSeq BGpoints;
    // equivocal points which either belongs to fg or bg
    PointCloud::SurfacePointSeq EQpoints;
    int status;
  };

  /**
   * 2D rectangular image Region Of Interest (ROI).
   */
  class ROI {
    cogx::Math::Rect2 rect;
    // time the ROI was last changed
    cast::cdl::CASTTime time;
  };

  class VisualLearnerRecognitionTask {
    // REQUEST:
    cast::cdl::WorkingMemoryAddress protoObjectAddr;

    // RESPONSE
    StringSeq labels;
    IntSeq labelConcepts;
    DoubleSeq distribution;
    DoubleSeq gains;

    // ASYNC DATA
    string visualObjectId;
  };

  // (review2010): processed by VisualLearner
  class AffordanceRecognitionTask {
    // REQUEST:
    cast::cdl::WorkingMemoryAddress protoObjectAddr;

    // RESPONSE
    string affordance;
  };

  class VisualLearningTask {
    string visualObjectId;
    string beliefId;

    // superConcept: color, shape, type; maybe also shape3D
    string concept;
    // concepts: red, compact, box
    StringSeq labels;
    DoubleSeq weights;
  };

  class VisualConceptModelStatus {
    // superConcept: color, shape, type; maybe also shape3D
    string concept;

    // concepts: red, compact, box
    StringSeq labels;
    DoubleSeq gains;
    BoolSeq askedFor;
  };


  /**
   * The segmentation mask is a grayscale image.
   */
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

    // (review2010) Size of the original image from which the patch and mask were created
    cogx::Math::Vector2 sourceImageSize;
    // (review2010) The top-left position of the image patch and mask in the original image, in pixel coordinates
    cogx::Math::Vector2 imageOrigin;

    // List of all surface 3D points
    PointCloud::SurfacePointSeq points;

    // RAS shape descriptor
    RASShapeDescriptor rasShapeDesc;

    // segmented planar surface patches
    SurfacePatchSeq surfacePatches;
  
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

  class Post3DObject{
    string label;
    cogx::Math::Pose3 pose;
  };
};

#endif
// vim:set sw=2 ts=8 et ai:
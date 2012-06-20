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
  sequence<int> IntSeq;
  sequence<bool> BoolSeq;
  sequence<cast::cdl::WorkingMemoryPointer> WorkingMemoryPointerSeq;
  sequence<string> IdSeq;

  const string IDENTITYxUNKNOWN = "unknown";

/**
* Enum for reporting status of vision commands. 
*/
enum VisionCommandStatus {
  //probably too much overhead ;)
  VCSUCCEEDED,
  VCFAILED,
  VCREQUESTED,
};

  /**
   * @brief A convex hull discribes
   * @author Kai Zhou
   */
  struct OneObj {
    // 3D vector sequence, describing the plane ???
    Vector3Seq pPlane;
    // 3D vector sequence, describing the object top surface ???
    Vector3Seq pTop;
  };
  sequence<OneObj> ObjSeq;

  /**
   * @brief A convex hull describes ...???
   * @author Kai Zhou
   */
  class ConvexHull {
    // Pose of the center of the convex hull
    cogx::Math::Pose3 center;

    // points forming convex hull (stereo based) in camera coordinates
    Vector3Seq PointsSeq;
    // cast time
    cast::cdl::CASTTime time;

    // distance between center and farthest point from center
    double radius;
    // ~ number of points in volume
    double density;
    // Objects on this plane
    ObjSeq Objects;
    // The estimated plane
    cogx::Math::Plane3 plane;
  };


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

  // @see: VisualObject::presence
  enum VisualObjectPresence {
    VopUNKNOWN,
    VopVISIBLE,
    VopWasVISIBLE,
    VopREMOVED
  };

  class VisualObject {
    // 3D position and orientation, in the robot ego coordinate system (ECS).
    // TODO: when the robot moves, it will take the ECS with it. We should tie object poses
    // to anchors instead of the ECS. The other option is to update the positions of all objects.
    cogx::Math::Pose3 pose;

    // 2011-10-03: apparently the link to PO is not enough for some VO uses so the following
    // states are aded:
    //   UNKNOWN - default when the object is created with createVisualObject()
    //   VISIBLE - the object is visible; the VO is visible and linked to a PO (via protoObject)
    //   WAS_VISIBLE - the camera turned and the object is no longer in view; the PO is still
    //      in WM, but the link in protoObject is empty; the field lastProtoObject holds the
    //      previous value of protoObject
    //   REMOVED - the object was removed from the scene; the PO was removed from WM; the link in 
    //      protoObject is empty; the link in lastProtoObject may be invalid.
    VisualObjectPresence presence;

    // The *visible* proto object associated with this VO.
    cast::cdl::WorkingMemoryPointer protoObject;
    // The proto object that was last associated with this VO.
    // Same as protoObject when the object is visible.
    cast::cdl::WorkingMemoryPointer lastProtoObject;

    // working memory IDs of the surface patches that belong to this object
    IdSeq patchIds;

    // a value between 0 and 1 indicating confidence in the pose
    // in future: poseConfidence
    double detectionConfidence;

    // In case an exact pose can not be established (e.g. because the object
    // detector model does not allow pose calculation) just provide a simple
    // bounding volume.
    // The position of the boundingSphere is relative to the robot ego coord. sys.
    cogx::Math::Sphere3 boundingSphere;

    // pose covariance matrix
    // PoseVar3 var;

    // The time when the object was last observed, in any view
    cast::cdl::CASTTime time;

    // ID of component with latest access to this WM entry
    string componentID;

    // List of views of the object from different cameras
    VisualObjectViewSeq views;

    // Geometric representation in 3D space
    // TODO: make a WM Pointer: ProtoObjectGeometryModel { PO-Ptr, GeometryModel }
    GeometryModel model;

    // The name with which we refer to the object linguistically
    // TODO: label and labelConfidence should be removed as they are now replaced
    // by the more generic identLabels and identDistrib
    //string label;
    // a value bwtween 0 and 1 indicating confidence in the label (i.e.
    // typically confidence of the recognition process that produced the label)
    //double labelConfidence;

    // Y1-Y3: a value between 0 and 1
    // Y4: 
    //   - contains a timestamp, getCASTTime converted to double
    //     (tm.s + 1e-6 * tm.us)
    //   - set when the VO is created, ie. the object's first appearnce on the
    //     scene (this is done in SOIFilter/TaskAnalyzePO.cpp)
    //   - updated if the robot points to the object
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

    // Y3, OBSOLETE: use protoObject/lastProtoObject
    // Source proto object
    // string protoObjectID;

    // HACK: Points in 2D
    DoubleSeq points2D;
  };

  /**
   * Ask the 3D object recognizer to detect any of the objects given in the
   * label list in the whole image.
   *
   * If the list of labels is empty, it will check all the labels known to the
   * recognizer.
   *
   * @author Michael Zillich
   */
  class DetectionCommand {
    StringSeq labels;
  };

  /**
   * Ask the 3D object recognizer to recognize a given as yet unknown visual
   * object, using only the image region defined by the bounding sphere of that
   * object. It checks for all labels given in the list.
   * 
   * If the list of labels is empty, it will check all the labels known to the
   * recognizer.
   *
   * @author Michael Zillich
   */
  class RecognitionCommand {
    cast::cdl::WorkingMemoryPointer visualObject;
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
    // for ADDMODEL, REMOVEMODEL, LOCK, UNLOCK, GETPOINT3D
    string visualObjectID;
    // GETPOINT3D (Input: vec2 texCoord; Output: vec3 pos, vec3 normal)
    VertexSeq points;
    // pointOnModel[i] is true if points[i] hits the VisualObject
    BoolSeq pointOnModel;
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
    // @attr sourceId identifies the hardware setup from which the SOI was generated.
    // Currently PPO sets it to component-id.
    string sourceId;

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
    //nah: what is this for? what are the possible values?
    int status;
  };
  sequence<SOI> SOISeq;


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
    cast::cdl::WorkingMemoryPointer protoObjectAddr;

    // RESPONSE
    VisionCommandStatus status;

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
    cast::cdl::WorkingMemoryPointer protoObjectAddr;

    // RESPONSE
    VisionCommandStatus status;

    string affordance;
  };

  class VisualLearningTask {
    // REQUEST:
    //string visualObjectId; replaced by visualObjectAddr
    cast::cdl::WorkingMemoryPointer visualObjectAddr;
    string beliefId;

    // superConcept: color, shape, type; maybe also shape3D
    string concept;
    // concepts: red, compact, box
    StringSeq labels;
    DoubleSeq weights;

    // RESPONSE
    VisionCommandStatus status;
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

  // @author mmarko
  class ViewCone {
    // An anchor is a known position of the robot.
    // The robot should be able to return to the anchor. In principle the
    // anchor doesn't have world coordinates, it's just a configuration in
    // space we can return to. While we have an anchor we can move the robot in
    // the relative coordinate system defined by the anchor.  ATM the anchor is
    // represented as an absolute position in the world coordinate system: x,
    // y, z=theta. DON'T COUNT ON IT, it is very likely to change.
    cogx::Math::Vector3 anchor;

    // all values are relative to the anchor
    double x;
    double y;

    // viewDirection is parallel to x-y plane; relative to anchor.z (theta)
    double viewDirection;

    // the vertical angle of the Pan-Tilt unit where the cameras are mounted
    double tilt;

    // the object that this was created for
    cast::cdl::WorkingMemoryPointer target;

  };
  sequence<ViewCone> ViewConeSeq;

  /// Cost to robot to move from one viewcone to another. This assumes that the transition is directed.
  /// @author Nick Hawes (n.a.hawes@cs.bham.ac.uk)
  class ViewConeMoveCost {
    cast::cdl::WorkingMemoryPointer from;
    cast::cdl::WorkingMemoryPointer to;
    double cost;
  };
  sequence<ViewConeMoveCost> ViewConeMoveCostSeq;

  ////List of viewcone move costs
  class ViewConeMoveCostList {
    ViewConeMoveCostSeq costs;
  };
  
  // A command to move the robot and the PTU so that the target ViewCone is
  // reached. The movements are performed in local coordinate frame relative
  // to target.anchor.
  // @author mmarko
  class MoveToViewConeCommand {
    // @param ViewCone target
    cast::cdl::WorkingMemoryPointer target;

    //nah: took this out until we need it.. then replace with enum
    // eg. look-at-object; maybe use enum instd of string
    //string reason;

    //nah: took this out until we need it
    // arbitrary callers reference to a WM entry; depends on reason.
    // cast::cdl::WorkingMemoryPointer object;

    // the result passed on overwrite
    VisionCommandStatus status;
  };

  // A command to analyze the proto-object that came into the visual field of
  // the precise stereo (after MoveToViewConeCommand). The command should wait
  // for precise SOIs to be generated.
  // @author mmarko
  class AnalyzeProtoObjectCommand {
    // which proto object
    // @param ProtoObject-address protoObjectAddr;
    cast::cdl::WorkingMemoryAddress protoObjectAddr;

    // @param ViewCone-address whereToLook
    // where in the visual field is it? This should be (approx.) the same as
    // the target of the MoveToViewConeCommand command. If the target could not
    // be reached, a new view cone is generated from the desired and the
    // reached VCs and passed to AnalyzeProtoObjectCommand. The anchor must be
    // the same in all 3 VCs. The (x,y) location of viewCone is the current
    // location of the robot. The viewDirection and tilt of viewCone are used
    // to find the SOIs for the ProtoObject in the scene.
    // (maybe it would be enough to use panDelta and tiltDelta instead of a ViewCone)
    cast::cdl::WorkingMemoryPointer whereToLook;


    // the result passed on overwrite
    VisionCommandStatus status;
  };

  class GetStableSoisCommand {
    // ID of the component that should provide the SOIs
    string componentId;
    SOISeq sois;

    // the result passed on overwrite
    VisionCommandStatus status;
  };

  // A LookAroundCommand will make the robot look in a new direction
  // to see if there is anything interesting. It will generate ViewCones that
  // could be visited.
  class LookAroundCommand {
    // the result passed on overwrite
    ViewConeSeq viewCones;
    VisionCommandStatus status;
  };

  /**
   * Proto Object
   */
  class ProtoObject {

    // List of source SOIs
    IdSeq SOIList;

    // Position of the camera 
    ViewCone cameraLocation;

    // The desired positions of the camera should have the same anchor as the cameraLocation
    WorkingMemoryPointerSeq desiredLocations;

    // The position of the object relative to the anchor (the position of the robot)
    // in cameraLocation. The center of a SOI is used.
    cogx::Math::Vector3 position;

    // 2D image patch
    // TODO: make a WM Pointer: ProtoObjectImage { PO-Ptr, Image } 
    Video::Image image;

    // Segmentation mask;
    // TODO: make a WM Pointer: ProtoObjectMask { PO-Ptr, Mask }
    SegmentMask mask;

    // (review2010) Size of the original image from which the patch and mask were created
    cogx::Math::Vector2 sourceImageSize;
    // (review2010) The top-left position of the image patch and mask in the original image, in pixel coordinates
    cogx::Math::Vector2 imageOrigin;

    // List of all surface 3D points
    // TODO: make a WM Pointer: ProtoObjectSuefacePoints { PO-Ptr, SurfacePointSeq }
    PointCloud::SurfacePointSeq points;

    // RAS shape descriptor
    RASShapeDescriptor rasShapeDesc;

    // segmented planar surface patches
    // TODO: make a WM Pointer: ProtoObjectSurfacePatches { PO-Ptr, SurfacePatchSeq }
    // XXX: is this used at all?
    SurfacePatchSeq surfacePatches;

    // The visual object created from this PO. size() == 0 if none created yet, otherwise size() == 1
    WorkingMemoryPointerSeq visualObject;

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
    double existProb;
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
	bool positiveDetection;
  };


  // A gerenic RPC call that uses WM filters.
  // The receiver responds and deletes the entry after it has been read.
  // Created for the purpose of VisualLearner/test/gy2article, used in Analyzer.
  dictionary<string, string> TStringMap;
  class WMRemoteProcedureCall {
     // TODO: component id: only this component will react to the call.
     string method;
     TStringMap argmap;
  };
  
  /**
  *
  * Command for AR tag recognizer
  * @author Alper Aydemir
  */
  class ARTagCommand{
  	string label;
  };
};

#endif
// vim:set sw=2 ts=8 et ai:

#ifndef FRONTIERINTERFACE_ICE
#define FRONTIERINTERFACE_ICE

#include <cast/slice/CDL.ice>
#include <NavData.ice>
#include <SpatialData.ice>
/**
 * Defines interface for reading exploration frontiers from SpatialControl
 *
 * @author Kristoffer Sjöö
 * @see
 */

module FrontierInterface {


  sequence<double> DoubleData;
  sequence<string> StringSeq;
  sequence<int> IntSeq;
  sequence<cogx::Math::Vector3> Vec3Seq;
  sequence<cogx::Math::Vector2> Vec2Seq;
  sequence<cogx::Math::Pose3> PoseSeq;

  dictionary<int, IntSeq> AdjacencyLists;

  enum FrontierPtStatus {
    FRONTIERSTATUSUNKNOWN,
    FRONTIERSTATUSOPEN,
    FRONTIERSTATUSCURRENT,
    FRONTIERSTATUSUNREACHABLE,
    FRONTIERSTATUSPATHBLOCKED,
    FRONTIERSTATUSGATEWAYBLOCKED,
  };

  class FrontierPt {
    double mWidth;
    int mState;

    double x;
    double y;
  };

  sequence<FrontierPt> FrontierPtSeq;

  interface FrontierReader {
    FrontierPtSeq getFrontiers();
  };
  interface FrontierReaderAsComponent extends cast::interfaces::CASTComponent {
    FrontierPtSeq getFrontiers();
  };

  struct HypothesisEvaluation 
  {
    float freeSpaceValue;
    float unexploredBorderValue;
    float gatewayValue;
  };

  /**
   * Container for data in a GridMap<double>
   * @author Kristoffer Sjöö
   */
  class GridMapDouble {
    double x;
    double y;
    double cellSize;
    int size;
    DoubleData contents;
  };

  class WeightedPointCloud
  {
    cogx::Math::Vector3 center;
    double interval;
    int xExtent;
    int yExtent;
    int zExtent;
    DoubleData values;
    bool isBaseObjectKnown;
  };


  enum ObjectRelation { ON, IN };
  sequence<ObjectRelation> RelationSeq;
  /**
   * Command to compute probability distributions
   * @author Kristoffer Sjöö
   */
  class ObjectPriorRequest {
    RelationSeq relationTypes;
//Starts with the query object's label
    StringSeq objects; 
    double cellSize;
    double totalMass;
    WeightedPointCloud outCloud;

// If this argument is present, the 
    			// base object (last in 'objects') 
    			// will be temporarily set to the
    			// pose in question, not randomized
    			// nor its known position used
    PoseSeq baseObjectPose; 
  };

  /**
   * Command to compute tilt angles from a relation
   * @author Kristoffer Sjöö
   */
  class ObjectTiltAngleRequest {
    ObjectRelation relationType;
//Starts with the query object's label
    StringSeq objects;
//Describes the 2D triangle in which to sample for points 
    Vec2Seq triangle; 
    Vec3Seq tiltAngles; 
  };


  class ObservedPlaneObject
  {
    int id;
    string label;

    cogx::Math::Vector3 pos;
    double angle;
  };

  class DoorHypothesis
  {
    double x;
    double y;
 //Direction of door's line
    double theta;
//End-to-end width
    double width; 
  };

  struct PlaceMembership
  {
    int placeID;
    double confidence;
  };

  interface LocalMapInterface {
    SpatialData::LocalGridMap getCombinedGridMap(SpatialData::PlaceIDSeq places);
  };
  interface LocalMapInterfaceAsComponent extends cast::interfaces::CASTComponent {
    SpatialData::LocalGridMap getCombinedGridMap(SpatialData::PlaceIDSeq places);
  };

  interface HypothesisEvaluator {
    HypothesisEvaluation getHypothesisEvaluation(int hypID);
  };
  interface HypothesisEvaluatorAsComponent extends cast::interfaces::CASTComponent {
    HypothesisEvaluation getHypothesisEvaluation(int hypID);
  };

  interface PlaceInterface {
    SpatialData::NodeHypothesis getHypFromPlaceID(int placeID);
    NavData::FNode getNodeFromPlaceID(int placeID);
    SpatialData::Place getPlaceFromNodeID(int nodeID);
    SpatialData::Place getPlaceFromHypID(int hypID);
    void beginPlaceTransition(int goalPlaceID);
    void endPlaceTransition(int failed);
    SpatialData::Place getCurrentPlace();
    FrontierInterface::PlaceMembership getPlaceMembership(double x, double y);
    int updatePlaceholderEdge(int placeholderId);
    AdjacencyLists getAdjacencyLists();
  };
  interface PlaceInterfaceAsComponent extends cast::interfaces::CASTComponent {
    SpatialData::NodeHypothesis getHypFromPlaceID(int placeID);
    NavData::FNode getNodeFromPlaceID(int placeID);
    SpatialData::Place getPlaceFromNodeID(int nodeID);
    SpatialData::Place getPlaceFromHypID(int hypID);
    void beginPlaceTransition(int goalPlaceID);
    void endPlaceTransition(int failed);
    SpatialData::Place getCurrentPlace();
    FrontierInterface::PlaceMembership getPlaceMembership(double x, double y);
  };

  interface RelationInterface {
    StringSeq getObjectRelationProbabilities(string obj);
  };

  interface RelationInterfaceAsComponent extends cast::interfaces::CASTComponent {
    StringSeq getObjectRelationProbabilities(string obj);
  };
};

#endif // FRONTIERINTERFACE_ICE

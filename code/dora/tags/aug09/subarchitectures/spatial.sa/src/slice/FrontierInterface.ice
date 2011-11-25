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
  class NodeHypothesis {
    double x;
    double y;

    int hypID;
    int originPlaceID;
  };

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

  interface PlaceInterface {
    NodeHypothesis getHypFromPlaceID(int placeID);
    NavData::FNode getNodeFromPlaceID(int placeID);
    SpatialData::Place getPlaceFromNodeID(int nodeID);
    SpatialData::Place getPlaceFromHypID(int hypID);
    void beginPlaceTransition(int goalPlaceID);
    void endPlaceTransition(int failed);
  };
  interface PlaceInterfaceAsComponent extends cast::interfaces::CASTComponent {
    NodeHypothesis getHypFromPlaceID(int placeID);
    NavData::FNode getNodeFromPlaceID(int placeID);
    SpatialData::Place getPlaceFromNodeID(int nodeID);
    SpatialData::Place getPlaceFromHypID(int hypID);
    void beginPlaceTransition(int goalPlaceID);
    void endPlaceTransition(int failed);
  };
};

#endif // FRONTIERINTERFACE_ICE
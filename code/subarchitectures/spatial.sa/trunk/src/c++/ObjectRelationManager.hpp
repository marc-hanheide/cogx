//
// = FILENAME
//    ObjectRelationManager.hpp
//
// = FUNCTION
//    Evaluate spatial relations between objects and between objects and places,
//    and write appropriate Property structs to Spatial WM
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#ifndef ObjectRelationManager_hpp
#define ObjectRelationManager_hpp

#include <cast/architecture/ManagedComponent.hpp>
#include <SpatialProperties.hpp>
#include <FrontierInterface.hpp>
#include <map>
#include <peekabot.hh>
#include "RelationEvaluation.hpp"
#include <PTZ.hpp>
#include <NavData.hpp>
#include <VisionData.hpp>

using namespace SpatialProperties;
using namespace SpatialData;

namespace spatial {

/**
 * This class monitors the sensory layer for objects and their poses, and writes
 * spatial relation structs to WM as appropriate
 *
 * @author Kristoffer Sjöö
 * @see
 */
class ObjectRelationManager : public cast::ManagedComponent
{
public:

  ObjectRelationManager ();
  virtual ~ObjectRelationManager();

  virtual void runComponent();
  virtual void start();

protected:
  int m_maxObjectCounter;

  //Map from SpatialObject::id to spatial objects managed by this component
  std::map<std::string, SpatialObjectPtr> m_objects; 
  std::map<std::string, spatial::Object*> m_objectModels;
  std::map<std::string, std::string> m_objectWMIDs; 
  std::map<std::string, std::string> m_visualObjectIDs;

  std::map<std::string, PlaneObject> m_planeObjectModels;
  std::map<std::string, FrontierInterface::ObservedPlaneObjectPtr> m_planeObjects;
  std::map<std::string, cast::cdl::CASTTime> m_lastPlaneObjectPoseTimes;

  NavData::RobotPose2dPtr lastRobotPose;
//  std::map<int, Pose3> m_lastKnownObjectPoses;
  std::map<std::string, cast::cdl::CASTTime> m_lastObjectPoseTimes;
  Pose3 m_CameraPoseR;

  //For keeping track of when the robot is moving (to init the tracker)
  double m_standingStillThreshold;
  double m_timeSinceLastMoved;
  double m_trackerTimeThreshold;
  double m_recognitionTimeThreshold;
  bool m_bRecognitionIssuedThisStop;


  std::map<std::string, PlaceContainmentObjectPropertyPtr> m_containmentProperties;
  std::map<std::string, std::string> m_containmentPropWMIDs; 

  FrontierInterface::PlaceInterfacePrx m_placeInterface;

  bool m_bTestOnness;
  bool m_bSampleOnness;
  bool m_bTestInness;
  bool m_bSampleInness;
  bool m_bDemoSampling;
  bool m_bNoPTZ;
  bool m_bNoVision;

  bool m_bDisplayPlaneObjectsInPB;
  bool m_bDisplayVisualObjectsInPB;

  std::string m_planeModelFilename;

  peekabot::PeekabotClient m_PeekabotClient;  
  peekabot::GroupProxy m_planeProxies;
  peekabot::GroupProxy m_objectProxies;
  peekabot::GroupProxy m_onnessTester;
  peekabot::GroupProxy m_innessTester;
  std::string m_PbHost;
  int m_PbPort;
  int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry


  void connectPeekabot();

  Pose3 getCameraToWorldTransform();

  virtual void configure(const std::map<std::string, std::string>& _config);

  void newRobotPose(const cast::cdl::WorkingMemoryChange &);
  void newObject(const cast::cdl::WorkingMemoryChange &);
  void objectChanged(const cast::cdl::WorkingMemoryChange &);

  void newPlaneObject(const cast::cdl::WorkingMemoryChange &);
  void readPlaneModelsFromFile();

  void generateNewObjectModel(const std::string &label);

  void recomputeOnnessForObject(const std::string &label);
  void recomputeOnnessForPlane(const std::string &label);
  void recomputeInnessForObject(const std::string &label);

  void sampleOnnessForPlane(const std::string &planeLabel, 
      const std::string &objectLabel);
  void sampleOnnessForObject(const std::string &supLabel, 
      const std::string &onLabel);
  void sampleOnnessRecursively(const std::vector<std::string> &objects, 
    int currentLevel, unsigned int nSamplesPerStep, unsigned int nMaxSamples,
    std::vector<cogx::Math::Vector3> &outPoints, spatial::Object *supportObject,
    const std::vector<Vector3> &triangle = std::vector<Vector3>());

  void newTiltAngleRequest(const cast::cdl::WorkingMemoryChange &);
  void newPriorRequest(const cast::cdl::WorkingMemoryChange &);

  void setContainmentProperty(const std::string &objectLabel, int placeID, double confidence);
  void setSupportProperty(int figureID, int groundID, double confidence);

  void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd,
      std::string label, std::string visualObjectID);
  void addTrackerCommand(VisionData::TrackingCommandType cmd, 
      std::string label);

  ptz::PTZInterfacePrx m_ptzInterface;
}; 

std::vector<cogx::Math::Vector3>
findPolygonIntersection(const std::vector<cogx::Math::Vector3> &polygon1, 
    const std::vector<cogx::Math::Vector3> &polygon2);

double getPolygonArea(const std::vector<cogx::Math::Vector3> &polygon);


}; // namespace spatial

#endif // ObjectRelationManager_hpp

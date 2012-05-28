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
#include <set>
#include <peekabot.hh>
#include "RelationEvaluation.hpp"
#include "DensitySampling.hpp"
#include <PTZ.hpp>
#include <NavData.hpp>
#include <VisionData.hpp>

using namespace SpatialProperties;
using namespace SpatialData;

namespace Cure {
template<class T>
class LocalGridMap;
}
typedef std::pair<std::string, std::string> StrPair;

namespace spatial {

/**
 * This class monitors the sensory layer for objects and their poses, and writes
 * spatial relation structs to WM as appropriate
 *
 * @author Kristoffer Sjöö
 * @see
 */
class ObjectRelationManager: public cast::ManagedComponent {
public:

	ObjectRelationManager();
	virtual ~ObjectRelationManager();

	virtual void runComponent();
	virtual void start();

protected:
	int m_maxObjectCounter;

	RelationEvaluator m_evaluator;

	//Map from SpatialObject::id to spatial objects managed by this component
	std::map<std::string, SpatialObjectPtr> m_objects;
	std::map<std::string, spatial::Object*> m_objectModels;
	std::map<std::string, std::string> m_objectWMIDs;
	//  std::map<std::string, std::string> m_visualObjectIDs;

	std::map<std::string, PlaneObject> m_planeObjectModels;
	std::map<std::string, FrontierInterface::ObservedPlaneObjectPtr>
			m_planeObjects;
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

	std::set<cdl::WorkingMemoryAddress> m_updatedObjects;
	//Pair order: (trajector, landmark)
	std::map<StrPair, double> m_objectOnnessValues;
	std::map<StrPair, double> m_objectInnessValues;

	std::map<std::string, PlaceContainmentObjectPropertyPtr>
			m_containmentProperties;
	//  std::map<std::string, std::string> m_containmentPropWMIDs; 

	FrontierInterface::PlaceInterfacePrx m_placeInterface;

	bool m_bNoPTZ;
	bool m_bDetectObjects;

	bool m_bDisplayPlaneObjectsInPB;
	bool m_bDisplayVisualObjectsInPB;

	std::string m_planeModelFilename;

	std::set<std::string> m_lookForObjects;

	peekabot::PeekabotClient m_PeekabotClient;
	peekabot::GroupProxy m_planeProxies;
	peekabot::GroupProxy m_objectProxies;
	std::string m_PbHost;
	int m_PbPort;
	int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry

	void connectPeekabot();

	void readRelationsFromFile(const string &filename);

	Pose3 getCameraToWorldTransform();

	virtual void configure(const std::map<std::string, std::string>& _config);

	void newRobotPose(const cast::cdl::WorkingMemoryChange &);
	void newObject(const cast::cdl::WorkingMemoryChange &);
	void newObject(const cast::cdl::WorkingMemoryAddress &);
	void objectChanged(const cast::cdl::WorkingMemoryChange &);

	void newPlaneObject(const cast::cdl::WorkingMemoryChange &);
	void readPlaneModelsFromFile();

	void recomputeOnnessForObject(const std::string &label);
	void recomputeOnnessForPlane(const std::string &label);
	void recomputeInnessForObject(const std::string &label);
	void runInference();

	//  void sampleOnnessForPlane(const std::string &planeLabel, 
	//      const std::string &objectLabel);
	//  void sampleOnnessForObject(const std::string &supLabel, 
	//      const std::string &onLabel);

	//  void newTiltAngleRequest(const cast::cdl::WorkingMemoryChange &);
	void newPriorRequest(const cast::cdl::WorkingMemoryChange &);
	void processPriorRequest(FrontierInterface::ObjectPriorRequestPtr request);
	void new3DPriorRequest(const cast::cdl::WorkingMemoryChange &wmc);

	void setContainmentProperty(const std::string &objectLabel, int placeID,
			double confidence);
	void setSupportProperty(int figureID, int groundID, double confidence);

	void addRecognizer3DCommand(VisionData::Recognizer3DCommandType cmd,
			std::string label, std::string visualObjectID);
	void
			addTrackerCommand(VisionData::TrackingCommandType cmd, std::string label);

	vector<string> computeMarginalDistribution(string object);
	double probabilityOfConfig(int *roomTypeVal, int *objectInRoomVal,
			int *objectInObjectVal, int *objectOnObjectVal,
			int *objectDirectlyOnObjectVal);

	ptz::PTZInterfacePrx m_ptzInterface;
	std::string m_ptzServerComponent;

	DensitySampler m_sampler;

	// Object relation hierarchy items
	vector<string> hierarchyObjects; //Ordered list of objects regarded. Lower numbers
	//are landmarks for higher numbers.
	int nRooms;
	int nRoomCategories;
	int nObjects;
	vector<double> roomCategoryDefault; //index = roomID * nRoomCategories + catID
	vector<double> objectInRoomDefault; //index = objectID * nRoomCategories + catID
	vector<double> objectInObjectDefault; //index = trajectorID * nObjects + landmarkID
	vector<double> objectOnObjectDefault; //index = trajectorID * nObjects + landmarkID
};

std::vector<cogx::Math::Vector3>
findPolygonIntersection(const std::vector<cogx::Math::Vector3> &polygon1,
		const std::vector<cogx::Math::Vector3> &polygon2);

double getPolygonArea(const std::vector<cogx::Math::Vector3> &polygon);

}
; // namespace spatial

#endif // ObjectRelationManager_hpp

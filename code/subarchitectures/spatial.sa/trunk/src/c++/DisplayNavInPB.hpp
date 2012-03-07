//
// = FILENAME
//   DisplaySpatialInPB.hpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt, Andrzej Pronobis
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt, Andrzej Pronobis
//
/*----------------------------------------------------------------------*/

#ifndef DisplaySpatialInPB_hpp
#define DisplaySpatialInPB_hpp

#include "PTZ.hpp"
#include "NavData.hpp"
#include "Scan2dReceiver.hpp"
#include "PointCloudClient.h"
#include "SpatialProbabilities.hpp"
#include "SpatialProperties.hpp"
#include "ConceptualData.hpp"

#include <peekabot.hh>
#include <peekabot/Types.hh>
#include <cast/architecture/ManagedComponent.hpp>
#include <SensorData/SensorPose.hh>
#include <Transformation/Transformation3D.hh>
#include <string>

namespace spatial
{

/**
 * Command line options:
 *
 * @param -c a CURE config file to optionally read the
 * PEEKABOT_HOST and PEEKABOT_ROBOTXMLFILE from. The first are to say where
 * the peekabot server is running and the second gives the name of the
 * xml file with the robot definition.
 * @param --retry-interval  How many secs to wait to try to connect again to the peekabot server (default 10s). Negative value mean no retries
 * @param --laser-server-name the ice server name for the laser (default LaserServer)
 * @param --fov-hor horizontal field of view for the camera [deg] (default 45 degs)
 * @param --fov-vert vertical field of view for the camera [deg] (default 35 deg)
 * @param --read-ptu        Read and display ptu angles
 * @param --no-robot        Do not display robot
 * @param --no-walls        Do not display walls
 * @param --no-graph        Do not display the graph
 * @param --no-people       Do not display people
 * @param --no-scans        Do not display scans
 * @param --no-objects      Do not display objects 
 * @param --no-robotviewcone Do not display the robot viewcone
 * @param --people-id       Display the id above the head of each person
 * @param --non-unique      More than one objects of each kind accepted
 * @param --log-path	    Output the robot's movements over time
 *
 * @author Patric Jensfelt
 */


class DisplayNavInPB : public cast::ManagedComponent, public cast::PointCloudClient
{

public:
	DisplayNavInPB();
	virtual ~DisplayNavInPB();
	virtual void runComponent();
	virtual void start();

 
protected:
	virtual void configure(const std::map<std::string, std::string>& _config);
	void debugScanPush(const char* msg) {} //debug(msg); }
	void logScanPush(const char* msg) {} // log(msg); }

private:

	struct PersonData
	{
		std::string m_WMid;
		NavData::PersonPtr m_data;
		bool m_Followed;
	};

	struct Node
	{
		long m_Id;
		bool m_Gateway;
		long m_areaId;
		double m_X;
		double m_Y;
		long m_AreaClassNo;
	};

	struct PlaceData
	{
		int placeId;
		SpatialData::PlaceStatus placeStatus;
		int nodeId;
	};


	void receiveScan2d(const Laser::Scan2d &scan);
	void newPointCloud(const cast::cdl::WorkingMemoryChange &objID);
	void newComaRoom(const cast::cdl::WorkingMemoryChange &objID);

	void newViewpointGenCommand(const cast::cdl::WorkingMemoryChange &objID);
	void newARTagCommand(const cast::cdl::WorkingMemoryChange &objID);
	void newRecognizerCommand(const cast::cdl::WorkingMemoryChange &objID);
	void logDetectionCommand(const std::string &label);

	void newShapeProperty(const cast::cdl::WorkingMemoryChange &objID);
	void newSizeProperty(const cast::cdl::WorkingMemoryChange &objID);
	void newAppearanceProperty(const cast::cdl::WorkingMemoryChange &objID);
	void newRoomCategoryPlaceholderProperty(const cast::cdl::WorkingMemoryChange &objID);

	void newVPlist(const cast::cdl::WorkingMemoryChange &objID);
	void newRobotPose(const cast::cdl::WorkingMemoryChange &objID);
	void newNavCommand(const cast::cdl::WorkingMemoryChange & objID);
	void newNavGraphNode(const cast::cdl::WorkingMemoryChange &objID);

	void movePlace(const cast::cdl::WorkingMemoryChange &objID);
	void newPlace(const cast::cdl::WorkingMemoryChange &objID);

	void deletePlace(const cast::cdl::WorkingMemoryChange &objID);
	void newPlanePointCloud(const cast::cdl::WorkingMemoryChange &objID);
	void newNavGraphEdge(const cast::cdl::WorkingMemoryChange &objID);
	void newNavGraphObject(const cast::cdl::WorkingMemoryChange &objID);
	void newLineMap(const cast::cdl::WorkingMemoryChange &objID);
	void newPerson(const cast::cdl::WorkingMemoryChange &objID);
	void deletePerson(const cast::cdl::WorkingMemoryChange &objID);
	void newPersonFollowed(const cast::cdl::WorkingMemoryChange &objID);

	void addEdgeToList(long id1, long id2);
	void displayEdge(const DisplayNavInPB::Node &node1, const DisplayNavInPB::Node &node2);
	void redisplayEdgesToNode(const DisplayNavInPB::Node &node);
	void addDoorpost(double x, double y, double theta, double width, peekabot::SphereProxy &node);
	void connectPeekabot();
	void createFOV(peekabot::GroupProxy &proxy,const char* path, double fovHorizAngle,
			double fovVertiAngle, double* color, double opacity,
			NavData::ViewPoint viewpoint,  bool robotfov = true);
	void createRobotFOV();
	void getColorByIndex(int id, float &r, float &g, float &b);
	double getProbabilityValue(const SpatialProbabilities::ProbabilityDistribution &pd,
			std::string varValue);
	double getProbabilityValue(const SpatialProperties::ProbabilityDistributionPtr pd,
			int varValue);
	int GetPlaceIdFromNodeId(int nodeId);
	void addProperties(peekabot::SphereProxy &sp, int placeId);
	void addRoomCategoryPlaceholderProperties(peekabot::CubeProxy &sp, int placeId);
	void displayPeople();
	Cure::Transformation3D getCameraToWorldTransform();

private:
	peekabot::PeekabotClient m_PeekabotClient;
	peekabot::GroupProxy m_ProxyLabels;
	peekabot::GroupProxy m_ProxyRobot;
	peekabot::GroupProxy m_ProxyTrajectory;

	peekabot::PointCloudProxy m_ProxyKinect;
	peekabot::GroupProxy m_ProxyGraph;
	peekabot::GroupProxy m_ProxyNodes;
	peekabot::GroupProxy m_ProxyEdges;
	peekabot::GroupProxy m_ProxyObjects;
	peekabot::GroupProxy m_ProxyObjectLabels;
	peekabot::GroupProxy m_ProxyViewPoints;
	peekabot::PolylineProxy m_ProxyPathLog;
	peekabot::PolygonProxy m_ProxyPathStartMarker;
	peekabot::PolygonProxy m_ProxyPathEndMarker;
	peekabot::GroupProxy m_ProxyViewpointGenCommands;
	peekabot::GroupProxy m_ProxyDetectionCommands;

	peekabot::ObjectProxy m_ProxyCam;
	peekabot::HingeProxy m_ProxyPan;
	peekabot::HingeProxy m_ProxyTilt;

	bool m_FollowRobot;
	bool m_ShowLabels;
	bool m_ShowWalls;
	bool m_ShowGraph;
	bool m_ShowPeople;
	bool m_ShowPeopleId;
	bool m_ShowRobot;
	bool m_ShowScans;
	bool m_ShowObjects;
	bool m_ShowRoomId;
	bool m_ShowProperties;
	bool m_ShowPlaceholders;
	bool m_ShowRoomCategory;
	bool m_ShowRobotViewCone;
	bool m_ShowPlanePoints;
	bool m_ShowSOIs;
	bool m_ShowPath;
	bool m_ShowCommands;
	bool m_NonUniqueObjects;
  bool m_ShowPointCloud;
  bool m_ShowViewCones;

  int m_RetryDelay; // Seconds to retry if cannot connect. -1 means dont retry
	int m_currGoalPlace;

	std::map<std::string, std::string> m_peekabotObjectFiles;

	double m_FovH; // horisontal fov in degs
	double m_FovV; // vertical fov in degs
	std::vector<double> previouscenter;
	std::string m_LaserServerName;
	IceUtil::Mutex m_Mutex;
	NavData::RobotPose2dPtr m_RobotPose;
	NavData::LineMapPtr m_LineMap;
	Laser::Scan2d m_Scan;
	IceUtil::Mutex m_scanMutex;
	std::string m_PbHost;
	int m_PbPort;
	std::string m_PbRobotFile;
	std::string m_PbRobotName;
	std::string m_PbPersonFile;
	bool m_NoPeopleModel;
	int m_CurrPersonId;
	double m_lastLoggedX;
	double m_lastLoggedY;
	std::string m_currentMostLikelyRoom;


	std::vector<PersonData> m_People; // The people that are currently in view
	std::map<long, Node> m_Nodes;
	std::list< std::pair<long,long> > m_Edges;
	std::list< std::pair<long,long> > m_NewEdges;
	std::map<std::string, PlaceData> _places;

	bool m_LaserConnected;
	bool m_ReadPTU;
	ptz::PTZInterfacePrx m_PTUServer;
	Cure::SensorPose m_CameraPoseR;

	/** List of categories of rooms that we know about. */
	std::vector<std::string> _roomCategories;
	std::map<int, SpatialProperties::RoomShapePlacePropertyPtr> _shapeProps;
	std::map<int, SpatialProperties::RoomSizePlacePropertyPtr> _sizeProps;
	std::map<int, SpatialProperties::RoomAppearancePlacePropertyPtr> _appearanceProps;
	std::map<std::string, SpatialProperties::RoomCategoryPlaceholderPropertyPtr> _roomCatPlaceholderProps;

};

}; // namespace spatial

#endif // DisplaySpatialInPB_hpp

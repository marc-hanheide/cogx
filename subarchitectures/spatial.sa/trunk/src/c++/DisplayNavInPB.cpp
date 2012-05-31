//
// = FILENAME
//    DisplayNavInPB.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//	  Alper Aydemir
//	  Alexey Bezugly
//
// = COPYRIGHT
//    Copyright (c) 2008 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#define wrn(args...); {char buf[512]; sprintf(buf,args); getLogger()->warn(buf);}

#include "DisplayNavInPB.hpp"

#include "NavData.hpp"
#include "Laser.hpp"
#include "CureHWUtils.hpp"
#include "VisionData.hpp"
#include "SpatialData.hpp"
#include "SpatialProperties.hpp"
#include "FrontierInterface.hpp"
#include "ComaData.hpp"

#include <cast/architecture/ChangeFilterFactory.hpp>
#include <FrontierInterface.hpp>
#include <Navigation/NavGraph.hh>
#include <Transformation/Pose3D.hh>
#include <AddressBank/ConfigFileReader.hh>
#include <Utils/CureDebug.hh>
#include <Navigation/NavGraphNode.hh>
#include <Navigation/NavGraphEdge.hh>
#include <Navigation/NavGraphGateway.hh>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <list>
#include <string>
#include <cfloat>
#include <iostream>

using namespace std;
using namespace cast;
using namespace boost;
using namespace spatial;

extern "C" {
cast::interfaces::CASTComponentPtr newComponent() {
  return new DisplayNavInPB();
}
}

DisplayNavInPB::DisplayNavInPB() {
  m_LaserConnected = false;
  m_NoPeopleModel = false;
  m_LaserServerName = "laser.server";
  m_RobotPose = 0;
  m_LineMap = 0;
  m_FovH = 45.0;
  m_FovV = 35.0;
  previouscenter.assign(3, 0.0);
  m_lastLoggedX = DBL_MAX;
  m_lastLoggedY = DBL_MAX;
  m_currentMostLikelyRoom = "";
  m_currGoalPlace = -1;
}

DisplayNavInPB::~DisplayNavInPB() {
}

void DisplayNavInPB::configure(const map<string, string>& _config) {
  log("configure entered");

  try {
    configureServerCommunication(_config);
    m_ShowPointCloud = true;
  } catch (...) {
    m_ShowPointCloud = false;
  }

  m_LoadRobotFile = (_config.find("--no-load-robot-file") == _config.end());
  m_ShowPanTilt = m_LoadRobotFile; // don't show pan tilt if not loading robot file

  m_FollowRobot = (_config.find("--follow-robot") != _config.end());
  m_ShowRobot = (_config.find("--no-robot") == _config.end());
  m_ShowWalls = (_config.find("--no-walls") == _config.end());
  m_ShowGraph = (_config.find("--no-graph") == _config.end());
  m_ShowLabels = (_config.find("--no-labels") == _config.end());
  m_ShowPeople = (_config.find("--no-people") == _config.end());
  m_ShowScans = (_config.find("--no-scans") == _config.end());
  m_ShowObjects = (_config.find("--no-objects") == _config.end());
  m_ShowRobotViewCone = (_config.find("--no-robotviewcone") == _config.end());
  m_ShowPlanePoints = (_config.find("--no-plane-points") == _config.end());
  m_ShowSOIs = (_config.find("--no-sois") == _config.end());
  m_ShowPeopleId = (_config.find("--people-id") != _config.end());
  m_NonUniqueObjects = (_config.find("--non-unique") != _config.end());
  m_ReadPTU = (_config.find("--read-ptu") != _config.end());
  m_ShowProperties = (_config.find("--no-properties") == _config.end());
  m_ShowPlaceholders = (_config.find("--no-placeholders") == _config.end());
  m_ShowRoomId = (_config.find("--no-roomid") == _config.end());
  m_ShowRoomCategory = (_config.find("--no-areaclass") == _config.end());
  m_ShowConnectivities = (_config.find("--show-connectivities")
      != _config.end());

  m_ShowPath = (_config.find("--log-path") != _config.end());
  m_ShowCommands = (_config.find("--log-commands") != _config.end());

  if (_config.find("--laser-server") != _config.end()) {
    std::istringstream str(_config.find("--laser-server")->second);
    str >> m_LaserServerName;
  }

  if (_config.find("--object-models") != _config.end()) {
    std::istringstream str(_config.find("--object-models")->second);

    while (!str.eof()) {
      std::string objCategory;
      std::string objFilename;

      str >> objCategory >> objFilename;
      if (objCategory == "" || objFilename == "") {
        continue;
      }
      m_peekabotObjectFiles[objCategory] = objFilename;
    }
  }

  if (_config.find("--fov-hor") != _config.end()) {
    std::istringstream str(_config.find("--fov-hor")->second);
    str >> m_FovH;
  }
  if (_config.find("--fov-vert") != _config.end()) {
    std::istringstream str(_config.find("--fov-vert")->second);
    str >> m_FovV;
  }

  if (_config.find("--room-categories") != _config.end()) {
    string rcStr = _config.find("--room-categories")->second;
    split(_roomCategories, rcStr, is_any_of(","));
  }

  m_RetryDelay = 1000;
  if (_config.find("--retry-interval") != _config.end()) {
    std::istringstream str(_config.find("--retry-interval")->second);
    str >> m_RetryDelay;
  }

  Cure::ConfigFileReader *cfg = 0;

  map<string, string>::const_iterator confIter = _config.find("-c");
  if (confIter != _config.end()) {
    cfg = new Cure::ConfigFileReader;
    log("About to try to open the config file");
    if (cfg->init(confIter->second) != 0) {
      delete cfg;
      cfg = 0;
      log("Could not init Cure::ConfigFileReader with -c argument");
    } else {
      log("Managed to open the Cure config file");
    }
  }

  m_PbPort = 5050;
  m_PbHost = "localhost";
  m_PbRobotName = "robot";
  m_PbRobotFile = "CogXp3.xml";
  m_PbPersonFile = "rolf.pbmf";

  if (cfg) {

    //cfg->getRobotName(m_PbRobotName);

    // To be backward compatible with config files that specify the
    // RoboLook host and really mean peekabot we read that first and
    // overwrite it below if both are specified
    cfg->getRoboLookHost(m_PbHost);

    std::string usedCfgFile, tmp;
    if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
      m_PbHost = tmp;
    }
    if (cfg->getString("PEEKABOT_ROBOT_XML_FILE", true, tmp, usedCfgFile) == 0) {
      m_PbRobotFile = tmp;
    }
    if (cfg->getString("PEEKABOT_PERSON_PBMF_FILE", true, tmp, usedCfgFile)
        == 0) {
      m_PbPersonFile = tmp;
    }

    if (cfg->getSensorPose(2, m_CameraPoseR)) {
      println(
          "configure(...) Failed to get sensor pose for camera. (Run with --no-planes to skip)");
      std::abort();
    }
  }

  log("Using %s as the robotfile in peekabot", m_PbRobotFile.c_str());
  log("Using %s as the person in peekabot", m_PbPersonFile.c_str());

  connectPeekabot();

  if (m_ReadPTU) {
    Ice::CommunicatorPtr ic = getCommunicator();

    Ice::Identity id;
    id.name = "PTZServer";
    id.category = "PTZServer";

    std::ostringstream str;
    str << ic->identityToString(id) << ":default" << " -h localhost" << " -p "
        << cast::cdl::CPPSERVERPORT;

    Ice::ObjectPrx base = ic->stringToProxy(str.str());
    m_PTUServer = ptz::PTZInterfacePrx::uncheckedCast(base);
  }

  log("configure done");
}

void DisplayNavInPB::start() {

  if (m_ShowPointCloud) {
    startPCCServerCommunication(*this);
  }

  // Robot pose
  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newRobotPose));

  addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newRobotPose));

  addChangeFilter(createLocalTypeFilter<SpatialData::NavCommand> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newNavCommand));

  // NavData
  addChangeFilter(createLocalTypeFilter<NavData::FNode> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newNavGraphNode));
  addChangeFilter(createLocalTypeFilter<NavData::FNode> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newNavGraphNode));

  addChangeFilter(createLocalTypeFilter<NavData::ObjData> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newNavGraphObject));
  addChangeFilter(createLocalTypeFilter<NavData::ObjData> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newNavGraphObject));

  addChangeFilter(createLocalTypeFilter<NavData::AEdge> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newNavGraphEdge));
  addChangeFilter(createLocalTypeFilter<NavData::AEdge> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newNavGraphEdge));

  // SpatialProperties::ConnectivityPathProperty
  addChangeFilter(createLocalTypeFilter<
      SpatialProperties::ConnectivityPathProperty> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newConnectivityPathProperty));
  addChangeFilter(createLocalTypeFilter<
      SpatialProperties::ConnectivityPathProperty> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newConnectivityPathProperty));
  addChangeFilter(createLocalTypeFilter<
      SpatialProperties::ConnectivityPathProperty> (cdl::DELETE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::deleteConnectivityPathProperty));

  // Places
  addChangeFilter(createLocalTypeFilter<SpatialData::Place> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newPlace));
  addChangeFilter(createLocalTypeFilter<SpatialData::Place> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newPlace));
  addChangeFilter(createLocalTypeFilter<SpatialData::Place> (cdl::DELETE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::deletePlace));

  // MapLoadStatus
  addChangeFilter(createLocalTypeFilter<SpatialData::MapLoadStatus> (
      cdl::WILDCARD), new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
      &DisplayNavInPB::mapLoadStatusChanged));

  // NodeHyhothesis
  addChangeFilter(createLocalTypeFilter<SpatialData::NodeHypothesis> (
      cdl::OVERWRITE), new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
      &DisplayNavInPB::movePlace));

  // People
  addChangeFilter(createLocalTypeFilter<NavData::Person> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newPerson));

  addChangeFilter(createLocalTypeFilter<NavData::Person> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newPerson));

  addChangeFilter(createLocalTypeFilter<NavData::Person> (cdl::DELETE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::deletePerson));

  addChangeFilter(createLocalTypeFilter<NavData::PersonFollowed> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newPersonFollowed));
  addChangeFilter(createLocalTypeFilter<NavData::PersonFollowed> (
      cdl::OVERWRITE), new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
      &DisplayNavInPB::newPersonFollowed));

  // LineMap
  addChangeFilter(createLocalTypeFilter<NavData::LineMap> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newLineMap));
  addChangeFilter(createLocalTypeFilter<NavData::LineMap> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newLineMap));

  // Place & Placeholder Properties
  if (m_ShowProperties) {
    addChangeFilter(createGlobalTypeFilter<
        SpatialProperties::RoomShapePlaceProperty> (cdl::ADD),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newShapeProperty));

    addChangeFilter(createGlobalTypeFilter<
        SpatialProperties::RoomShapePlaceProperty> (cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newShapeProperty));

    addChangeFilter(createGlobalTypeFilter<
        SpatialProperties::RoomSizePlaceProperty> (cdl::ADD),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newSizeProperty));

    addChangeFilter(createGlobalTypeFilter<
        SpatialProperties::RoomSizePlaceProperty> (cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newSizeProperty));

    addChangeFilter(createGlobalTypeFilter<
        SpatialProperties::RoomAppearancePlaceProperty> (cdl::ADD),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newAppearanceProperty));

    addChangeFilter(createGlobalTypeFilter<
        SpatialProperties::RoomAppearancePlaceProperty> (cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newAppearanceProperty));

    if (m_ShowPlaceholders) {
      addChangeFilter(createGlobalTypeFilter<
          SpatialProperties::RoomCategoryPlaceholderProperty> (cdl::ADD),
          new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
              &DisplayNavInPB::newRoomCategoryPlaceholderProperty));

      addChangeFilter(createGlobalTypeFilter<
          SpatialProperties::RoomCategoryPlaceholderProperty> (cdl::OVERWRITE),
          new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
              &DisplayNavInPB::newRoomCategoryPlaceholderProperty));
    }
  }

  // SOIs
  if (m_ShowSOIs) {
    addChangeFilter(createGlobalTypeFilter<VisionData::SOI> (cdl::ADD),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newPointCloud));

    addChangeFilter(createGlobalTypeFilter<VisionData::SOI> (cdl::OVERWRITE),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newPointCloud));
  }

  // Plane points
  if (m_ShowPlanePoints) {
    addChangeFilter(createLocalTypeFilter<SpatialData::PlanePoints> (cdl::ADD),
        new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
            &DisplayNavInPB::newPlanePointCloud));

    addChangeFilter(createLocalTypeFilter<SpatialData::PlanePoints> (
        cdl::OVERWRITE), new MemberFunctionChangeReceiver<DisplayNavInPB> (
        this, &DisplayNavInPB::newPlanePointCloud));
  }

  // Coma Rooms
  addChangeFilter(createGlobalTypeFilter<comadata::ComaRoom> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newComaRoom));
  addChangeFilter(createGlobalTypeFilter<comadata::ComaRoom> (cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newComaRoom));

  // Logging events

  addChangeFilter(createGlobalTypeFilter<
      SpatialData::RelationalViewPointGenerationCommand> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newViewpointGenCommand));
  addChangeFilter(createGlobalTypeFilter<VisionData::ARTagCommand> (cdl::ADD),
      new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
          &DisplayNavInPB::newARTagCommand));
  addChangeFilter(createGlobalTypeFilter<VisionData::Recognizer3DCommand> (
      cdl::ADD), new MemberFunctionChangeReceiver<DisplayNavInPB> (this,
      &DisplayNavInPB::newRecognizerCommand));
  log("start done");
}

void DisplayNavInPB::newPlanePointCloud(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("Entered newPlacePointCloud");
  log("new PlanePointCloud received.");

  try {

    SpatialData::PlanePointsPtr objData = getMemoryEntry<
        SpatialData::PlanePoints> (objID.address);

    double color[3] = { 0.9, 0, 0 };

    numeric::ublas::matrix<double> m(3, 3);
    m(0, 0) = 0;
    m(0, 1) = 1;
    m(0, 2) = 0;
    m(1, 0) = 0;
    m(1, 1) = 0;
    m(1, 2) = -1;
    m(2, 0) = 1;
    m(2, 1) = 0;
    m(2, 2) = 0;

    peekabot::PointCloudProxy pcloud;
    char tmp[256];
    sprintf(tmp, "planepoints%s", objID.address.id.c_str());
    pcloud.add(m_PeekabotClient, tmp, peekabot::REPLACE_ON_CONFLICT);

    numeric::ublas::vector<double> v(3);
    numeric::ublas::vector<double> t(3);
    //add plane points
    log("points size: %d", objData->points.size());
    peekabot::VertexSet vs;
    for (unsigned int i = 0; i < objData->points.size(); i++) {
      v(0) = objData->points.at(i).x;
      v(1) = objData->points.at(i).y;
      v(2) = objData->points.at(i).z;
      //   t = prod(v,m);
      vs.add(v(0), v(1), v(2));
    }
    pcloud.add_vertices(vs);
    pcloud.set_color(color[0], color[1], color[2]);
  } catch (DoesNotExistOnWMException) {
    log("Error! plane point cloud disappeared from WM.");
  }

  debug("Exited newPlacePointCloud");
}

void DisplayNavInPB::newShapeProperty(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("Entered newShapeProperty");

  // Get the property
  SpatialProperties::RoomShapePlacePropertyPtr property;
  try {
    property = getMemoryEntry<SpatialProperties::RoomShapePlaceProperty> (
        objID.address);
  } catch (...) {
    log("Error! property disappeared from WM.");
    debug("Exited newShapeProperty");
    return;
  }

  ::FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));

  // Get node it for the place ID
  ::NavData::FNodePtr fnodePtr = agg->getNodeFromPlaceID((::Ice::Int)(
      property->placeId)); // why is iter a long?

  if (!fnodePtr) {
    error("Place with id %d for added shape property does not exist",
        property->placeId);
    return;
  }

  IceUtil::Mutex::Lock lock(m_Mutex);

  _shapeProps[property->placeId] = property;

  // search m_Nodes for place and change its room id
  std::map<long, Node>::iterator nodeIter = m_Nodes.find(fnodePtr->nodeId);

  if (nodeIter == m_Nodes.end()) {
    println(
        "error: coma room contains a node I do not know about: placeID = %d",
        property->placeId);

    debug("Exited newShapeProperty");
    return;
  }

  DisplayNavInPB::Node node = nodeIter->second;

  // Get the node proxy
  peekabot::SphereProxy sp;
  char name1[32];
  sprintf(name1, "node%ld", (long) node.m_Id);
  sp.assign(m_ProxyNodes, name1);

  addProperties(sp, property->placeId);

  debug("Exited newShapeProperty");
}

void DisplayNavInPB::newSizeProperty(
    const cast::cdl::WorkingMemoryChange &objID) {

  log("Entered newSizeProperty");

  // Get the property
  SpatialProperties::RoomSizePlacePropertyPtr property;
  try {
    property = getMemoryEntry<SpatialProperties::RoomSizePlaceProperty> (
        objID.address);
  } catch (...) {
    debug("Error! property disappeared from WM.");
    debug("Exited newSizeProperty");
    return;
  }

  ::FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));

  // Get node it for the place ID
  ::NavData::FNodePtr fnodePtr = agg->getNodeFromPlaceID((::Ice::Int)(
      property->placeId)); // why is iter a long?

  if (!fnodePtr) {
    error("Place with id %d for added size property does not exist",
        property->placeId);
    return;
  }

  IceUtil::Mutex::Lock lock(m_Mutex);

  _sizeProps[property->placeId] = property;

  // search m_Nodes for place and change its room id
  std::map<long, Node>::iterator nodeIter = m_Nodes.find(fnodePtr->nodeId);

  if (nodeIter == m_Nodes.end()) {
    println(
        "error: coma room contains a node I do not know about: placeID = %d",
        property->placeId);

    debug("Exited newSizeProperty");
    return;
  }

  DisplayNavInPB::Node node = nodeIter->second;

  // Get the node proxy
  peekabot::SphereProxy sp;
  char name1[32];
  sprintf(name1, "node%ld", (long) node.m_Id);
  sp.assign(m_ProxyNodes, name1);

  addProperties(sp, property->placeId);

  debug("Exited newSizeProperty");
}

void DisplayNavInPB::newAppearanceProperty(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("Entered newAppearanceProperty");
  // Get the property
  SpatialProperties::RoomAppearancePlacePropertyPtr property;
  try {
    property = getMemoryEntry<SpatialProperties::RoomAppearancePlaceProperty> (
        objID.address);
  } catch (...) {
    log("Error! property disappeared from WM.");
    debug("Exited newAppearanceProperty");
    return;
  }

  ::FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));

  // Get node it for the place ID
  ::NavData::FNodePtr fnodePtr = agg->getNodeFromPlaceID((::Ice::Int)(
      property->placeId)); // why is iter a long?

  if (!fnodePtr) {
    error("Place with id %d for added appearance property does not exist",
        property->placeId);
    return;
  }

  IceUtil::Mutex::Lock lock(m_Mutex);

  _appearanceProps[property->placeId] = property;

  // search m_Nodes for place and change its room id
  std::map<long, Node>::iterator nodeIter = m_Nodes.find(fnodePtr->nodeId);

  if (nodeIter == m_Nodes.end()) {
    println(
        "error: coma room contains a node I do not know about: placeID = %d",
        property->placeId);

    debug("Exited newAppearanceProperty");
    return;
  }

  DisplayNavInPB::Node node = nodeIter->second;

  // Get the node proxy
  peekabot::SphereProxy sp;
  char name1[32];
  sprintf(name1, "node%ld", (long) node.m_Id);
  sp.assign(m_ProxyNodes, name1);

  addProperties(sp, property->placeId);

  debug("Exited newAppearanceProperty");
}

void DisplayNavInPB::newComaRoom(const cast::cdl::WorkingMemoryChange &objID) {
  log("Entered newComaRoom (%s)", objID.address.id.c_str());
  if (!m_PeekabotClient.is_connected()) {
    log("Exited newComaRoom");
    return;
  }

  // Get the ComaRoom
  comadata::ComaRoomPtr croom;
  try {
    croom = getMemoryEntry<comadata::ComaRoom> (objID.address);
  } catch (...) {
    log("Error! coma room cloud disappeared from WM.");
    log("Exited newComaRoom");
    return;
  }

  // Initialization
  ::Ice::Int roomId = croom->roomId;
  ::FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));
  debug("New Coma Room recieved: id=%d", roomId);

  map<long, long> nodeIDs;
  // For each place in the room
  for (std::vector<Ice::Long>::iterator iter = croom->containedPlaceIds.begin(); iter
      != croom->containedPlaceIds.end(); iter++) {
    // Get node id from place id
    ::NavData::FNodePtr fnodePtr = agg->getNodeFromPlaceID((::Ice::Int) * iter); // why is iter a long?
    if (fnodePtr) {
      nodeIDs[*iter] = fnodePtr->nodeId;
    } else {
      log("comaroom contained unknown node");
    }
  }

  // Update DisplayNavInPB
  IceUtil::Mutex::Lock lock(m_Mutex);

  // For each place in the room
  for (std::vector<Ice::Long>::iterator iter = croom->containedPlaceIds.begin(); iter
      != croom->containedPlaceIds.end(); iter++) {

    // Search m_Nodes for place and change its room id
    std::map<long, Node>::iterator nodeIter = m_Nodes.find(nodeIDs[*iter]);

    if (nodeIter == m_Nodes.end()) {
      log(
          "error: coma room %d contains a node I do not know about: placeID = %d",
          roomId, *iter);
      continue;
    }

    DisplayNavInPB::Node node = nodeIter->second;
    node.m_areaId = roomId;

    // Is the node a gateway?
    if (node.m_Gateway) {
      // If there is a gateway node here it means the node has changed to
      // a door while we were processing this ComaRoom, so we can ignore it
      log(
          "Wasn't expecting a gateway node (id %d) here! (DisplayNavInPB::newComaRoom)",
          node.m_Id);
    } else {
      // Update Peekabot
      peekabot::SphereProxy sp;
      char name[32];
      sprintf(name, "node%ld", (long) node.m_Id);
      sp.add(m_ProxyNodes, name, peekabot::REPLACE_ON_CONFLICT);
      sp.set_position(node.m_X, node.m_Y, 0);
      float r, g, b;
      getColorByIndex(roomId, r, g, b);
      sp.set_scale(0.1, 0.1, 0.05);
      debug("Added coma place with id %d", node.m_Id);
      sp.set_color(r, g, b);

      if (m_ShowProperties) {
        addProperties(sp, *iter);
      }

      if (m_ShowRoomId) {
        peekabot::CylinderProxy cp;
        cp.add(sp, "class", peekabot::REPLACE_ON_CONFLICT);
        cp.set_scale(0.04, 0.04, 0.16);
        cp.set_position(0, 0, 0.08);
        getColorByIndex(roomId, r, g, b);
        cp.set_color(r, g, b);

        peekabot::SphereProxy mp;
        mp.add(sp, "mushroom");
        mp.set_scale(0.08, 0.08, 0.05);
        mp.set_position(0, 0, 0.16);
        mp.set_color(r, g, b);
      }

      if (m_ShowRoomCategory) {
        // Pie chart of category probabilities
        double probSum = 0.0;
        double startAngle = 0.0;
        unsigned int index = 0;
        for (; index < _roomCategories.size(); ++index) {
          double prob = getProbabilityValue(croom->categories,
              _roomCategories[index]);
          getColorByIndex(index, r, g, b);
          char name2[32];
          if (index == 0)
            sprintf(name2, "node%ld.area_class", node.m_Id);
          else
            sprintf(name2, "node%ld.area_class%d", node.m_Id, index);

          // Calculations
          probSum += prob;
          double endAngle = (2 * 3.1415926) * probSum;
          // Add a polygon that corresponds to part of the pie chart
          // between startAngle and endAngle;
          peekabot::PolygonProxy acp;
          peekabot::VertexSet vs;
          acp.add(sp, name2, peekabot::REPLACE_ON_CONFLICT);
          vs.add(0, 0, 0);
          for (double tmp = startAngle; tmp < endAngle; tmp += (2.0 * 3.1415926
              / 36))
            vs.add(0.5 * cosf(tmp), 0.5 * sinf(tmp), 0);
          vs.add(0.5 * cosf(endAngle), 0.5 * sinf(endAngle), 0);
          acp.add_vertices(vs);
          //acp.set_opacity(0.7);
          acp.set_color(r, g, b);
          startAngle = endAngle;
        }
        // Add one for the rest
        char name2[32];
        if (index == 0)
          sprintf(name2, "node%ld.area_class", node.m_Id);
        else
          sprintf(name2, "node%ld.area_class%d", node.m_Id, index);
        double endAngle = (2 * 3.1415926);
        peekabot::PolygonProxy acp;
        peekabot::VertexSet vs;
        acp.add(sp, name2, peekabot::REPLACE_ON_CONFLICT);
        vs.add(0, 0, 0);
        for (double tmp = startAngle; tmp < endAngle; tmp += (2.0 * 3.1415926
            / 36))
          vs.add(0.5 * cosf(tmp), 0.5 * sinf(tmp), 0);
        vs.add(0.5 * cosf(endAngle), 0.5 * sinf(endAngle), 0);
        acp.add_vertices(vs);
        //acp.set_opacity(0.7);
        acp.set_color(0.7, 0.7, 0.7);
        startAngle = endAngle;
      }

      // We also redraw the edge just to make sure that it is drawn
      // correctly if the position of the node changed.
      redisplayEdgesToNode(node);
    }
  } // end for
  log("Exited newComaRoom");
}

void DisplayNavInPB::newRoomCategoryPlaceholderProperty(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("Entered newRoomCategoryPlaceholderProperty");

  // Get the property
  SpatialProperties::RoomCategoryPlaceholderPropertyPtr property;
  try {
    property = getMemoryEntry<
        SpatialProperties::RoomCategoryPlaceholderProperty> (objID.address);
  } catch (...) {
    log("Error! property disappeared from WM.");
    debug("Exited newRoomCategoryPlaceholderProperty");
    return;
  }

  ::FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));
  // Get node hyp for the placeholder ID
  SpatialData::NodeHypothesisPtr nodeHypPtr = agg->getHypFromPlaceID(
      (::Ice::Int)(property->placeId));

  IceUtil::Mutex::Lock lock(m_Mutex);

  // Remember the property
  stringstream ss;
  ss << property->placeId << "_";
  ss << property->category;
  _roomCatPlaceholderProps[ss.str()] = property;

  if (nodeHypPtr) {
    // Get the node proxy
    peekabot::CubeProxy sp;
    char name[32];
    sprintf(name, "node_hyp%ld", (long) nodeHypPtr->hypID);
    sp.assign(m_ProxyNodes, name);

    addRoomCategoryPlaceholderProperties(sp, property->placeId);
  }

  log("Exited newRoomCategoryPlaceholderProperty");
}

void DisplayNavInPB::addProperties(peekabot::SphereProxy &sp, int placeId) {
  if (_sizeProps.find(placeId) != _sizeProps.end()) {
    string map;
    if (_sizeProps[placeId]->mapValue) {
      SpatialProperties::StringValuePtr ptr =
          SpatialProperties::StringValuePtr::dynamicCast(
              _sizeProps[placeId]->mapValue);
      if (ptr) {
        map = ptr->value;
      }
    }

    // Add out property proxy
    peekabot::LabelProxy lp;
    lp.add(sp, "size_property", peekabot::REPLACE_ON_CONFLICT);
    lp.set_pose(0.5, 0.7, 0.3, 0, 0, 0);
    lp.set_scale(10, 10, 10);
    lp.set_text(map.c_str());
    lp.set_alignment(peekabot::ALIGN_CENTER);
    lp.set_color(0.3, 0.3, 0.3);
  }

  if (_shapeProps.find(placeId) != _shapeProps.end()) {
    string map;
    if (_shapeProps[placeId]->mapValue) {
      SpatialProperties::StringValuePtr ptr =
          SpatialProperties::StringValuePtr::dynamicCast(
              _shapeProps[placeId]->mapValue);
      if (ptr) {
        map = ptr->value;
      }
    }

    // Add out property proxy
    peekabot::LabelProxy lp;
    lp.add(sp, "shape_property", peekabot::REPLACE_ON_CONFLICT);
    lp.set_pose(0.5, 0.5, 0.3, 0, 0, 0);
    lp.set_scale(10, 10, 10);
    lp.set_text(map.c_str());
    lp.set_alignment(peekabot::ALIGN_CENTER);
    lp.set_color(0.3, 0.3, 0.3);
  }

  if (_appearanceProps.find(placeId) != _appearanceProps.end()) {
    string map;
    if (_appearanceProps[placeId]->mapValue) {
      SpatialProperties::StringValuePtr ptr =
          SpatialProperties::StringValuePtr::dynamicCast(
              _appearanceProps[placeId]->mapValue);
      if (ptr) {
        map = ptr->value;
      }
    }

    // Add out property proxy
    peekabot::LabelProxy lp;
    lp.add(sp, "appearance_property", peekabot::REPLACE_ON_CONFLICT);
    lp.set_pose(0.5, 0.3, 0.3, 0, 0, 0);
    lp.set_scale(10, 10, 10);
    lp.set_text(string(map + "-like").c_str());
    lp.set_alignment(peekabot::ALIGN_CENTER);
    lp.set_color(0.3, 0.3, 0.3);
  }

}

double DisplayNavInPB::getProbabilityValue(
    const SpatialProbabilities::ProbabilityDistribution &pd,
    std::string varValue) {
  for (unsigned int i = 0; i < pd.massFunction.size(); ++i) {
    SpatialProbabilities::JointProbabilityValue jpv = pd.massFunction[i];
    string v1 =
        SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
            jpv.variableValues[0])->value;
    if (v1 == varValue)
      return jpv.probability;
  }

  return 0;
}

double DisplayNavInPB::getProbabilityValue(
    const SpatialProperties::ProbabilityDistributionPtr pd, int varValue) {
  SpatialProperties::DiscreteProbabilityDistributionPtr dpd =
      SpatialProperties::DiscreteProbabilityDistributionPtr::dynamicCast(pd);

  for (unsigned int i = 0; i < dpd->data.size(); ++i) {
    SpatialProperties::ValueProbabilityPair vpp = dpd->data[i];
    int v1 = SpatialProperties::IntegerValuePtr::dynamicCast(vpp.value)->value;
    if (v1 == varValue)
      return vpp.probability;
  }

  return 0;
}

void DisplayNavInPB::createRobotFOV() {
  if (!m_ShowRobotViewCone)
    return;

  std::string path = "peoplebot_base/ptu.pan/tilt/stereo_cam/cam_right";

  double color[3];
  color[0] = 0.9;
  color[1] = 0.9;
  color[2] = 0.9;

  if (m_PbRobotFile == "Robone.xml") {
    path = "peoplebot_base/ptu/pan.tilt/stereo_cam";
  } else {
    path = "model/camera";
  }

  peekabot::GroupProxy cam;
  cam.assign(m_ProxyRobot, path);
  NavData::ViewPoint vp;
  vp.pos.z = 0;
  vp.pan = 0;
  vp.tilt = 0;
  vp.length = 1.0;
  createFOV(cam, "cam_right/cone", m_FovH, m_FovV, color, 0.3, vp);
}

void DisplayNavInPB::createFOV(peekabot::GroupProxy &proxy, const char* path,
    double fovHorizAngle, double fovVertiAngle, double* color, double opacity,
    NavData::ViewPoint viewpoint, bool robotfov) {

  peekabot::GroupProxy proxyCone;
  proxyCone.add(proxy, path, peekabot::AUTO_ENUMERATE_ON_CONFLICT);
  //proxyCone.hide();
  const double coneLen = viewpoint.length;
  // The "half angle" of the field of view
  const double fovHoriz = fovHorizAngle * M_PI / 180.0 / 2;
  const double fovVerti = fovVertiAngle * M_PI / 180.0 / 2;
  peekabot::PolygonProxy proxyConeParts[5];
  peekabot::VertexSet vs;
  proxyConeParts[0].add(proxyCone, "top");
  vs.add(0, 0, 0);
  vs.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(fovVerti));
  vs.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(fovVerti));
  proxyConeParts[0].add_vertices(vs);
  vs.clear();
  proxyConeParts[1].add(proxyCone, "bottom");
  vs.add(0, 0, 0);
  vs.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(-fovVerti));
  vs.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(-fovVerti));
  proxyConeParts[1].add_vertices(vs);
  vs.clear();
  proxyConeParts[2].add(proxyCone, "left");
  vs.add(0, 0, 0);
  vs.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(-fovVerti));
  vs.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(fovVerti));
  proxyConeParts[2].add_vertices(vs);
  vs.clear();
  proxyConeParts[3].add(proxyCone, "right");
  vs.add(0, 0, 0);
  vs.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(-fovVerti));
  vs.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(fovVerti));
  proxyConeParts[3].add_vertices(vs);
  vs.clear();
  proxyConeParts[4].add(proxyCone, "image");
  vs.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(fovVerti));
  vs.add(coneLen, coneLen * tan(fovHoriz), coneLen * tan(-fovVerti));
  vs.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(-fovVerti));
  vs.add(coneLen, coneLen * tan(-fovHoriz), coneLen * tan(fovVerti));
  proxyConeParts[4].add_vertices(vs);

  for (int i = 0; i < 5; i++) {
    proxyConeParts[i].set_color(color[0], color[1], color[2]);
    proxyConeParts[i].set_opacity(opacity);
    proxyConeParts[i].set_scale(1); // This is how I make the cone
    // larger or smaller

  }
  if (!robotfov) {

    //proxyCone.set_rotation(viewpoint.pan,viewpoint.tilt,0);
    proxyCone.rotate(viewpoint.pan, 0, 0, 1);
    proxyCone.rotate(viewpoint.tilt, 0, -1, 0);
    proxyCone.set_position(viewpoint.pos.x, viewpoint.pos.y, viewpoint.pos.z);

  } else {

    proxyCone.set_rotation(viewpoint.pan, viewpoint.tilt, 0);
    proxyCone.set_position(0, 0, viewpoint.pos.z);
  }
}

void DisplayNavInPB::newPointCloud(const cdl::WorkingMemoryChange &objID) {
  debug("Entered newPointCloud");
  log("Got new SOI points.");
  double color[3] = { 0.9, 0, 0 };

  try {
    VisionData::SOIPtr objData =
        getMemoryEntry<VisionData::SOI> (objID.address);

    Cure::Transformation3D cam2WorldTrans = getCameraToWorldTransform();
    double tmp[6];
    cam2WorldTrans.getCoordinates(tmp);
    log("total transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
        tmp[4], tmp[5]);

    //Convert hull to world coords
    for (unsigned int i = 0; i < objData->points.size(); i++) {
      Cure::Vector3D from(objData->points[i].p.x, objData->points[i].p.y,
          objData->points[i].p.z);
      Cure::Vector3D to;
      cam2WorldTrans.invTransform(from, to);
      //    log("vertex at %f, %f, %f", objData->points[i].p.x, objData->points[i].p.y, objData->points[i].p.z);
      objData->points[i].p.x = to.X[0];
      objData->points[i].p.y = to.X[1];
      objData->points[i].p.z = to.X[2];
      //    log("Transformed vertex at %f, %f, %f", to.X[0], to.X[1], to.X[2]);
    }

    //  numeric::ublas::matrix<double> m (3, 3);
    //  m(0,0) = 0; m(0,1) = 1; m(0,2) = 0;
    //  m(1,0) = 0; m(1,1) = 0; m(1,2) = -1;
    //  m(2,0) = 1; m(2,1) = 0; m(2,2) = 0;
    peekabot::PointCloudProxy pcloud;
    pcloud.add(m_PeekabotClient, "planepopout", peekabot::REPLACE_ON_CONFLICT);

    numeric::ublas::vector<double> v(3);
    //add plane points
    peekabot::VertexSet vs;
    for (unsigned int i = 0; i < objData->points.size(); i++) {

      v(0) = objData->points.at(i).p.x;
      v(1) = objData->points.at(i).p.y;
      v(2) = objData->points.at(i).p.z;
      vs.add(v(0), v(1), v(2));

    }
    pcloud.add_vertices(vs);
    pcloud.set_color(color[0], color[1], color[2]);
  } catch (DoesNotExistOnWMException) {
    log("Error! SOI WM entry went missing!");
  }

  debug("Exited newPointCloud");
}

void DisplayNavInPB::runComponent() {

  log("runComponent");

  while (!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)) {
    if (!isRunning())
      return;
    sleepComponent( m_RetryDelay);
    connectPeekabot();
  }

  log("Connected to peekabot, ready to go");
  if (m_PeekabotClient.is_connected()) {
    while (isRunning()) {
      log("while loop");

      ptz::PTZReading ptuPose;
      if (m_ReadPTU) {
        ptuPose = m_PTUServer->getPose();
        //log("Read ptu pose %f %f", ptuPose.pose.pan, ptuPose.pose.tilt);
      } else {
        // If we are not connecting and reading from PTU we assume
        // angles are 0
        ptuPose.pose.pan = 0;
        ptuPose.pose.tilt = -0.75;
      }

      {
        IceUtil::Mutex::Lock lock(m_Mutex);

        if (m_RobotPose && m_ShowPointCloud) {
          PointCloud::SurfacePointSeq points;
          getPoints(true, 640 / 4, points);
          peekabot::ColoredVertexSet kinectVerts;
          double rangeMax = 1;
          for (unsigned int i = 0; i < points.size(); i += 16) {
            /* Transform point in cloud with regards to the robot pose */
            Cure::Transformation3D robotTransform;
            robotTransform.setXYTheta(m_RobotPose->x, m_RobotPose->y,
                m_RobotPose->theta);
            robotTransform = robotTransform;
            Cure::Vector3D from(points[i].p.x, points[i].p.y, points[i].p.z);
            Cure::Vector3D to;
            robotTransform.invTransform(from, to);
            double pX = to.X[0];
            double pY = to.X[1];
            double pZ = to.X[2];
            cogx::Math::Vector3 p;
            p.x = points[i].p.x;
            p.y = points[i].p.y;
            p.z = points[i].p.z;
            if (!isPointInViewCone(p))
              continue;
            kinectVerts.add(pX, pY, pZ, 1, std::max(0.0, (1 - pZ / rangeMax)),
                0);
          }
          m_ProxyKinect.set_vertices(kinectVerts);
        }

        debug("Display robot pose");
        // Display robot pose
        if (m_ShowRobot && m_RobotPose) {
          m_ProxyRobot.set_pose(m_RobotPose->x, m_RobotPose->y, 0,
              m_RobotPose->theta);
          if (m_FollowRobot) {
            m_DefCam.set_position(m_RobotPose->x, m_RobotPose->y, 0);
          }

          if (m_ShowPanTilt) {
            m_ProxyPan.set_dof(ptuPose.pose.pan);
            m_ProxyTilt.set_dof(-ptuPose.pose.tilt);
          }
        }

        // Display the line map
        if (m_ShowWalls && m_LineMap) {

          log("Displaying %d wall segments", m_LineMap->lines.size());

          for (int i = m_LineMap->lines.size(); i > 10000; i++) {
            peekabot::PolygonProxy pp;
            char name[32];
            sprintf(name, "wall%d", i);
            peekabot::Status s = pp.assign(m_ProxyWalls, name).status();
            if (s.succeeded()) {
              pp.remove();
            } else {
              break;
            }
          }

          for (unsigned int i = 0; i < m_LineMap->lines.size(); i++) {

            peekabot::PolygonProxy pp;
            char buf[32];
            sprintf(buf, "wall%d", i);
            pp.add(m_ProxyWalls, buf, peekabot::REPLACE_ON_CONFLICT);

            peekabot::VertexSet vs;
            vs.add(m_LineMap->lines[i].start.x, m_LineMap->lines[i].start.y, 0);
            vs.add(m_LineMap->lines[i].start.x, m_LineMap->lines[i].start.y,
                1.0);
            vs.add(m_LineMap->lines[i].end.x, m_LineMap->lines[i].end.y, 1.0);
            vs.add(m_LineMap->lines[i].end.x, m_LineMap->lines[i].end.y, 0.0);

            pp.add_vertices(vs);
            pp.set_color(255. / 255, 198. / 255, 0.);
            pp.set_opacity(0.2);

          }

        }
        log("Display the people");
        // Display the people being tracked
        if (m_ShowPeople)
          displayPeople();

      }

      // Make sure the server processed what we've sent
      m_PeekabotClient.sync();

      usleep(250000);
    }
  }
}

void DisplayNavInPB::displayPeople() {
  peekabot::GroupProxy people;
  people.add(m_PeekabotClient, "people", peekabot::REPLACE_ON_CONFLICT);

  for (unsigned int i = 0; i < m_People.size(); i++) {

    char buf[32];
    sprintf(buf, "person%ld", (long) m_People[i].m_data->id);

    if (m_NoPeopleModel) {

      // Using a cylinde rmodel instead of beautiful Rolf

      peekabot::CylinderProxy cp;
      cp.add(people, buf, peekabot::REPLACE_ON_CONFLICT);
      cp.set_pose(m_People[i].m_data->x, m_People[i].m_data->y, 0.9,
          m_People[i].m_data->direction);
      cp.set_scale(0.1, 0.2, 1.8);

      if (m_CurrPersonId == m_People[i].m_data->id) {
        cp.set_opacity(1);
      } else {
        cp.set_opacity(0.2);
      }

    } else {

      peekabot::ModelProxy mp;
      mp.add(people, buf, m_PbPersonFile, peekabot::REPLACE_ON_CONFLICT);
      mp.set_pose(m_People[i].m_data->x, m_People[i].m_data->y, 0,
          m_People[i].m_data->direction);

      if (m_CurrPersonId == m_People[i].m_data->id) {
        mp.set_opacity(1);
      } else {
        mp.set_opacity(0.2);
      }

      if (m_ShowPeopleId) {
        sprintf(buf, "id%ld", (long) m_People[i].m_data->id);
        peekabot::LabelProxy text;
        text.add(mp, buf, peekabot::REPLACE_ON_CONFLICT);
        sprintf(buf, "%ld", (long) m_People[i].m_data->id);
        text.set_text(buf);
        text.set_pose(0, 0, 1.8, M_PI / 2, 0, M_PI / 2);
        text.set_scale(30, 30, 30);
        text.set_alignment(peekabot::ALIGN_CENTER);
        text.set_color(1, 0, 0);
      }
    }
  }

}

void DisplayNavInPB::newRobotPose(const cdl::WorkingMemoryChange &objID) {
  log("Entered newRobotPose");

  NavData::RobotPose2dPtr oobj = getMemoryEntry<NavData::RobotPose2d> (
      objID.address);

  {
    IceUtil::Mutex::Lock lock(m_Mutex);
    m_RobotPose = oobj;
  }
  log("newRobotPose(x=%.2f y=%.2f a=%.4f t=%ld.%06ld", m_RobotPose->x,
      m_RobotPose->y, m_RobotPose->theta, (long) m_RobotPose->time.s,
      (long) m_RobotPose->time.us);
  if (m_ShowPath) {
    if (objID.operation == cdl::ADD) {
      m_ProxyPathStartMarker.add(m_PeekabotClient, "PathStart",
          peekabot::REPLACE_ON_CONFLICT);
      peekabot::VertexSet set1;
      set1.add(0, 0, 0);
      set1.add(-0.5, 0.5, 0);
      set1.add(-0.3, 0, 0);
      set1.add(-0.5, -0.5, 0);
      m_ProxyPathStartMarker.add_vertices(set1);
      m_ProxyPathStartMarker.set_rotation(m_RobotPose->theta, 0, 0);
      m_ProxyPathStartMarker.set_position(m_RobotPose->x, m_RobotPose->y, 0.02);

      m_ProxyPathEndMarker.add(m_PeekabotClient, "PathEnd",
          peekabot::REPLACE_ON_CONFLICT);
      peekabot::VertexSet set2;
      set2.add(0.3, 0, 0);
      set2.add(-0.2, 0.5, 0);
      set2.add(0, 0, 0);
      set2.add(-0.2, -0.5, 0);
      m_ProxyPathEndMarker.add_vertices(set2);

      m_ProxyPathEndMarker.set_rotation(m_RobotPose->theta, 0, 0);
      m_ProxyPathEndMarker.set_position(m_RobotPose->x, m_RobotPose->y, 0.02);
    }
    double diffSq = (m_RobotPose->x - m_lastLoggedX) * (m_RobotPose->x
        - m_lastLoggedX) + (m_RobotPose->y - m_lastLoggedY) * (m_RobotPose->y
        - m_lastLoggedY);
    if (diffSq > 0.01) {
      peekabot::VertexSet set;
      set.add(m_RobotPose->x, m_RobotPose->y, 0.01);
      m_ProxyPathLog.add_vertices(set);
      m_lastLoggedX = m_RobotPose->x;
      m_lastLoggedY = m_RobotPose->y;

      FrontierInterface::PlaceInterfacePrx agg2(getIceServer<
          FrontierInterface::PlaceInterface> ("place.manager"));

      NavData::FNodePtr node = new NavData::FNode;
      SpatialData::PlacePtr place = new SpatialData::Place;
      place = agg2->getCurrentPlace();
      if (place) {

        node = agg2->getNodeFromPlaceID(place->id);
        std::string roomid = lexical_cast<string> (node->areaId);

        try {
          ConceptualData::QueryHandlerServerInterfacePrx
              m_queryHandlerServerInterfacePrx(getIceServer<
                  ConceptualData::QueryHandlerServerInterface> (
                  "conceptual.queryhandler"));
          ConceptualData::ProbabilityDistributions conceptualProbdist =
              m_queryHandlerServerInterfacePrx->query("p(*room" + roomid
                  + "_category)");
          log("prod dist has: %d", conceptualProbdist.size());
          if (conceptualProbdist.size() != 0) {
            SpatialProbabilities::ProbabilityDistribution probdist =
                conceptualProbdist[0];
            double maxprob = -1;
            int maxindex = -1;
            log("massfunction has: %d", probdist.massFunction.size());
            for (unsigned int i = 0; i < probdist.massFunction.size(); i++) {
              if (probdist.massFunction[i].probability > maxprob) {
                maxindex = i;
                maxprob = probdist.massFunction[i].probability;
              }
            }
            log("maxprob index is: %d", maxindex);
            SpatialProbabilities::StringRandomVariableValuePtr val =
                new SpatialProbabilities::StringRandomVariableValue;
            val
                = SpatialProbabilities::StringRandomVariableValuePtr::dynamicCast(
                    probdist.massFunction[maxindex].variableValues[0]);
            peekabot::PolylineProxy ProxyPathLogtemp;
            if (val->value != m_currentMostLikelyRoom) {
              m_currentMostLikelyRoom = val->value;
              log("Changing line color!");
              ProxyPathLogtemp.add(m_ProxyTrajectory, "path_log",
                  peekabot::AUTO_ENUMERATE_ON_CONFLICT);
              peekabot::VertexSet set;
              set.add(m_RobotPose->x, m_RobotPose->y, 0.01);
              ProxyPathLogtemp.add_vertices(set);
              ProxyPathLogtemp.set_line_width(5);
              m_ProxyPathLog = ProxyPathLogtemp;
            }
            int index = -1;
            // @demmeln 22.03.2012: prob seems unused. Comment out.
            //				double prob = 0.0;
            for (unsigned int j = 0; j < _roomCategories.size(); ++j) {
              if (val->value == _roomCategories[j]) {
                index = j;
              }
            }
            float r, g, b;
            getColorByIndex(index, r, g, b);
            m_ProxyPathLog.set_color(r, g, b);
          }
        } catch (const cast::CASTException& e) {
          getLogger()->warn(
              "CASTException when displaying path, disabling path display: "
                  + std::string(e.what()));
          m_ShowPath = false;
        }
      }
    }
  }
  debug("Exited newRobotPose");
}

void DisplayNavInPB::newNavGraphObject(const cdl::WorkingMemoryChange &objID) {
  if (!m_ShowObjects)
    return;

  log("Entered newNavGraphObject");

  shared_ptr<CASTData<NavData::ObjData> > oobj = getWorkingMemoryEntry<
      NavData::ObjData> (objID.address);

  NavData::ObjDataPtr objData = oobj->getData();

  if (!m_PeekabotClient.is_connected()) {
    log(
        "Received an object of category %s, not displaying it since not connected to peekabot",
        objData->category.c_str());
    debug("Exited newNavGraphObject");
    return;
  }
  log("Received an object of category %s", objData->category.c_str());

  IceUtil::Mutex::Lock lock(m_Mutex);

  peekabot::CylinderProxy sp;
  peekabot::LabelProxy text;

  if (m_peekabotObjectFiles.count(objData->category) > 0) {

    peekabot::ModelProxy objProxy;

    if (m_NonUniqueObjects) {
      objProxy.add(m_ProxyObjects, objData->category,
          m_peekabotObjectFiles[objData->category].c_str());
    } else {
      objProxy.add(m_ProxyObjects, objData->category,
          m_peekabotObjectFiles[objData->category].c_str(),
          peekabot::REPLACE_ON_CONFLICT);
    }

    if (objData->angles.empty()) {

      if (m_RobotPose) {

        objProxy.set_pose(objData->x, objData->y, objData->z, atan2(objData->y
            - m_RobotPose->y, objData->x - m_RobotPose->x), 0, 0);
      } else {
        // Assume robot is at 0,0,0
        objProxy.set_pose(objData->x, objData->y, objData->z, atan2(objData->y
            - 0, objData->x - 0), 0, 0);
      }

    } else {
      double ang[3] = { 0, 0, 0 };
      for (unsigned int i = 0; i < objData->angles.size(); i++) {
        ang[i] = objData->angles[i];
      }
      objProxy.set_pose(objData->x, objData->y, objData->z, ang[0], ang[1],
          ang[2]);
    }
    text.add(objProxy, "label");
  } else {
    // Add a center marker just in case the model file did not exist
    peekabot::CubeProxy centerProxy;
    centerProxy.add(m_ProxyObjects, objData->category);
    centerProxy.set_scale(0.02, 0.08, 0.18);
    centerProxy.set_color(0, 1, 0);
    if (objData->angles.empty()) {

      if (m_RobotPose) {

        centerProxy.set_pose(objData->x, objData->y, objData->z, atan2(
            objData->y - m_RobotPose->y, objData->x - m_RobotPose->x), 0, 0);
      } else {
        // Assume robot is at 0,0,0
        centerProxy.set_pose(objData->x, objData->y, objData->z, atan2(
            objData->y - 0, objData->x - 0), 0, 0);
      }

    } else {
      double ang[3] = { 0, 0, 0 };
      for (unsigned int i = 0; i < objData->angles.size(); i++) {
        ang[i] = objData->angles[i];
      }
      centerProxy.set_pose(objData->x, objData->y, objData->z, ang[0], ang[1],
          ang[2]);
    }
    text.add(centerProxy, "label");
  }

  text.set_text(objData->category);
  text.set_position(0, 0, 0.5);
  text.set_rotation(-M_PI_2, 0, M_PI_2);
  text.set_scale(20, 20, 20);
  text.set_alignment(peekabot::ALIGN_CENTER); //see TextAlignment in peekabot/src/Types.hh for more.
  text.set_color(0, 0, 1);

  debug("Exited newNavGraphObject");
}

void DisplayNavInPB::newLineMap(const cdl::WorkingMemoryChange &objID) {
  log("newLineMap called");

  shared_ptr<CASTData<NavData::LineMap> > oobj = getWorkingMemoryEntry<
      NavData::LineMap> (objID.address);

  {
    IceUtil::Mutex::Lock lock(m_Mutex);
    m_LineMap = oobj->getData();
  }
  debug("Exited newLineMap");
}

void DisplayNavInPB::newPerson(const cdl::WorkingMemoryChange &objID) {
  debug("Entered newPerson");
  // Person entries can be removed at any time
  try {
    shared_ptr<CASTData<NavData::Person> > oobj = getWorkingMemoryEntry<
        NavData::Person> (objID.address);

    NavData::PersonPtr p = oobj->getData();

    bool addNewPerson = true;

    IceUtil::Mutex::Lock lock(m_Mutex);

    // Check if the person already exists, otherwise add it
    for (unsigned int i = 0; i < m_People.size(); i++) {
      if (m_People[i].m_data->id == p->id) {
        // Update it

        char buf[256];
        sprintf(buf, "Got new person at x=%.2f y=%.2f theta=%.2f id=%ld", p->x,
            p->y, p->direction, (long) p->id);
        debug(buf);

        m_People[i].m_data = p;
        addNewPerson = false;
        break;
      }
    }

    if (addNewPerson) {
      DisplayNavInPB::PersonData pd;
      pd.m_WMid = objID.address.id;
      pd.m_data = p;
      m_People.push_back(pd);
    }

  } catch (DoesNotExistOnWMException) {
  }

  debug("Exited newPerson");
}

void DisplayNavInPB::deletePerson(const cdl::WorkingMemoryChange &objID) {
  debug("Entered deletePerson");
  int i = 0;
  for (std::vector<DisplayNavInPB::PersonData>::iterator pi = m_People.begin(); pi
      != m_People.end(); pi++, i++) {

    if (objID.address.id == pi->m_WMid) {
      m_People.erase(pi);
      break;
    }
  }
  debug("Exited deletePerson");
}

void DisplayNavInPB::newPersonFollowed(const cdl::WorkingMemoryChange &objID) {
  debug("Entered newPersonFollowed");
  shared_ptr<CASTData<NavData::PersonFollowed> > oobj = getWorkingMemoryEntry<
      NavData::PersonFollowed> (objID.address);

  IceUtil::Mutex::Lock lock(m_Mutex);
  m_CurrPersonId = oobj->getData()->id;
  char buf[256];
  sprintf(buf, "Got id of person being tracked %d", m_CurrPersonId);
  debug(buf);
  debug("Exited newPersonFollowed");
}

void DisplayNavInPB::newNavCommand(const cdl::WorkingMemoryChange & objID) {
  debug("Entered newNavCommand");
  try {
    SpatialData::NavCommandPtr oobj = getMemoryEntry<SpatialData::NavCommand> (
        objID.address);

    if (oobj != 0 && oobj->cmd == SpatialData::GOTOPLACE) {
      FrontierInterface::PlaceInterfacePrx piPrx(getIceServer<
          FrontierInterface::PlaceInterface> ("place.manager"));
      /* Reset the old goal (if any) */
      if (m_currGoalPlace != -1) {
        /* Was the old goal a placeholder? */
        SpatialData::NodeHypothesisPtr nodeHypPtr = piPrx->getHypFromPlaceID(
            m_currGoalPlace);
        if (nodeHypPtr) {
          // Get the node proxy
          peekabot::SphereProxy sp;
          std::stringstream ss;
          ss << "node_hyp" << nodeHypPtr->hypID << "/goalMarker";
          log("Removing old goal marker: %s", ss.str().c_str());
          sp.assign(m_ProxyNodes, ss.str());
          sp.remove();
        }
        /* Or was it a proper place? */
        NavData::FNodePtr fnodePtr = piPrx->getNodeFromPlaceID(m_currGoalPlace);
        if (fnodePtr) {
          // Get the node proxy
          peekabot::SphereProxy sp;
          std::stringstream ss;
          ss << "node" << fnodePtr->nodeId << "/goalMarker";
          log("Removing old goal marker: %s", ss.str().c_str());
          sp.assign(m_ProxyNodes, ss.str());
          sp.remove();
        }
      }

      if (oobj->destId.empty()) {
        log("No destID. Returning.");
        debug("Exited newNavCommand");
        return;
      }

      log("Updating goal from %d to %d", m_currGoalPlace, oobj->destId[0]);

      /* Update new goal */
      m_currGoalPlace = oobj->destId[0];

      /* Was the old goal a placeholder? */
      SpatialData::NodeHypothesisPtr nodeHypPtr = piPrx->getHypFromPlaceID(
          m_currGoalPlace);
      if (nodeHypPtr) {
        // Get the node proxy
        peekabot::CubeProxy parent;
        std::stringstream ss;
        ss << "node_hyp" << nodeHypPtr->hypID;
        log("Adding goal marker: %s", ss.str().c_str());
        parent.assign(m_ProxyNodes, ss.str());
        peekabot::SphereProxy sp;
        sp.add(parent, "goalMarker");
        sp.set_scale(0.5, 0.5, 0.5);
        sp.set_color(0, 1, 0);
        sp.set_opacity(0.3);
      }
      /* Or was it a proper place? */
      NavData::FNodePtr fnodePtr = piPrx->getNodeFromPlaceID(m_currGoalPlace);
      if (fnodePtr) {
        // Get the node proxy
        peekabot::SphereProxy parent;
        std::stringstream ss;
        ss << "node" << fnodePtr->nodeId;
        log("Adding goal marker: %s", ss.str().c_str());
        parent.assign(m_ProxyNodes, ss.str());
        peekabot::SphereProxy sp;
        sp.add(parent, "goalMarker");
        sp.set_scale(0.5, 0.5, 0.5);
        sp.set_color(0, 1, 0);
        sp.set_opacity(0.3);
      }
    }
  } catch (DoesNotExistOnWMException) {
    log("NavCommand %s disppeared from WM in newNavCommand!",
        objID.address.id.c_str());
  }
  debug("Exited newNavCommand");
}

void DisplayNavInPB::newNavGraphNode(const cdl::WorkingMemoryChange &objID) {
  log("Entered newNavGraphNode");

  try {
    if (!m_PeekabotClient.is_connected()) {
      debug("Exited newNavGraphNode");
      return;
    }

    shared_ptr<CASTData<NavData::FNode> > oobj = getWorkingMemoryEntry<
        NavData::FNode> (objID.address);

    NavData::FNodePtr fnode = oobj->getData();

    int placeId = GetPlaceIdFromNodeId(fnode->nodeId);
    debug("5 %d %d", placeId, fnode->nodeId);
    if (placeId != -1) {
      for (map<string, std::pair<long, long> >::iterator it = _cpp.begin(); it
          != _cpp.end(); ++it) {
        debug("6 %d - %d %d", placeId, it->second.first, it->second.second);

        if (it->second.first == placeId || it->second.second == placeId) {
          displayConnectivityPathProperty(it->second.first, it->second.second);
        }
      }
    }

    // Update DisplayNavInPB
    IceUtil::Mutex::Lock lock(m_Mutex);

    // Update the node
    std::map<long, Node>::iterator n = m_Nodes.find(fnode->nodeId);
    if (n == m_Nodes.end()) { // Node does not exist, create new
      debug("Node %d new", (int) fnode->nodeId);

      DisplayNavInPB::Node node;
      node.m_Id = fnode->nodeId;
      node.m_Gateway = (fnode->gateway != 0);
      node.m_X = fnode->x;
      node.m_Y = fnode->y;
      node.m_areaId = fnode->areaId;
      if (!fnode->type.empty())
        node.m_AreaClassNo = fnode->type[0].id;
      else
        node.m_AreaClassNo = -1;
      m_Nodes.insert(std::make_pair(node.m_Id, node));

      // Update Peekabot
      peekabot::SphereProxy sp;
      char name[32];
      sprintf(name, "node%ld", (long) fnode->nodeId);
      sp.add(m_ProxyNodes, name);
      sp.set_position(fnode->x, fnode->y, 0);

      float r, g, b;
      if (fnode->gateway) {
        // add gateway sphere
        sp.set_scale(0.2, 0.2, 0.05);
        getColorByIndex(1, r, g, b);

        double width = 1;
        if (!fnode->width.empty())
          width = fnode->width[0];
        addDoorpost(fnode->x, fnode->y, fnode->theta, width, sp);

        debug("Added gateway with id %d", fnode->nodeId);
      } else {
        r = 0;
        g = 0;
        b = 0;
        // set root sphere size zero
        sp.set_scale(0, 0, 0);
        debug("Added normal node with id %d", fnode->nodeId);
      }
      sp.set_color(r, g, b);

      if (m_ShowRoomId && !fnode->gateway) {
        // add translucent area circle
        peekabot::CylinderProxy acp;
        char name2[32];
        sprintf(name2, "node%ld/area_class", (long) fnode->nodeId);
        acp.add(m_ProxyNodes, name2, peekabot::REPLACE_ON_CONFLICT);
        acp.set_scale(0.5, 0.5, 0.0);
        acp.set_position(0, 0, 0);
        acp.set_opacity(0.3);
        acp.set_color(0, 0, 0); // Node is not assigned to a comaRoom so colour it black
      }

    } else { // Node already exists
      debug("Node %d already there, should be changed", fnode->nodeId);

      n->second.m_Id = fnode->nodeId;
      n->second.m_Gateway = (fnode->gateway != 0);
      n->second.m_X = fnode->x;
      n->second.m_Y = fnode->y;
      n->second.m_areaId = fnode->areaId;
      if (!fnode->type.empty())
        n->second.m_AreaClassNo = fnode->type[0].id;
      else
        n->second.m_AreaClassNo = -1;

      // Update Peekabot
      peekabot::SphereProxy sp;
      char name[32];
      sprintf(name, "node%ld", (long) fnode->nodeId);
      sp.add(m_ProxyNodes, name, peekabot::REPLACE_ON_CONFLICT);
      sp.set_position(fnode->x, fnode->y, 0);

      float r, g, b;
      if (fnode->gateway) {
        sp.set_scale(0.2, 0.2, 0.05);
        getColorByIndex(1, r, g, b);

        double width = 1;
        if (!fnode->width.empty())
          width = fnode->width[0];
        addDoorpost(fnode->x, fnode->y, fnode->theta, width, sp);

        debug("modified gateway with id %d", fnode->nodeId);
      } else {
        // Do not show root sphere for non-doorways
        r = 0;
        g = 0;
        b = 0;
        sp.set_scale(0, 0, 0);
        debug("modified normal node with id %d", fnode->nodeId);
      }
      sp.set_color(r, g, b);

      if (m_ShowRoomId && !fnode->gateway) {
        peekabot::CylinderProxy acp;
        char name2[32];
        sprintf(name2, "node%ld/area_class", (long) fnode->nodeId);
        acp.add(m_ProxyNodes, name2, peekabot::REPLACE_ON_CONFLICT);
        acp.set_scale(0.5, 0.5, 0.0);
        acp.set_position(0, 0, 0);
        acp.set_opacity(0.3);
        acp.set_color(r, g, b);
      }

      // We also redraw the edge just to make sure that it is drawn
      // correctly if the position of the node changed.
      redisplayEdgesToNode(n->second);
    }

    for (std::list<std::pair<long, long> >::iterator ei = m_NewEdges.begin(); ei
        != m_NewEdges.end();) {

      // Check if the nodes are there now
      std::map<long, Node>::iterator n1 = m_Nodes.find(ei->first);
      std::map<long, Node>::iterator n2 = m_Nodes.find(ei->second);

      if (n1 != m_Nodes.end() && n2 != m_Nodes.end()) { // Found nodes

        displayEdge(n1->second, n2->second);
        addEdgeToList(n1->second.m_Id, n2->second.m_Id);
        ei = m_NewEdges.erase(ei);
      } else {
        ei++;
      }
    }
  } catch (DoesNotExistOnWMException e) {
    log("Error! SpatialObject disappeared from WM!");
  }
  log("Exited newNavGraphNode");

}

void DisplayNavInPB::movePlace(const cdl::WorkingMemoryChange &wmChange) {
  log("Entered movePlace");
  SpatialData::NodeHypothesisPtr nodePtr;
  try {
    nodePtr = getMemoryEntry<SpatialData::NodeHypothesis> (wmChange.address);
  } catch (CASTException &e) {
    log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
    debug("Exited movePlace");
    return;
  }
  FrontierInterface::PlaceInterfacePrx piPrx(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));

  SpatialData::PlacePtr placePtr = piPrx->getPlaceFromHypID(nodePtr->hypID);

  if (placePtr) {
    int pid = placePtr->id;
    if (m_ShowPlaceholders) {

      peekabot::CubeProxy sp;
      char name[32];
      sprintf(name, "node_hyp%ld", (long) nodePtr->hypID);
      peekabot::Status s = sp.assign(m_ProxyNodes, name).status();

      if (s.succeeded()) {

        sp.set_position(nodePtr->x, nodePtr->y, 0);

        // Delete ID text
        if (m_ShowLabels) {

          peekabot::LabelProxy text;
          char buf[32];
          sprintf(buf, "%d", pid);
          text.assign(m_ProxyLabels, buf);
          text.set_position(nodePtr->x, nodePtr->y, 0);
        }

//        int nodeId;
        int originPlaceID;
        double hypX, hypY;
        try {
  //        nodeId = nodePtr->hypID;
          originPlaceID = nodePtr->originPlaceID;
          hypX = nodePtr->x;
          hypY = nodePtr->y;
        } catch (IceUtil::NullHandleException &e) {
          log("nodeHypPtr is no longer valid.\n");
          return;
        }
        double originX = 0;
        double originY = 0;
        NavData::FNodePtr fnodePtr;
        fnodePtr = piPrx->getNodeFromPlaceID(originPlaceID);
        try {
          originX = fnodePtr->x;
          originY = fnodePtr->y;
        } catch (IceUtil::NullHandleException &e) {
          log("Could not get origin node.\n");
        }
        if (fnodePtr) {
          //			    int parentNodeID= fnodePtr->nodeId;
          peekabot::PolylineProxy lcp;
          peekabot::VertexSet points;
          points.add(0, 0, 0);
          points.add(originX - hypX, originY - hypY, 0);
          lcp.add(sp, "parent", peekabot::REPLACE_ON_CONFLICT);
          lcp.set_line_style("dotted", 0.5);
          lcp.add_vertices(points);
        }

      }
    }

    for (map<string, std::pair<long, long> >::iterator it = _cpp.begin(); it
        != _cpp.end(); ++it) {
      debug("1 %d - %d %d", pid, it->second.first, it->second.second);
      if (it->second.first == pid || it->second.second == pid) {
        displayConnectivityPathProperty(it->second.first, it->second.second);
      }
    }

  }

  log("Exited movePlace");
}

void DisplayNavInPB::newPlace(const cdl::WorkingMemoryChange &wmChange) {
  log("Entered newPlace");
  SpatialData::PlacePtr placePtr;
  try {
    placePtr = getMemoryEntry<SpatialData::Place> (wmChange.address);
  } catch (CASTException &e) {
    log("Caught exception at %s. Message: %s", __HERE__, e.message.c_str());
    debug("Exited newPlace");
    return;
  }

  // Init
  int placeId;
  bool placeholder;
  try {
    placeId = placePtr->id;
    placeholder = (placePtr->status == SpatialData::PLACEHOLDER);
  } catch (IceUtil::NullHandleException e) {
    log("Place suddenly disappeared!\n");
    debug("Exited newPlace");
    return;
  }
  // Get node Id for this place
  FrontierInterface::PlaceInterfacePrx piPrx(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));

  // Does place exist? If so, delete it first.
  log("Delete place if it existed.. ");
  if (_places.find(wmChange.address.id) != _places.end()) {
    deletePlace(wmChange);
  }
  debug("done.\n");

  // Is it a place or a placeholder
  if (placeholder) {
    if (m_ShowPlaceholders) {
      SpatialData::NodeHypothesisPtr nodeHypPtr = piPrx->getHypFromPlaceID(
          placeId);
      if (!nodeHypPtr) {
        log("Hyp doesnt exist\nExiting.\n");
        return;
      }

      int nodeId;
      int originPlaceID;
      double hypX, hypY;
      try {
        nodeId = nodeHypPtr->hypID;
        originPlaceID = nodeHypPtr->originPlaceID;
        hypX = nodeHypPtr->x;
        hypY = nodeHypPtr->y;
      } catch (IceUtil::NullHandleException &e) {
        log("nodeHypPtr is no longer valid.\n");
        return;
      }

      double originX = 0;
      double originY = 0;
      NavData::FNodePtr fnodePtr;
      fnodePtr = piPrx->getNodeFromPlaceID(originPlaceID);
      try {
        originX = fnodePtr->x;
        originY = fnodePtr->y;
      } catch (IceUtil::NullHandleException &e) {
        log("Could not get origin node.\n");
      }

      // Create internal representation of the placeholder
      PlaceData pd;
      pd.nodeId = nodeId;
      pd.placeId = placeId;
      pd.placeStatus = SpatialData::PLACEHOLDER;
      _places[wmChange.address.id] = pd;

      // Draw the placeholder
      // Add text
      if (m_ShowLabels) {
        char buf[32];
        peekabot::LabelProxy text;
        sprintf(buf, "%d", placeId);
        text.add(m_ProxyLabels, buf, peekabot::REPLACE_ON_CONFLICT);
        text.set_text(buf);
        text.set_pose(hypX, hypY, 0.3, 0, 0.0, 0);
        text.set_rotation(0, 0, 0);
        text.set_scale(20, 20, 20);
        text.set_alignment(peekabot::ALIGN_CENTER);
        text.set_color(1, 0, 0);
      }

      // Mashroom
      peekabot::CubeProxy sp;
      char name[32];
      sprintf(name, "node_hyp%ld", (long) nodeHypPtr->hypID);
      sp.add(m_ProxyNodes, name, peekabot::REPLACE_ON_CONFLICT);
      sp.set_position(hypX, hypY, 0);
      sp.set_scale(0.2, 0.2, 0.05);
      sp.set_color(0.2, 0.2, 0.2);

      peekabot::CubeProxy cp;
      cp.add(sp, "class", peekabot::REPLACE_ON_CONFLICT);
      cp.set_scale(0.08, 0.08, 0.20);
      cp.set_position(0, 0, 0.08);
      cp.set_color(0.2, 0.2, 0.2);

      peekabot::CubeProxy mp;
      mp.add(sp, "mushroom");
      mp.set_scale(0.16, 0.16, 0.05);
      mp.set_position(0, 0, 0.20);
      mp.set_color(0.2, 0.2, 0.2);

      // Origin
      if (nodeHypPtr) {
        int originPlaceID = nodeHypPtr->originPlaceID;

        NavData::FNodePtr fnodePtr = piPrx->getNodeFromPlaceID(originPlaceID);
        if (fnodePtr) {
          //			    int parentNodeID= fnodePtr->nodeId;
          peekabot::PolylineProxy lcp;
          peekabot::VertexSet points;
          points.add(0, 0, 0);
          points.add(originX - hypX, originY - hypY, 0);
          lcp.add(sp, "parent", peekabot::REPLACE_ON_CONFLICT);
          lcp.set_line_style("dotted", 0.5);
          lcp.add_vertices(points);
        }
      }

      // Draw placeholder properties
      addRoomCategoryPlaceholderProperties(sp, placeId);
    }
  } else {
    NavData::FNodePtr fnodePtr = piPrx->getNodeFromPlaceID(placeId);
    int nodeId = (fnodePtr) ? fnodePtr->nodeId : -1;

    // Create internal representation of the placeholder
    PlaceData pd;
    pd.nodeId = nodeId;
    pd.placeId = placeId;
    pd.placeStatus = SpatialData::PLACEHOLDER;
    _places[wmChange.address.id] = pd;

    // Add PlaceID text to existing node
    if (m_ShowLabels) {
      std::map<long, Node>::iterator n = m_Nodes.find(nodeId);
      if (n != m_Nodes.end()) {
        char buf[32];
        peekabot::LabelProxy text;
        sprintf(buf, "%d", placeId);
        text.add(m_ProxyLabels, buf, peekabot::REPLACE_ON_CONFLICT);
        text.set_text(buf);
        text.set_pose(n->second.m_X, n->second.m_Y, 0.3, 0, 0.0, 0);
        text.set_rotation(0, 0, 0);
        text.set_scale(20, 20, 20);
        text.set_alignment(peekabot::ALIGN_CENTER);
        text.set_color(1, 0, 0);
      }
    }
  }
  if (placeId != -1) {
    for (map<string, std::pair<long, long> >::iterator it = _cpp.begin(); it
        != _cpp.end(); ++it) {
      debug("8 %d - %d %d", placeId, it->second.first, it->second.second);

      if (it->second.first == placeId || it->second.second == placeId) {
        displayConnectivityPathProperty(it->second.first, it->second.second);
      }
    }
  }

  log("Exited newPlace");
}

void DisplayNavInPB::deletePlace(const cdl::WorkingMemoryChange &wmChange) {
  log("Entered deletePlace");
  // Find and remove the place data
  string wmId = wmChange.address.id;
  map<string, PlaceData>::iterator it = _places.find(wmChange.address.id);
  if (it == _places.end()) {
    debug("Exited deletePlace");
    return;
  }
  PlaceData pd = it->second;
  _places.erase(it);
  bool placeholder = (pd.placeStatus == SpatialData::PLACEHOLDER);

  // Remove peekabot representation of the place
  if (placeholder) {
    if (m_ShowPlaceholders) {
      if (m_ShowLabels) {
        // Delete ID text
        peekabot::LabelProxy text;
        char buf[32];
        sprintf(buf, "%d", pd.placeId);
        text.assign(m_ProxyLabels, buf);
        text.remove();
      }

      // Delete the node
      peekabot::CubeProxy sp;
      char name[32];
      sprintf(name, "node_hyp%ld", (long) pd.nodeId);
      sp.assign(m_ProxyNodes, name);
      sp.remove();
    }
  } else {
    if (m_ShowLabels) {
      // Delete ID text
      peekabot::LabelProxy text;
      char buf[32];
      sprintf(buf, "%d", pd.placeId);
      text.assign(m_ProxyLabels, buf);
      text.remove();
    }
  }
  debug("Exited deletePlace");
}

void DisplayNavInPB::addRoomCategoryPlaceholderProperties(
    peekabot::CubeProxy &sp, int placeId) {
  map<string, double> existsProbs;

  // Get all the properties for that place
  for (std::map<string, SpatialProperties::RoomCategoryPlaceholderPropertyPtr>::iterator
      it = _roomCatPlaceholderProps.begin(); it
      != _roomCatPlaceholderProps.end(); ++it) {
    if (it->second->placeId == placeId) {
      double prob = getProbabilityValue(it->second->distribution, 1);
      existsProbs[it->second->category] = prob;
    }
  }

  // Pie chart of category probabilities
  double probSum = 0.0;
  double startAngle = 0.0;
  double normalizer = _roomCategories.size();
  for (map<string, double>::iterator it = existsProbs.begin(); it
      != existsProbs.end(); ++it) {
    int index = -1;
    double prob = 0.0;
    for (unsigned int j = 0; j < _roomCategories.size(); ++j) {
      if (it->first == _roomCategories[j]) {
        index = j;
        prob = it->second;
      }
    }

    if (index >= 0) { // One of those room categories
      float r, g, b;
      getColorByIndex(index, r, g, b);
      char name[32];
      sprintf(name, "category%d", index);
      // Calculations
      probSum += prob;
      double endAngle = (2 * 3.1415926) * probSum / normalizer;

      // Add a polygon that corresponds to part of the pie chart
      // between startAngle and endAngle;
      peekabot::PolygonProxy acp;
      peekabot::VertexSet vs;
      acp.add(sp, name, peekabot::REPLACE_ON_CONFLICT);
      vs.add(0, 0, 0);
      for (double tmp = startAngle; tmp < endAngle; tmp += (2.0 * 3.1415926
          / 36))
        vs.add(0.5 * cosf(tmp), 0.5 * sinf(tmp), 0);
      vs.add(0.5 * cosf(endAngle), 0.5 * sinf(endAngle), 0);
      acp.add_vertices(vs);
      acp.set_opacity(0.3);
      acp.set_color(r, g, b);
      startAngle = endAngle;
    }
  }

  // Add one for the rest
  double endAngle = (2 * 3.1415926);
  peekabot::PolygonProxy acp;
  peekabot::VertexSet vs;
  acp.add(sp, "not_exists", peekabot::REPLACE_ON_CONFLICT);
  vs.add(0, 0, 0);
  for (double tmp = startAngle; tmp < endAngle; tmp += (2.0 * 3.1415926 / 36))
    vs.add(0.5 * cosf(tmp), 0.5 * sinf(tmp), 0);
  vs.add(0.5 * cosf(endAngle), 0.5 * sinf(endAngle), 0);
  acp.add_vertices(vs);
  acp.set_opacity(0.3);
  acp.set_color(0.7, 0.7, 0.7);
  startAngle = endAngle;
}

int DisplayNavInPB::GetPlaceIdFromNodeId(int nodeId) {
  FrontierInterface::PlaceInterfacePrx agg(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));
  SpatialData::PlacePtr place = agg->getPlaceFromNodeID(nodeId);
  if (place) {
    int d = place->id;
    return d;
  }
  return -1;
}

void DisplayNavInPB::addDoorpost(double x, double y, double theta,
    double width, peekabot::SphereProxy &node) {
  peekabot::CubeProxy cpL;
  cpL.add(node, "doorpostleft");
  cpL.set_scale(0.1, 0.1, 2.0);
  cpL.set_pose(0.5 * width * cos(theta), 0.5 * width * sin(theta), 1.0, theta,
      0, 0);
  cpL.set_color(1.0, 0.817, 0.269);

  peekabot::CubeProxy cpR;
  cpR.add(node, "doorpostright");
  cpR.set_scale(0.1, 0.1, 2.0);
  cpR.set_pose(0.5 * width * cos(theta + M_PI),
      0.5 * width * sin(theta + M_PI), 1.0, theta, 0, 0);
  cpR.set_color(1.0, 0.817, 0.269);

  peekabot::CubeProxy cpT;
  cpT.add(node, "doorposttop");
  cpT.set_scale(width + 0.1, 0.1, 0.1);
  cpT.set_pose(0, 0, 2.0, theta, 0, 0);
  cpT.set_color(1.0, 0.817, 0.269);
}

void DisplayNavInPB::newConnectivityPathProperty(
    const cdl::WorkingMemoryChange &objID) {
  log("Entered newConnectivityPathProperty");

  if (!m_PeekabotClient.is_connected()) {
    log("Exited newConnectivityPathProperty");
    return;
  }

  shared_ptr<CASTData<SpatialProperties::ConnectivityPathProperty> > oobj =
      getWorkingMemoryEntry<SpatialProperties::ConnectivityPathProperty> (
          objID.address);

  SpatialProperties::ConnectivityPathPropertyPtr cpp = oobj->getData();
  _cpp[objID.address.id] = std::make_pair(cpp->place1Id, cpp->place2Id);

  displayConnectivityPathProperty(cpp->place1Id, cpp->place2Id);

  log("Exited newConnectivityPathProperty");
}
void DisplayNavInPB::displayConnectivityPathProperty(int place1Id, int place2Id) {
  FrontierInterface::PlaceInterfacePrx piPrx(getIceServer<
      FrontierInterface::PlaceInterface> ("place.manager"));
  double fx, fy, tx, ty;
  NavData::FNodePtr fnodePtr;
  fnodePtr = piPrx->getNodeFromPlaceID(place1Id);
  if (!fnodePtr) {
    //Placeholder
    SpatialData::NodeHypothesisPtr nodeHypPtr = piPrx->getHypFromPlaceID(
        place1Id);
    if (!nodeHypPtr) {
      log("Hyp doesnt exist\nExiting.\n");
      return;
    }
    fx = nodeHypPtr->x;
    fy = nodeHypPtr->y;
  } else {
    //Place
    fx = fnodePtr->x;
    fy = fnodePtr->y;
  }

  fnodePtr = piPrx->getNodeFromPlaceID(place2Id);
  if (!fnodePtr) {
    //Placeholder
    SpatialData::NodeHypothesisPtr nodeHypPtr = piPrx->getHypFromPlaceID(
        place2Id);
    if (!nodeHypPtr) {
      log("Hyp doesnt exist\nExiting.\n");
      return;
    }
    tx = nodeHypPtr->x;
    ty = nodeHypPtr->y;
  } else {
    //Place
    tx = fnodePtr->x;
    ty = fnodePtr->y;
  }
  double angle = atan2(ty - fy, tx - fx);
  debug("lp %f %f %f %f %f %f", fx, fy, tx, ty, tx - 0.05 * cos(angle - M_PI
      / 6), ty - 0.05 * sin(angle - M_PI / 6));
  char name[32];
  sprintf(name, "cpp%06d", place1Id * 1000 + place2Id);
  peekabot::LineCloudProxy lp;
  peekabot::VertexSet vs;
  lp.add(m_ProxyConnectivityPathProperties, name, peekabot::REPLACE_ON_CONFLICT);
  vs.add(fx, fy, 0);
  vs.add(tx, ty, 0);
  vs.add(tx, ty, 0);
  vs.add(tx - 0.3 * cos(angle - M_PI / 6), ty - 0.3 * sin(angle - M_PI / 6), 0);

  lp.add_vertices(vs);
  lp.set_color(1., 0., 0.);
  lp.set_opacity(1);
  lp.set_line_width(3);

}
void DisplayNavInPB::deleteConnectivityPathProperty(
    const cdl::WorkingMemoryChange &objID) {
  log("Entered deleteConnectivityPathProperty");

  if (!m_PeekabotClient.is_connected()) {
    log("Exited deleteConnectivityPathProperty");
    return;
  }

  map<string, std::pair<long, long> >::iterator it =
      _cpp.find(objID.address.id);
  if (it == _cpp.end()) {
    debug("Exited deleteConnectivityPathProperty");
    return;
  }

  char name[32];
  sprintf(name, "cpp%06ld", it->second.first * 1000 + it->second.second);
  debug("3 %s", name);
  peekabot::LineCloudProxy lp;
  peekabot::Status s =
      lp.assign(m_ProxyConnectivityPathProperties, name).status();
  if (s.succeeded()) {
    debug("4 %s", name);
    lp.remove();
  }
  _cpp.erase(it);

  log("Exited deleteConnectivityPathProperty");
}

void DisplayNavInPB::newNavGraphEdge(const cdl::WorkingMemoryChange &objID) {
  log("Entered newNavGraphEdge");

  if (!m_PeekabotClient.is_connected()) {
    debug("Exited newNavGraphEdge");
    return;
  }

  shared_ptr<CASTData<NavData::AEdge> > oobj = getWorkingMemoryEntry<
      NavData::AEdge> (objID.address);

  NavData::AEdgePtr aedge = oobj->getData();

  IceUtil::Mutex::Lock lock(m_Mutex);

  // Check if the nodes are there now
  std::map<long, Node>::iterator n1 = m_Nodes.find(aedge->startNodeId);
  std::map<long, Node>::iterator n2 = m_Nodes.find(aedge->endNodeId);

  if (n1 != m_Nodes.end() && n2 != m_Nodes.end()) { // Found nodes

    displayEdge(n1->second, n2->second);

    addEdgeToList(n1->second.m_Id, n2->second.m_Id);

  } else {
    m_NewEdges.push_back(std::make_pair(aedge->startNodeId, aedge->endNodeId));
  }

  debug("Got a new edge connecting nodes %d and %d", aedge->startNodeId,
      aedge->endNodeId);

  debug("Exited newNavGraphEdge");
}

void DisplayNavInPB::getColorByIndex(int id, float &r, float &g, float &b) {
  switch (id) {
  case 0: //Vivid Yellow
    r = 1.0 / 0xFF * 0xFF;
    g = 1.0 / 0xFF * 0xB3;
    b = 1.0 / 0xFF * 0x00;
    break;
  case 1: //Strong Purple
    r = 1.0 / 0xFF * 0x80;
    g = 1.0 / 0xFF * 0x3E;
    b = 1.0 / 0xFF * 0x75;
    break;
  case 2: //Vivid Orange
    r = 1.0 / 0xFF * 0xFF;
    g = 1.0 / 0xFF * 0x68;
    b = 1.0 / 0xFF * 0x00;
    break;
  case 3: //Very Light Blue
    r = 1.0 / 0xFF * 0xA6;
    g = 1.0 / 0xFF * 0xBD;
    b = 1.0 / 0xFF * 0xD7;
    break;
  case 4://Vivid Red
    r = 1.0 / 0xFF * 0xC1;
    g = 1.0 / 0xFF * 0x00;
    b = 1.0 / 0xFF * 0x20;
    break;
  case 5: //Grayish Yellow
    r = 1.0 / 0xFF * 0xCE;
    g = 1.0 / 0xFF * 0xA2;
    b = 1.0 / 0xFF * 0x62;
    break;
  case 6: //Vivid Green
    r = 1.0 / 0xFF * 0x00;
    g = 1.0 / 0xFF * 0x7D;
    b = 1.0 / 0xFF * 0x34;
    break;
  case 7: //Strong Purplish Pink
    r = 1.0 / 0xFF * 0xF6;
    g = 1.0 / 0xFF * 0x76;
    b = 1.0 / 0xFF * 0x8E;
    break;
  case 8: //Strong Blue
    r = 1.0 / 0xFF * 0x00;
    g = 1.0 / 0xFF * 0x53;
    b = 1.0 / 0xFF * 0x8A;
    break;
  case 9: //Strong Yellowish Pink
    r = 1.0 / 0xFF * 0xFF;
    g = 1.0 / 0xFF * 0x7A;
    b = 1.0 / 0xFF * 0x5C;
    break;
  case 10: //Strong Violet
    r = 1.0 / 0xFF * 0x53;
    g = 1.0 / 0xFF * 0x37;
    b = 1.0 / 0xFF * 0x7A;
    break;
  case 11: //Vivid Orange Yellow
    r = 1.0 / 0xFF * 0xFF;
    g = 1.0 / 0xFF * 0x8E;
    b = 1.0 / 0xFF * 0x00;
    break;
  case 12://Strong Purplish Red
    r = 1.0 / 0xFF * 0xB3;
    g = 1.0 / 0xFF * 0x28;
    b = 1.0 / 0xFF * 0x51;
    break;
  case 13: //Vivid Greenish Yellow
    r = 1.0 / 0xFF * 0xF4;
    g = 1.0 / 0xFF * 0xC8;
    b = 1.0 / 0xFF * 0x00;
    break;
  case 14: //Strong Reddish Brown
    r = 1.0 / 0xFF * 0x7F;
    g = 1.0 / 0xFF * 0x18;
    b = 1.0 / 0xFF * 0x0D;
    break;
  case 15: //Vivid Yellowish Green
    r = 1.0 / 0xFF * 0x93;
    g = 1.0 / 0xFF * 0xAA;
    b = 1.0 / 0xFF * 0x00;
    break;
  case 16://Deep Yellowish Brown
    r = 1.0 / 0xFF * 0x59;
    g = 1.0 / 0xFF * 0x33;
    b = 1.0 / 0xFF * 0x15;
    break;
  case 17: //Vivid Reddish Orange
    r = 1.0 / 0xFF * 0xF1;
    g = 1.0 / 0xFF * 0x3A;
    b = 1.0 / 0xFF * 0x13;
    break;
  case 18: //Dark Olive Green
    r = 1.0 / 0xFF * 0x23;
    g = 1.0 / 0xFF * 0x2C;
    b = 1.0 / 0xFF * 0x16;
    break;
  default: // If more colours are added change
    if (id < 0) {
      r = 0;
      g = 0;
      b = 0;
    } else {
      const int MAX_COLORS = 18;
      const int useColor = id % MAX_COLORS;
      //		debug("Only handles color with indices 0-%d, not %d, reusing colour %d", MAX_COLORS, id, useColor);
      getColorByIndex(useColor, r, g, b);
    }
  }
}

void DisplayNavInPB::addEdgeToList(long id1, long id2) {
  if (id1 > id2) {
    m_Edges.push_back(std::make_pair(id1, id2));
  } else if (id2 > id1) {
    m_Edges.push_back(std::make_pair(id2, id1));
  } else {
    log("WARNING: Trying to connect node with id %d with itself", id1);
  }
}

void DisplayNavInPB::displayEdge(const DisplayNavInPB::Node &node1,
    const DisplayNavInPB::Node &node2) {
  peekabot::LineCloudProxy lp;
  char name[32];
  if (node1.m_Id > node2.m_Id) {
    sprintf(name, "edge%06ld", node1.m_Id * 1000 + node2.m_Id);
  } else {
    sprintf(name, "edge%06ld", node2.m_Id * 1000 + node1.m_Id);
  }

  debug("Adding edge between (%f,%f) %ld and (%f,%f) %ld", node1.m_X,
      node1.m_Y, node1.m_Id, node2.m_X, node2.m_Y, node2.m_Id);

  peekabot::VertexSet vs;
  lp.add(m_ProxyEdges, name, peekabot::REPLACE_ON_CONFLICT);
  vs.add(node1.m_X, node1.m_Y, 0);
  vs.add(node2.m_X, node2.m_Y, 0);
  lp.add_vertices(vs);
  lp.set_color(0., 0., 0.);
  lp.set_opacity(1);
}

void DisplayNavInPB::redisplayEdgesToNode(const DisplayNavInPB::Node &node) {
  std::map<long, Node>::iterator n;

  for (std::list<std::pair<long, long> >::iterator e = m_Edges.begin(); e
      != m_Edges.end(); e++) {
    if (e->first == node.m_Id) {
      n = m_Nodes.find(e->second);
      if (n != m_Nodes.end()) {
        displayEdge(node, n->second);
        log("Redisplaying edge between nodes %ld and %ld", node.m_Id, e->second);
      } else {
        log("WARNING: Did not find node %ld, couldn't redisplay edge from %ld",
            e->second, node.m_Id);
      }
    } else if (e->second == node.m_Id) {
      n = m_Nodes.find(e->first);
      if (n != m_Nodes.end()) {
        displayEdge(node, n->second);
        log("Redisplaying edge between nodes %ld and %ld", node.m_Id, e->first);
      } else {
        log("WARNING: Did not find node %ld, couldn't redisplay edge from %ld",
            e->first, node.m_Id);
      }
    }
  }
}

void DisplayNavInPB::createSimpleRobotModel() {
  m_ProxyRobot.add(m_PeekabotClient, m_PbRobotName,
      peekabot::REPLACE_ON_CONFLICT);
  peekabot::CubeProxy cube;
  cube.add(m_ProxyRobot, "base", peekabot::REPLACE_ON_CONFLICT);
  cube.set_scale(0.4, 0.3, 0.2);
  cube.set_position(0, 0, 0.1);
  cube.set_color(0, 1, 0);

  peekabot::CubeProxy nose;
  nose.add(cube, "nose", peekabot::REPLACE_ON_CONFLICT);
  nose.set_scale(0.2, 0.05, 0.05);
  nose.set_position(0.1, 0, 0.125);
  nose.set_color(1, 0, 0);
}

void DisplayNavInPB::connectPeekabot() {
  try {
    log("Trying to connect to Peekabot (again?) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort);

    //   m_PeekabotClient.assign(m_PeekabotClient, "root");
    peekabot::Status s1, s2, s3, s4, s5;

    peekabot::Status s0 = m_ProxyRobot.add(m_PeekabotClient, m_PbRobotName,
        peekabot::REPLACE_ON_CONFLICT).status();
    if (s0.failed())
      log("failed to add robot");
    else
      log("added robot.");

    if (m_FollowRobot) {
      peekabot::Status s =
          m_DefCam.assign(m_PeekabotClient, "default_camera").status();
      if (!s.succeeded()) {
        error("Error: Could not find default_camera for --follow-robot!");
      }
    }

    if (m_LoadRobotFile) {
      log("Loading robot file \"%s\"", m_PbRobotFile.c_str());
    }

    if (m_ShowCommands) {
      m_ProxyViewpointGenCommands.add(m_PeekabotClient, "ViewpointCommands",
          peekabot::REPLACE_ON_CONFLICT);
      m_ProxyDetectionCommands.add(m_PeekabotClient, "DetectionCommands",
          peekabot::REPLACE_ON_CONFLICT);
    }

    peekabot::Status status_labels = m_ProxyLabels.add(m_PeekabotClient,
        "labels", peekabot::REPLACE_ON_CONFLICT).status();
    if (status_labels.failed()) {
      log("failed to add labels");
    }

    peekabot::LabelProxy mapStatusLabel;
    mapStatusLabel.add(m_PeekabotClient, "MapStatus",
        peekabot::REPLACE_ON_CONFLICT);
    mapStatusLabel.hide();

    if (m_LoadRobotFile) {

      s1 = m_ProxyRobot.load_scene(m_PbRobotFile).status();
      if (s1.failed()) {
        log("Could not load robot file \"%s\"", m_PbRobotFile.c_str());
        createSimpleRobotModel();
        m_ShowPanTilt = false;
      } else {
        log("loaded robot file \"%s\"", m_PbRobotFile.c_str());
        if (m_PbRobotFile == "CogXp3.xml" || m_PbRobotFile
            == "CogX_base_arm.xml" || m_PbRobotFile == "CogX_base.xml") {

          std::string path =
              "robot/chassis/superstructure/ptu/pan/tilt/baseline/cam_left";
          s4 = m_ProxyCam.assign(m_PeekabotClient, path).status();
          if (s4.failed()) {
            log("cam proxy failed.");
          }

          s4 = m_ProxyPan.assign(m_PeekabotClient,
              "robot/chassis/superstructure/ptu/pan").status();
          if (s4.failed()) {
            log("pan proxy failed.");
          }
          s4 = m_ProxyTilt.assign(m_PeekabotClient,
              "robot/chassis/superstructure/ptu/pan/tilt").status();
          if (s4.failed()) {
            log("tilt proxy failed.");
          }
          m_ProxyPan.set_dof(0);
          m_ProxyTilt.set_dof(30 * M_PI / 180.0);
        }
      }
    } else {
      log("Not loading a robot file - using simple shapes instead");
      createSimpleRobotModel();
    }

    peekabot::Status status_kinect = m_ProxyKinect.add(m_PeekabotClient,
        "kinect", peekabot::REPLACE_ON_CONFLICT).status();
    if (!status_kinect.succeeded()) {
      log("failed to add kinect");
    }
    m_ProxyKinect.set_max_vertices(10);
    m_ProxyKinect.set_vertex_overflow_policy(
        peekabot::VERTEX_OVERFLOW_TRUNCATE_HALF);

    if (m_ShowWalls) {
      m_ProxyWalls.add(m_PeekabotClient, "walls", peekabot::REPLACE_ON_CONFLICT);
    }

    m_ProxyGraph.add(m_PeekabotClient, "graph", peekabot::REPLACE_ON_CONFLICT);

    m_ProxyNodes.add(m_ProxyGraph, "nodes", peekabot::REPLACE_ON_CONFLICT);

    m_ProxyConnectivityPathProperties.add(m_ProxyGraph, "cpp",
        peekabot::REPLACE_ON_CONFLICT);
    if (!m_ShowConnectivities) {
      m_ProxyConnectivityPathProperties.hide();
    }
    m_ProxyConnectivityPathProperties.set_position(0, 0, 0.01);
    m_ProxyEdges.add(m_ProxyGraph, "edges", peekabot::REPLACE_ON_CONFLICT);

    m_ProxyObjects.add(m_ProxyGraph, "objects", peekabot::REPLACE_ON_CONFLICT);

    m_ProxyObjectLabels.add(m_ProxyGraph, "labels",
        peekabot::REPLACE_ON_CONFLICT);

    if (m_ShowPath) {
      m_ProxyTrajectory.add(m_PeekabotClient, "trajectory",
          peekabot::REPLACE_ON_CONFLICT);

      m_ProxyPathLog.add(m_ProxyTrajectory, "path_log",
          peekabot::AUTO_ENUMERATE_ON_CONFLICT);
      m_ProxyPathLog.set_line_width(5);
    }

    createRobotFOV();

    peekabot::Status client_status = m_PeekabotClient.noop().status();
    if (client_status.succeeded()) {
      log("All well with Peekabot");
    } else {
      error("Peekabot may have crashed");
    }

    if (m_PeekabotClient.is_connected()) {
      log("Connection to Peekabot established");
    } else {
      error("Connection to Peekabot has broken!");
    }

  } catch (std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)", e.what());
    return;
  }
}

Cure::Transformation3D DisplayNavInPB::getCameraToWorldTransform() {
  //Get camera ptz from PTZServer
  Cure::Transformation3D cameraRotation;
  if (m_PTUServer != 0) {
    ptz::PTZReading reading = m_PTUServer->getPose();

    double angles[] = { reading.pose.pan, -reading.pose.tilt, 0.0 };
    //    double angles[] = {0.0, M_PI/4, 0.0};
    cameraRotation.setAngles(angles);
  }
  //
  //  //Get camera position on robot from Cure
  //  m_CameraPoseR;

  //Additional transform of ptz base frame
  Cure::Transformation3D ptzBaseTransform;
  //  double nR[] = {0.0, 0.0, 1.0,
  //    1.0, 0.0, 0.0,
  //    0.0, 1.0, 0.0};
  double camAngles[] = { -M_PI / 2, 0.0, -M_PI / 2 };
  ptzBaseTransform.setAngles(camAngles);

  //Get robot pose
  Cure::Transformation2D robotTransform;
  if (m_RobotPose != 0) {
    robotTransform.setXYTheta(m_RobotPose->x, m_RobotPose->y,
        m_RobotPose->theta);
  }
  Cure::Transformation3D robotTransform3 = robotTransform;
  double tmp[6];
  robotTransform3.getCoordinates(tmp);
  log("robot transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);
  cameraRotation.getCoordinates(tmp);
  log("ptz transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);
  m_CameraPoseR.getCoordinates(tmp);
  log("cam transform: %f %f %f %f %f %f", tmp[0], tmp[1], tmp[2], tmp[3],
      tmp[4], tmp[5]);

  Cure::Transformation3D cameraOnRobot = m_CameraPoseR + cameraRotation
      + ptzBaseTransform;
  return robotTransform3 + cameraOnRobot;
}

void DisplayNavInPB::newViewpointGenCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("Entered newViewpointGenCommand");
  if (!m_PeekabotClient.is_connected() || !m_ShowCommands || !m_RobotPose) {
    debug("Exited newViewpointGenCommand");
    return;
  }

  SpatialData::RelationalViewPointGenerationCommandPtr obj;
  try {
    obj = getMemoryEntry<SpatialData::RelationalViewPointGenerationCommand> (
        objID.address);

    peekabot::PolygonProxy crossProxy;
    char buf[256];
    sprintf(buf, "Gen-%s-%s-%s", obj->searchedObjectCategory.c_str(),
        (obj->relation == SpatialData::ON) ? "on" : "in",
        obj->supportObject.c_str());
    crossProxy.add(m_ProxyViewpointGenCommands, buf,
        peekabot::AUTO_ENUMERATE_ON_CONFLICT);
    peekabot::VertexSet crossVerts;
    crossVerts.add(0.1, 0, 0.02);
    crossVerts.add(0.5, 0.4, 0.02);
    crossVerts.add(0.4, 0.5, 0.02);
    crossVerts.add(0, 0.1, 0.02);
    crossVerts.add(-0.4, 0.5, 0.02);
    crossVerts.add(-0.5, 0.4, 0.02);
    crossVerts.add(-0.1, 0, 0.02);
    crossVerts.add(-0.5, -0.4, 0.02);
    crossVerts.add(-0.4, -0.5, 0.02);
    crossVerts.add(0, -0.1, 0.02);
    crossVerts.add(0.4, -0.5, 0.02);
    crossVerts.add(0.5, -0.4, 0.02);
    crossProxy.add_vertices(crossVerts);
    crossProxy.set_position(m_RobotPose->x, m_RobotPose->y, 0);
    crossProxy.set_color(1.0, 0.5, 0);
  } catch (...) {
    log("Error! %s %i: disappeared from WM.", __FILE__, __LINE__);
    debug("Exited newViewpointGenCommand");
    return;
  }
  debug("Exited newViewpointGenCommand");
}

void DisplayNavInPB::newARTagCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  log("Entered newARTagCommand");
  if (!m_PeekabotClient.is_connected() || !m_RobotPose || !m_ShowCommands) {
    debug("Exited newARTagCommand");
    return;
  }

  VisionData::ARTagCommandPtr obj;
  try {
    obj = getMemoryEntry<VisionData::ARTagCommand> (objID.address);

    logDetectionCommand(obj->label);
  } catch (...) {
    log("Error! %s %i: disappeared from WM.", __FILE__, __LINE__);
    debug("Exited newARTagCommand");
    return;
  }
  debug("Exited newARTagCommand");
}

void DisplayNavInPB::newRecognizerCommand(
    const cast::cdl::WorkingMemoryChange &objID) {
  debug("Entered newRecognizerCommand");
  if (!m_PeekabotClient.is_connected() || !m_RobotPose || !m_ShowCommands) {
    debug("Exited newRecognizerCommand");
    return;
  }

  VisionData::Recognizer3DCommandPtr obj;
  try {
    obj = getMemoryEntry<VisionData::Recognizer3DCommand> (objID.address);

    logDetectionCommand(obj->label);
  } catch (...) {
    log("Error! %s %i: disappeared from WM.", __FILE__, __LINE__);
    debug("Exited newRecognizerCommand");
    return;
  }
  debug("Exited newRecognizerCommand");
}

void DisplayNavInPB::logDetectionCommand(const string &label) {
  peekabot::PolygonProxy viewProxy;
  char buf[256];
  sprintf(buf, "Detect-%s", label.c_str());
  viewProxy.add(m_ProxyDetectionCommands, buf,
      peekabot::AUTO_ENUMERATE_ON_CONFLICT);
  peekabot::VertexSet viewVerts;
  viewVerts.add(0, 0, 0.02);
  const double radius = 0.5;
  double step = M_PI / 32;
  for (double angle = -M_PI / 4; angle <= M_PI / 4; angle += step) {
    viewVerts.add(radius * cos(angle), radius * sin(angle), 0.02);
  }
  viewProxy.add_vertices(viewVerts);
  viewProxy.set_position(m_RobotPose->x, m_RobotPose->y, 0);
  viewProxy.set_rotation(m_RobotPose->theta, 0, 0);
  viewProxy.set_color(1.0, 0, 0);
}

void DisplayNavInPB::mapLoadStatusChanged(const cdl::WorkingMemoryChange &wmc) {
  log("mapLoadStatusChanged");
  try {
    if (wmc.operation == cdl::DELETE) {
      peekabot::LabelProxy mapStatusLabel;
      mapStatusLabel.assign(m_PeekabotClient, "MapStatus");
      mapStatusLabel.hide();
    } else {
      const SpatialData::MapLoadStatusPtr statusStruct = getMemoryEntry<
          SpatialData::MapLoadStatus> (wmc.address.id);
      peekabot::LabelProxy mapStatusLabel;
      mapStatusLabel.add(m_PeekabotClient, "MapStatus",
          peekabot::REPLACE_ON_CONFLICT);
      mapStatusLabel.show();
      mapStatusLabel.set_scale(10, 10, 10);
      mapStatusLabel.set_position(0, 0, 3);
      std::string labelString("Loading:");
      if (statusStruct->nodesWritten) {
        labelString = labelString + " nodes";
      }
      if (statusStruct->placesWritten) {
        labelString = labelString + " places";
      }
      if (statusStruct->roomsWritten) {
        labelString = labelString + " rooms";
      }
      if (statusStruct->categoryDataWritten) {
        labelString = labelString + " categoryData";
      }
      if (statusStruct->obstacleMapsLoaded) {
        labelString = labelString + " obstacles";
      }
      if (statusStruct->categoryDataWritten) {
        labelString = labelString + " localMaps";
      }
      mapStatusLabel.set_text(labelString.c_str());
    }
  } catch (DoesNotExistOnWMException) {
    log("MapLoadStatus struct disappeared from WM!");
  }
}

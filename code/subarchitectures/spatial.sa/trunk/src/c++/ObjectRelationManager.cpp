//
// = FILENAME
//    ObjectRelationManager.cpp
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

#include "ObjectRelationManager.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <VisionData.hpp>
#include <AddressBank/ConfigFileReader.hh>

using namespace std;
using namespace cast;
using namespace boost;
using namespace spatial;


/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new ObjectRelationManager();
  }
}

ObjectRelationManager::ObjectRelationManager()
{
  m_maxObjectCounter = 0;
  m_bDisplayPlaneObjectsInPB = false;
}

ObjectRelationManager::~ObjectRelationManager() 
{ 

}



void ObjectRelationManager::configure(const map<string,string>& _config) 
{
  map<string,string>::const_iterator it = 
    _config.find("--display-planes-in-pb");
  if (it != _config.end()) {
    m_bDisplayPlaneObjectsInPB = true;
  }

  m_RetryDelay = 10;

  Cure::ConfigFileReader *cfg = 0;
  it = _config.find("-c");
  if (it != _config.end()) {
    cfg = new Cure::ConfigFileReader;
    log("About to try to open the config file");
    if (cfg->init(it->second) != 0) {
      delete cfg;
      cfg = 0;
      log("Could not init Cure::ConfigFileReader with -c argument");
    } else {
      log("Managed to open the Cure config file");
    }
  }

  m_PbPort = 5050;
  m_PbHost = "localhost";

  if (cfg) {
    std::string usedCfgFile, tmp;
    if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
      m_PbHost = tmp;
    }
  }
} 

void ObjectRelationManager::start() 
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::newObject));  

  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE), 
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::objectChanged));  

  addChangeFilter(createLocalTypeFilter<FrontierInterface::ObservedPlaneObject>
      (cdl::ADD), 
      new MemberFunctionChangeReceiver<ObjectRelationManager>(this, 
	&ObjectRelationManager::newPlaneObject));  

  addChangeFilter(createLocalTypeFilter<FrontierInterface::ObservedPlaneObject>
      (cdl::OVERWRITE), 
      new MemberFunctionChangeReceiver<ObjectRelationManager>(this, 
	&ObjectRelationManager::newPlaneObject));  

  m_placeInterface = getIceServer<FrontierInterface::PlaceInterface>("place.manager");
  log("ObjectRelationManager started");
}

void ObjectRelationManager::runComponent() 
{
  log("I am running!");

  if (m_bDisplayPlaneObjectsInPB) {
    while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
      sleep(m_RetryDelay);
      connectPeekabot();
    }

    peekabot::GroupProxy root;
    root.assign(m_PeekabotClient, "root");
    m_planeProxies.add(root, "plane_objects", peekabot::REPLACE_ON_CONFLICT);
  }

  println("Connected to peekabot, ready to go");
  
//  while(isRunning()){
//    usleep(250000);
//  }
}

void
ObjectRelationManager::newObject(const cast::cdl::WorkingMemoryChange &wmc)
{
  try {
    VisionData::VisualObjectPtr observedObject =
      getMemoryEntry<VisionData::VisualObject>(wmc.address);

    cogx::Math::Pose3 pose = observedObject->pose;

    SpatialData::SpatialObjectPtr newObject = new SpatialData::SpatialObject;
    newObject->label = observedObject->label;
    newObject->id = m_maxObjectCounter;
    m_maxObjectCounter++;

    m_objects[newObject->id] = newObject;

    // Check degree of containment in Places
    FrontierInterface::PlaceMembership membership = 
      m_placeInterface->getPlaceMembership(pose.pos.x, pose.pos.y);

    setContainmentProperty(newObject->id, membership.placeID, 1.0);

    //  Needs estimate of extent of Places, in sensory frame
    //   Add interface to PlaceManager?
    // Post one or more Place containment structs on WM
    // Find and remove/update extant property structs

    // Check degree of support between Objects
    //  Need a list of tracked Object
    //  Check contact points and estimated centers of gravity
    //   Need access to representation of object models
    //   
  }
  catch (DoesNotExistOnWMException) {
    log("Error! new Object missing on WM!");
  }
}

void 
ObjectRelationManager::objectChanged(const cast::cdl::WorkingMemoryChange &wmc)
{
  try {
    VisionData::VisualObjectPtr observedObject =
      getMemoryEntry<VisionData::VisualObject>(wmc.address);
  }
  catch (DoesNotExistOnWMException) {
    log("Error! overwritten Object missing on WM!");
  }
}

void 
ObjectRelationManager::newPlaneObject(const cast::cdl::WorkingMemoryChange &wmc)
{
  try {
    FrontierInterface::ObservedPlaneObjectPtr observedObject =
      getMemoryEntry<FrontierInterface::ObservedPlaneObject>
      (wmc.address);

    log("Read object at %f, %f, %f", observedObject->x, observedObject->y,
	observedObject->height);
    if (m_PeekabotClient.is_connected()) {
      char identifier[100];
      sprintf(identifier, "label%d", 0);
      peekabot::PolygonProxy pp;
      pp.add(m_planeProxies, identifier, peekabot::REPLACE_ON_CONFLICT);
      pp.add_vertex(0.55, 0.45, 0);
      pp.add_vertex(-0.55, 0.45, 0);
      pp.add_vertex(-0.55, -0.45, 0);
      pp.add_vertex(0.55, -0.45, 0);

      pp.translate(observedObject->x, observedObject->y, 
	  observedObject->height);
      pp.rotate(observedObject->angle, 0.0, 0.0, 1.0);
    }
  }
  catch (DoesNotExistOnWMException) {
    log("Error! overwritten Object missing on WM!");
  }
}

void
ObjectRelationManager::setContainmentProperty(int objectID, int placeID, double confidence)
{
  map<int, PlaceContainmentObjectPropertyPtr>::iterator it =
    m_containmentProperties.find(objectID);
  SpatialProperties::PlaceContainmentObjectPropertyPtr containmentProp;
  if (it == m_containmentProperties.end()) {
    //New containment property for this Object

    containmentProp =
      new SpatialProperties::PlaceContainmentObjectProperty;
    m_containmentProperties[objectID] = containmentProp;
  }
  else {
    containmentProp = it->second;
  }
  SpatialProperties::IntegerValuePtr placeValue1 = 
    new SpatialProperties::IntegerValue;
  SpatialProperties::IntegerValuePtr placeValue2 = 
    new SpatialProperties::IntegerValue;
  placeValue1->value = placeID;
  placeValue2->value = -1;

  SpatialProperties::ValueProbabilityPair pair1 =
  { placeValue1, confidence };
  SpatialProperties::ValueProbabilityPair pair2 =
  { placeValue2, 1.0-confidence };

  SpatialProperties::ValueProbabilityPairs pairs;
  pairs.push_back(pair1);
  pairs.push_back(pair2);

  SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
    new SpatialProperties::DiscreteProbabilityDistribution;
  discDistr->data = pairs;
  containmentProp->placeId = placeID;
  containmentProp->objectId = objectID;
  containmentProp->distribution = discDistr;
  containmentProp->mapValue = confidence > 0.5 ? placeValue1 : placeValue2;
  containmentProp->mapValueReliable = confidence > 0.8 || confidence < 0.2;

  //string newID = newDataID();
  //addToWorkingMemory<SpatialProperties::PlaceContainmentObjectProperty>(newID, containmentProp);
}

void 
ObjectRelationManager::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort, true);

  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}


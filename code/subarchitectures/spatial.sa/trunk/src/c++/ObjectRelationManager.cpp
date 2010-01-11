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
}

ObjectRelationManager::~ObjectRelationManager() 
{ 

}



void ObjectRelationManager::configure(const map<string,string>& _config) 
{
//  map<string,string>::const_iterator it = _config.find("-c");
//  if (it== _config.end()) {
//    println("configure(...) Need config file (use -c option)\n");
//    std::abort();
//  }
//  std::string configfile = it->second;
//
//  Cure::ConfigFileReader cfg;
//  if (cfg.init(configfile)) {
//    println("configure(...) Failed to open with \"%s\"\n",
//            configfile.c_str());
//    std::abort();
//  }  
//
//  if (Cure::NavController::config(configfile)) {
//    println("configure(...) Failed to config with \"%s\", use -c option\n",
//            configfile.c_str());
//    std::abort();
//  } 
//
//  if (cfg.getSensorPose(1, m_LaserPoseR)) {
//    println("configure(...) Failed to get sensor pose");
//    std::abort();
//  } 
//
//  m_MaxExplorationRange = 1.5;
//  it = _config.find("--explore-range");
//  if (it != _config.end()) {
//    m_MaxExplorationRange = (atof(it->second.c_str()));
//  }
//
//  it = _config.find("--robot-server-host");
//  if (it != _config.end()) {
//    std::istringstream str(it->second);
//    str >> m_RobotServerHost;
//  }
//
//  m_lgm = new Cure::LocalGridMap<unsigned char>(200, 0.1, '2', Cure::LocalGridMap<unsigned char>::MAP1);
//  m_Glrt  = new Cure::GridLineRayTracer<unsigned char>(*m_lgm);
//  m_Explorer = new Cure::FrontierExplorer(*this,*m_lgm);
//  //m_Explorer->setExplorationConfinedByGateways(true);
//  m_Explorer->addEventListener(this);
//
//
//  if (_config.find("--no-x-window") == _config.end()) {
//    m_Displaylgm = new Cure::XDisplayLocalGridMap<unsigned char>(*m_lgm);
//    println("Will use X window to show the exploration map");
//  } else {
//    m_Displaylgm = 0;
//    println("Will NOT use X window to show the exploration map");
//  }
//
//  double maxGotoV = 0.5;
//  double maxGotoW = 0.5;
//
//  if ((it = _config.find("--max-goto-v")) != _config.end()) {
//    std::istringstream str(it->second);
//    str >> maxGotoV;
//  }
//  if ((it = _config.find("--max-goto-w")) != _config.end()) {
//    std::istringstream str(it->second);
//    str >> maxGotoW;
//  }
//
//  Cure::NavController::addEventListener(this);
//  Cure::NavController::setTurnAngleIntoSpeed(true, 0.5);
//  Cure::NavController::setMinNonzeroSpeeds(0.04, 
//                                           Cure::HelpFunctions::deg2rad(10));
//  Cure::NavController::setApproachTolerances(0.5, 
//                                             Cure::HelpFunctions::deg2rad(10));
//  Cure::NavController::setUsePathTrimming(false);
//  Cure::NavController::setMaxPathTrimDist(3);
//  Cure::NavController::setProgressTimeout(10);
//  Cure::NavController::setGotoMaxSpeeds(maxGotoV, maxGotoW);
//  Cure::NavController::setGatewayMaxSpeeds(0.3, 0.3);
//  
//  Cure::NavController::setFollowDistances(0.8, 0.4);
//  Cure::NavController::setFollowTolerances(0.1, 
//                                           Cure::HelpFunctions::deg2rad(10));
//
//  Cure::NavController::setPoseProvider(m_TOPP);
//
//  /*
//  it = _config.find("--max-target-graph-dist");
//  double maxDist = 5;
//  if (it != _config.end()) {
//    std::istringstream str(it->second);
//    str >> maxDist;
//  }  
//  m_NavGraph.setMaxDistTargetFromNode(maxDist);
//  */
//
//  m_taskId = 1;
//  m_taskStatus = NothingToDo;
//  m_ready = false;
//  m_DefTolPos = 0.25;
//  m_DefTolRot = Cure::HelpFunctions::deg2rad(5);
//
//  m_RobotServer = RobotbaseClientUtils::getServerPrx(*this,
//                                                     m_RobotServerHost);
//
//  FrontierInterface::FrontierReaderPtr servant = new FrontierServer(this);
//  registerIceServer<FrontierInterface::FrontierReader, FrontierInterface::FrontierReader>(servant);
} 

void ObjectRelationManager::start() 
{
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD), 
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::newObject));  

  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE), 
		  new MemberFunctionChangeReceiver<ObjectRelationManager>(this,
								  &ObjectRelationManager::objectChanged));  

  m_placeInterface = getIceServer<FrontierInterface::PlaceInterface>("place.manager");
  log("ObjectRelationManager started");
}

void ObjectRelationManager::runComponent() 
{
  log("I am running!");
  
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

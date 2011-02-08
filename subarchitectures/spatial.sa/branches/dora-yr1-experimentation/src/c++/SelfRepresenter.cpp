#include "SelfRepresenter.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <Ice/Ice.h>
#include <sstream>
#include <float.h>
#include <FrontierInterface.hpp>
#include <sstream>

using namespace std;
using namespace cast;
using namespace binder::autogen::core;
using namespace spatial;

extern "C" {
  cast::interfaces::CASTComponentPtr newComponent() {
    return new SelfRepresenter();
  }
}

SelfRepresenter::SelfRepresenter()
{
}

/**
 * Destructor
 */
SelfRepresenter::~SelfRepresenter()
{
}

void
SelfRepresenter::start()
{
}

void
SelfRepresenter::stop()
{
}

NavData::FNodePtr
SelfRepresenter::getCurrentNavNode()
{
  vector<NavData::FNodePtr> nodes;
  getMemoryEntries<NavData::FNode>(nodes, 0);

  vector<NavData::RobotPose2dPtr> robotPoses;
  getMemoryEntries<NavData::RobotPose2d>(robotPoses, 0);

  if (robotPoses.size() == 0) {
    log("Could not find RobotPose!");
    return 0;
  }

  //Find the node closest to the robotPose
  double robotX = robotPoses[0]->x;
  double robotY = robotPoses[0]->y;
  double minDistance = FLT_MAX;
  NavData::FNodePtr ret = 0;

  for (vector<NavData::FNodePtr>::iterator it = nodes.begin();
      it != nodes.end(); it++) {
    double x = (*it)->x;
    double y = (*it)->y;

    double distance = (x - robotX)*(x-robotX) + (y-robotY)*(y-robotY);
    if (distance < minDistance) {
      ret = *it;
      minDistance = distance;
    }
  }
  return ret;
}

void
SelfRepresenter::runComponent()
{
  try {
    Marshalling::MarshallerPrx agg(getIceServer<Marshalling::Marshaller>("proxy.marshaller"));
    try {
      cast::cdl::WorkingMemoryPointerPtr origin = new cast::cdl::WorkingMemoryPointer();
      origin->address.subarchitecture = "spatial.sa";
      origin->address.id = "local";
      origin->type = "data"; //uh oh, do we always need to include this?

      agg->addProxy("robot", "1", 1.0, origin);

      FeaturePtr feature = new Feature();
      feature->featlabel = "category";
      feature->alternativeValues.push_back(new
					   binder::autogen::featvalues::StringValue(1, getCASTTime(), "robot"));
      agg->addFeature("robot", "1", feature);
      agg->commitFeatures("robot","1");
    }
    catch (Ice::Exception e) {
      log("Failed to add robot proxy!");
    }

    FrontierInterface::PlaceInterfacePrx agg2(getIceServer<FrontierInterface::PlaceInterface>("place.manager"));

    int prevPlaceID = -1;

    cast::cdl::WorkingMemoryPointerPtr origin = new cast::cdl::WorkingMemoryPointer();
    origin->address.subarchitecture = "spatial.sa";
    origin->address.id = "local";
    origin->type = "data"; //uh oh, do we always need to include this?

    while (isRunning()) {
      // Regularly check robot pose
      NavData::FNodePtr curFNode = getCurrentNavNode();
      if (curFNode==0) {
	usleep(500000);
	continue;
      }

      try {
	SpatialData::PlacePtr curPlace = agg2->getPlaceFromNodeID(curFNode->nodeId);

	int curPlaceID = curPlace->id;
	if (curPlaceID != prevPlaceID) {
	  // Place has changed!

	  // replace position feature
	  agg->deleteProxy("robotpos", "robot");

	  stringstream ss;
	  ss << curPlaceID;

	  agg->addRelation("robotpos", "robot",
	      "robot", "1",
	      "place", ss.str(), 1.0, origin);

	  // Add the "relationType" label feature
	  FeaturePtr feature = new Feature();
	  feature->featlabel = "located";

	  feature->alternativeValues.push_back(new
	      binder::autogen::featvalues::BooleanValue(1, getCASTTime(), true));
	  agg->addFeature("robotpos", "robot", feature);

	  // Foreground relation proxy
	  agg->commitFeatures("robotpos", "robot");
	}
	prevPlaceID = curPlaceID;
      }
      catch (Ice::Exception e) {
	log("Unable to get current Place!");
      }
      usleep(500000);
    }
  }
  catch (Ice::Exception e) {
    log("Unexpected ICE exception!");
    exit(1);
  }
}

void
SelfRepresenter::configure(const std::map<std::string, std::string>& _config)
{
}


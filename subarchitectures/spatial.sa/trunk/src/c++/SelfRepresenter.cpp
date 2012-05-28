#include "SelfRepresenter.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <Ice/Ice.h>
#include <sstream>
#include <float.h>
#include <FrontierInterface.hpp>
#include <sstream>

using namespace std;
using namespace cast;
using namespace spatial;

extern "C" {
cast::interfaces::CASTComponentPtr newComponent() {
	return new SelfRepresenter();
}
}

SelfRepresenter::SelfRepresenter() {
}

/**
 * Destructor
 */
SelfRepresenter::~SelfRepresenter() {
}

void SelfRepresenter::start() {
}

void SelfRepresenter::stop() {
}

NavData::FNodePtr SelfRepresenter::getCurrentNavNode() {
	vector<NavData::FNodePtr> nodes;
	getMemoryEntries<NavData::FNode> (nodes, 0);

	vector<NavData::RobotPose2dPtr> robotPoses;
	getMemoryEntries<NavData::RobotPose2d> (robotPoses, 0);

	if (robotPoses.size() == 0) {
		log("Could not find RobotPose!");
		return 0;
	}

	//Find the node closest to the robotPose
	double robotX = robotPoses[0]->x;
	double robotY = robotPoses[0]->y;
	double minDistance = FLT_MAX;
	NavData::FNodePtr ret = 0;

	for (vector<NavData::FNodePtr>::iterator it = nodes.begin(); it
			!= nodes.end(); it++) {
		double x = (*it)->x;
		double y = (*it)->y;

		double distance = (x - robotX) * (x - robotX) + (y - robotY) * (y - robotY);
		if (distance < minDistance) {
			ret = *it;
			minDistance = distance;
		}
	}
	return ret;
}

void SelfRepresenter::runComponent() {
	string robotPositionPropertyWMID = "";

	FrontierInterface::PlaceInterfacePrx agg2(getIceServer<
			FrontierInterface::PlaceInterface> ("place.manager"));

	int prevPlaceID = -1;

	cast::cdl::WorkingMemoryPointerPtr origin =
			new cast::cdl::WorkingMemoryPointer();
	origin->address.subarchitecture = "spatial.sa";
	origin->address.id = "local";
	origin->type = "data"; //uh oh, do we always need to include this?

	while (isRunning()) {
		// Regularly check robot pose
		NavData::FNodePtr curFNode = getCurrentNavNode();
		if (curFNode == 0) {
			usleep(100000);
			log("have to wait for novnodes to show up");
			continue;
		}
		try {
			SpatialData::PlacePtr curPlace = agg2->getPlaceFromNodeID(
					curFNode->nodeId);

			if (curPlace != 0) {
				int curPlaceID = curPlace->id;
				if (curPlaceID != prevPlaceID) {
					// Place has changed!
					log("Robot is now at Place %i", curPlaceID);

					SpatialProperties::IntegerValuePtr placeIDValue =
							new SpatialProperties::IntegerValue;
					placeIDValue->value = curPlaceID;

					SpatialProperties::ValueProbabilityPair pair1 = { placeIDValue, 1.0 };

					SpatialProperties::ValueProbabilityPairs pairs;
					pairs.push_back(pair1);

					SpatialProperties::DiscreteProbabilityDistributionPtr discDistr =
							new SpatialProperties::DiscreteProbabilityDistribution;
					discDistr->data = pairs;

					SpatialProperties::PlaceContainmentAgentPropertyPtr
							robotLocationProperty =
									new SpatialProperties::PlaceContainmentAgentProperty();
					robotLocationProperty->agentID = 0; //Robot's ID
					robotLocationProperty->distribution = discDistr;
					robotLocationProperty->mapValue = placeIDValue;
					robotLocationProperty->mapValueReliable = 1;

					if (robotPositionPropertyWMID == "") {
						robotPositionPropertyWMID = newDataID();
						addToWorkingMemory<SpatialProperties::PlaceContainmentAgentProperty> (
								robotPositionPropertyWMID, robotLocationProperty);
					} else {
						overwriteWorkingMemory<
								SpatialProperties::PlaceContainmentAgentProperty> (
								robotPositionPropertyWMID, robotLocationProperty);
					}
				}
				prevPlaceID = curPlaceID;
			}
		} catch (Ice::Exception e) {
			log("Unable to get current Place! %s", e.what());
		}
		usleep(500000);
	}
}

void SelfRepresenter::configure(
		const std::map<std::string, std::string>& _config) {
}


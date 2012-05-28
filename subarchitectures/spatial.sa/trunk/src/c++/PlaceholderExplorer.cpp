#include "PlaceholderExplorer.hpp"

#include <NavData.hpp>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <queue>
#include <set>
#include <math.h>

using namespace navsa;

extern "C" {
cast::interfaces::CASTComponentPtr newComponent() {
	return new PlaceholderExplorer();
}
}

PlaceholderExplorer::PlaceholderExplorer() {
	m_status = NavData::SUCCEEDED;
	m_hasPosition = false;
}

void PlaceholderExplorer::configure(
		const std::map<std::string, std::string>& config) {
}

void PlaceholderExplorer::start() {
	addChangeFilter(cast::createLocalTypeFilter<NavData::RobotPose2d>(
			cast::cdl::OVERWRITE), new cast::MemberFunctionChangeReceiver<
			PlaceholderExplorer>(this, &PlaceholderExplorer::poseChange));

	addChangeFilter(cast::createLocalTypeFilter<SpatialData::NavCommand>(
			cast::cdl::OVERWRITE), new cast::MemberFunctionChangeReceiver<
			PlaceholderExplorer>(this, &PlaceholderExplorer::navCommandResponse));
}

void PlaceholderExplorer::runComponent() {
	FrontierInterface::PlaceInterfacePrx placeInterface(getIceServer<
			FrontierInterface::PlaceInterface> ("place.manager"));
	int placeId = -1;

	while (true) {
		std::set<int> updatedPlaceholders;

		if (m_status == SpatialData::COMMANDINPROGRESS || m_status
				== SpatialData::COMMANDPENDING || !m_hasPosition) {
			sleep(1);
			continue;
		}

		SpatialData::PlacePtr currentPlace = placeInterface->getCurrentPlace();
		while (true) {

			placeId = findClosestPlaceholderInNodeGraph(currentPlace->id);
			if (placeId < 0) {
				/* Did not find a placeholder. Sleep and try again. */
				sleep(1);
				break;
			}

			// Have we updated this placeholders edge before?
			if (updatedPlaceholders.find(placeId) == updatedPlaceholders.end()) {
				log("We have not updated this placeholder before");
				int res = placeInterface->updatePlaceholderEdge(placeId);
				updatedPlaceholders.insert(placeId);
				log("Placeholder %d is now connected to %d", placeId, res);
				continue;
			} else {
				log("We have updated this placeholder before!");
				int placeConnectedToPlaceholder;
				try {
					placeConnectedToPlaceholder = placeInterface->getHypFromPlaceID(
							placeId)->originPlaceID;
				} catch (IceUtil::NullHandleException &e) {
					log("Couldnt get hyp from placeid. Place is probably deleted.");
					break;
				}

				// Are we already at the closest node?
				if (placeConnectedToPlaceholder == currentPlace->id)
					goToPlace(placeId);
				else
					goToPlace(placeConnectedToPlaceholder);

				log("We've sent a movement command.");
				break;
			}
		}
	}
}

int PlaceholderExplorer::findClosestPlaceholderInNodeGraph(int curPlaceId) {
	debug("FindclosestPlaceholderInNodeGraph entered\n");
	std::queue<int> toSearch;
	std::set<int> visited;
	std::set<int> placeholders;
	std::map<int, std::vector<int> > adjacencyLists;

	FrontierInterface::PlaceInterfacePrx placeInterface(getIceServer<
			FrontierInterface::PlaceInterface> ("place.manager"));

	// Get the graph
	log("Getting adjacencylists\n");
	adjacencyLists = placeInterface->getAdjacencyLists();

	// Build a set of all the placeholders
	log("building a set of all the placeholders\n");
	std::vector<SpatialData::PlacePtr> places;
	getMemoryEntries<SpatialData::Place> (places);
	for (std::vector<SpatialData::PlacePtr>::iterator placesIt = places.begin(); placesIt
			!= places.end(); ++placesIt) {

		try {
			SpatialData::PlacePtr place = *placesIt;
			if (place->status == SpatialData::PLACEHOLDER)
				placeholders.insert(place->id);
		} catch (IceUtil::NullHandleException e) {
			log("Place disappeared before we got to it. Ignoring.");
		}
	}

	// Add the places connected to the current node to the queue
	log("add places connected to the current node to the queue");
	std::map<int, std::vector<int> >::iterator adjIt = adjacencyLists.find(
			curPlaceId);
	if (adjIt == adjacencyLists.end()) {
		log("Current place does not have any edges.\n");

		// Print the graph. For debugging purposes.
		for (std::map<int, std::vector<int> >::iterator it = adjacencyLists.begin(); it
				!= adjacencyLists.end(); ++it) {
			log("Place %d has the following edges:\n", it->first);
			for (vector<int>::iterator it2 = it->second.begin(); it2
					!= it->second.end(); ++it2) {
				log("  %d", *it2);
			}
		}
		return -1;
	}

	std::vector<int> &curPlaceConnections = adjIt->second;
	for (std::vector<int>::iterator connectionsIt = curPlaceConnections.begin(); connectionsIt
			!= curPlaceConnections.end(); ++connectionsIt) {

		// If this is a placeholder, we're done
		if (placeholders.find(*connectionsIt) != placeholders.end()) {
			debug("exited function");
			return *connectionsIt;
		}

		toSearch.push(*connectionsIt);
	}
	visited.insert(curPlaceId);

	// Do a BFS over the graph
	log("Do the bfs");
	while (!toSearch.empty()) {
		int placeId = toSearch.front();
		toSearch.pop();

		// Otherwise, add all of the places this place is connected to
		adjIt = adjacencyLists.find(placeId);
		if (adjIt == adjacencyLists.end())
			continue; // No connections from this place.

		std::vector<int> &placeConnections = adjIt->second;
		for (std::vector<int>::iterator connectionsIt = placeConnections.begin(); connectionsIt
				!= placeConnections.end(); ++connectionsIt) {

			// If this is a placeholder, we're done
			if (placeholders.find(*connectionsIt) != placeholders.end())
				return *connectionsIt;

			// Don't add places we've already searched
			if (visited.find(*connectionsIt) != visited.end()) {
				log("exited function");
				continue;
			}

			toSearch.push(*connectionsIt);
		}

		// Mark this place as visited
		visited.insert(placeId);
	}

	// We did not find a placeholder.
	// Print the graph. For debugging purposes.
	log("Did not find a placeholder in graph.");
	for (std::map<int, std::vector<int> >::iterator it = adjacencyLists.begin(); it
			!= adjacencyLists.end(); ++it) {
		log("Place %d has the following edges:", it->first);
		for (vector<int>::iterator it2 = it->second.begin(); it2
				!= it->second.end(); ++it2) {
			log("  %d", *it2);
		}
	}
	log("exited function");
	return -1;
}

void PlaceholderExplorer::goToPlace(int placeId) {
	log("Trying to go to place id: %d", placeId);

	SpatialData::NavCommandPtr navCmd = new SpatialData::NavCommand;
	navCmd->prio = SpatialData::NORMAL;
	navCmd->cmd = SpatialData::GOTOPLACE;
	navCmd->destId.push_back(placeId);
	navCmd->comp = SpatialData::COMMANDPENDING;
	navCmd->status = SpatialData::NONE;
	/* Lower the default tolerance so we don't get as much rejected hypotheses */
	//navCmd->tolerance.push_back(0.1);

	m_status = SpatialData::COMMANDINPROGRESS;
	std::string navCmdID = newDataID();
	addToWorkingMemory<SpatialData::NavCommand> (navCmdID, navCmd);
}

void PlaceholderExplorer::poseChange(
		const cast::cdl::WorkingMemoryChange &objID) {
	try {
		NavData::RobotPose2dPtr oobj = getMemoryEntry<NavData::RobotPose2d> (
				objID.address);

		m_x = oobj->x;
		m_y = oobj->y;
		m_theta = oobj->theta;

		m_hasPosition = true;
	} catch (IceUtil::NullHandleException e) {
	}
}

void PlaceholderExplorer::navCommandResponse(
		const cast::cdl::WorkingMemoryChange &objID) {
	try {
		SpatialData::NavCommandPtr oobj = getMemoryEntry<SpatialData::NavCommand> (
				objID.address);

		m_status = oobj->comp;

		switch (oobj->comp) {
		case 0:
			log("Navigation command status: PENDING");
			break;
		case 1:
			log("Navigation command status: INPROGRESS");
			break;
		case 2:
			log("Navigation command status: ABORTED");
			break;
		case 3:
			log("Navigation command status: FAILED");
			break;
		case 4:
			log("Navigation command status: SUCCEEDED");
			break;
		}
	} catch (IceUtil::NullHandleException e) {
	}
}


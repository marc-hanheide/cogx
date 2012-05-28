//
// = Filename
//   ExplorationTester.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

#include "ExplorationTester.hpp"
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <FrontierInterface.hpp>
#include <SpatialData.hpp>
#include <Rendezvous.h>
#include <float.h>

using namespace cast;
using namespace std;
using namespace Ice;
using namespace spatial;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
cast::interfaces::CASTComponentPtr newComponent() {
	return new ExplorationTester();
}
}

ExplorationTester::ExplorationTester() {
}

ExplorationTester::~ExplorationTester() {
}

void ExplorationTester::configure(const map<string, string>& _config) {
	log("Configure entered");
}

void ExplorationTester::start() {
	addChangeFilter(createLocalTypeFilter<SpatialData::Place> (cdl::ADD),
			new MemberFunctionChangeReceiver<ExplorationTester> (this,
					&ExplorationTester::newPlace));
	addChangeFilter(createLocalTypeFilter<SpatialData::Place> (cdl::DELETE),
			new MemberFunctionChangeReceiver<ExplorationTester> (this,
					&ExplorationTester::placeDeleted));
	addChangeFilter(createLocalTypeFilter<SpatialData::Place> (cdl::OVERWRITE),
			new MemberFunctionChangeReceiver<ExplorationTester> (this,
					&ExplorationTester::PlaceChanged));
}

void ExplorationTester::stop() {
}

NavData::FNodePtr ExplorationTester::getCurrentNavNode() {
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

void ExplorationTester::runComponent() {
	Rendezvous *rv = new Rendezvous(*this);

	rv->addChangeFilter(createLocalTypeFilter<NavData::RobotPose2d> (cdl::ADD)); // local
	rv->wait();

	while (isRunning()) {
		log("I am running");

		FrontierInterface::PlaceInterfacePrx agg(getIceServer<
				FrontierInterface::PlaceInterface> ("place.manager"));
		NavData::FNodePtr currentNavNode = getCurrentNavNode();
		if (!currentNavNode) {
			sleepComponent(1000);
			continue;
		}
		int currNodeID = currentNavNode->nodeId;
		//log("currNodeID = %i", currNodeID);
		SpatialData::PlacePtr currPlace = agg->getPlaceFromNodeID(currNodeID);
		if (!currPlace) {
			sleepComponent(1000);
			continue;
		}
		unsigned int currPlaceID = currPlace->id;
		//log("currPlaceID = %i", currPlaceID);
		//
		//      debug("Locking 1");
		//      lockComponent(); //Protect m_newPlaces while reading it
		//      if (!m_newPlaces.empty()) {
		//	SpatialData::PlacePtr place = m_newPlaces.front();
		//	m_newPlaces.pop();
		//	debug("Unlocking 1a");
		//	unlockComponent();
		//      log("New Place %i is placeholder; find exploration path", place->id);
		//	//Find an existing Place where the new place can be reached

		map<int, SpatialData::PlacePtr> places;
		lockComponent();
		for (map<string, SpatialData::PlacePtr>::iterator it = m_places.begin(); it
				!= m_places.end(); it++) {
			places[it->second->id] = it->second;
		}
		unlockComponent();
		//unsigned int nPlaces = places.size();

		//Search two-way breadth-first for the nearest unexplored place
		map<int, int> predecessor;//Place closest to start that leads here
		map<int, int> successor;//Place closest to a goal that can be reached from here
		map<int, bool> toSearched;//Has spawned all its predecessors
		map<int, bool> fromSearched;//Has spawned all its successors

		for (map<int, SpatialData::PlacePtr>::iterator it = places.begin(); it
				!= places.end(); it++) {
			unsigned int placeID = it->first;
			SpatialData::PlacePtr place = it->second;
			if (placeID == currPlaceID)
				predecessor[placeID] = placeID;
			else
				predecessor[placeID] = -1;

			if (place->status == SpatialData::PLACEHOLDER) {
				successor[placeID] = placeID;
				//log("Place %i: placeholder", placeID);
			} else
				successor[placeID] = -1;

			toSearched[placeID] = false;
			fromSearched[placeID] = false;
			//log("Place %i: %i, %i, %i, %i", placeID, predecessor[placeID], successor[placeID], (int)toSearched[placeID], (int)fromSearched[placeID]);
		}

		debug("3");
		//Procedure: For each Place with a predecessor that
		//is not already "fromSearched",
		//query first all Places with successors that aren't "toSearched"
		//then query all remaining Places without a predecessor.
		//Then mark the Place as fromSearched.
		//Then, dually:
		//For each Place with a successor that
		//is not already "toSearched",
		//query first all Places with predecessors that aren't "fromSearched"
		//then query all remaining Places without a successor.
		//Then mark the Place as toSearched.
		//Repeat.

		bool found = false;
		unsigned int nexus;
		int unsearched = places.size();
		while (!found && unsearched > 0) {
			//for (map<int, SpatialData::PlacePtr>::iterator it = places.begin();
			//    it != places.end(); it++) {
			//  unsigned int placeID = it->first;
			//  SpatialData::PlacePtr place = it->second;
			//  log("Place %i: %i, %i, %i, %i", placeID, predecessor[placeID], successor[placeID], (int)toSearched[placeID], (int)fromSearched[placeID]);
			//}
			for (map<int, SpatialData::PlacePtr>::iterator it = places.begin(); it
					!= places.end(); it++) {
				unsigned int placeID = it->first;
				SpatialData::PlacePtr place = it->second;
				if (predecessor[placeID] >= 0 && !fromSearched[placeID]) {
					for (map<int, SpatialData::PlacePtr>::iterator it2 = places.begin(); !found
							&& it2 != places.end(); it2++) {
						unsigned int toPlaceID = it2->first;
						if (successor[toPlaceID] >= 0 && !toSearched[toPlaceID]) {
							//Query i->j
							//log("A: from %i to %i", placeID, toPlaceID);
							SpatialData::PathTransitionProbRequestPtr req =
									doTransitionQuery(placeID, toPlaceID);
							if (req) {
								if (req->status == SpatialData::QUERYCOMPLETED) {
									if (req->successProb > 0.5) {
										//log("Found %i as predecessor to %i", placeID, toPlaceID);
										predecessor[toPlaceID] = placeID;

										found = true;
										//log("Found nexus at %i", toPlaceID);
										nexus = toPlaceID;
									}
								} else {
									switch (req->status) {
									case SpatialData::QUERYPLACE1INVALID:
										log("Query returned QUERYPLACE1INVALID!");
										break;
									case SpatialData::QUERYPLACE2INVALID:
										log("Query returned QUERYPLACE2INVALID!");
										break;
									case SpatialData::QUERYINTERNALERROR:
										log("Query returned QUERYINTERNALERROR!");
										break;
									default:
										log("Query returned an unexpected error!");
									}
								}
							} else {
								log("PathTransitionProbRequest disappeared!");
							}
						}
					}
					for (map<int, SpatialData::PlacePtr>::iterator it2 = places.begin(); it2
							!= places.end(); it2++) {
						unsigned int toPlaceID = it2->first;
						if (predecessor[toPlaceID] == -1 && successor[toPlaceID] == -1) {
							//Query i->j
							//log("B: from %i to %i", placeID, toPlaceID);
							SpatialData::PathTransitionProbRequestPtr req =
									doTransitionQuery(placeID, toPlaceID);
							if (req) {
								if (req->status == SpatialData::QUERYCOMPLETED) {
									if (req->successProb > 0.5) {
										//log("Found %i as predecessor to %i", placeID, toPlaceID);
										predecessor[toPlaceID] = placeID;
									}
								} else {
									switch (req->status) {
									case SpatialData::QUERYPLACE1INVALID:
										log("Query returned QUERYPLACE1INVALID!");
										break;
									case SpatialData::QUERYPLACE2INVALID:
										log("Query returned QUERYPLACE2INVALID!");
										break;
									case SpatialData::QUERYINTERNALERROR:
										log("Query returned QUERYINTERNALERROR!");
										break;
									default:
										log("Query returned an unexpected error!");
									}
								}
							} else {
								log("PathTransitionProbRequest disappeared!");
							}
						}
					}
					fromSearched[placeID] = true;
					unsearched--;
				}
			}
			for (map<int, SpatialData::PlacePtr>::iterator it = places.begin(); it
					!= places.end(); it++) {
				unsigned int toPlaceID = it->first;
				if (successor[toPlaceID] >= 0 && !toSearched[toPlaceID]) {
					for (map<int, SpatialData::PlacePtr>::iterator it2 = places.begin(); !found
							&& it2 != places.end(); it2++) {
						unsigned int fromPlaceID = it2->first;
						if (predecessor[fromPlaceID] >= 0 && !fromSearched[fromPlaceID]) {
							//Query j->i
							//log("C: from %i to %i", fromPlaceID, toPlaceID);
							SpatialData::PathTransitionProbRequestPtr req =
									doTransitionQuery(fromPlaceID, toPlaceID);
							if (req) {
								if (req->status == SpatialData::QUERYCOMPLETED) {
									if (req->successProb > 0.5) {
										//log("Found %i as successor to %i", fromPlaceID, toPlaceID);
										successor[fromPlaceID] = toPlaceID;

										found = true;
										//log("Found nexus at %i", fromPlaceID);
										nexus = fromPlaceID;
									}
								} else {
									switch (req->status) {
									case SpatialData::QUERYPLACE1INVALID:
										log("Query returned QUERYPLACE1INVALID!");
										break;
									case SpatialData::QUERYPLACE2INVALID:
										log("Query returned QUERYPLACE2INVALID!");
										break;
									case SpatialData::QUERYINTERNALERROR:
										log("Query returned QUERYINTERNALERROR!");
										break;
									default:
										log("Query returned an unexpected error!");
									}
								}
							} else {
								log("PathTransitionProbRequest disappeared!");
							}
						}
					}
					for (map<int, SpatialData::PlacePtr>::iterator it2 = places.begin(); it2
							!= places.end(); it2++) {
						unsigned int fromPlaceID = it2->first;
						if (predecessor[fromPlaceID] == -1 && successor[fromPlaceID] == -1) {
							//Query j->i
							//log("D: from %i to %i", fromPlaceID, toPlaceID);
							SpatialData::PathTransitionProbRequestPtr req =
									doTransitionQuery(fromPlaceID, toPlaceID);
							if (req) {
								if (req->status == SpatialData::QUERYCOMPLETED) {
									if (req->successProb > 0.5) {
										//log("Found %i as successor to %i", fromPlaceID, toPlaceID);
										successor[fromPlaceID] = toPlaceID;
									}
								} else {
									switch (req->status) {
									case SpatialData::QUERYPLACE1INVALID:
										log("Query returned QUERYPLACE1INVALID!");
										break;
									case SpatialData::QUERYPLACE2INVALID:
										log("Query returned QUERYPLACE2INVALID!");
										break;
									case SpatialData::QUERYINTERNALERROR:
										log("Query returned QUERYINTERNALERROR!");
										break;
									default:
										log("Query returned an unexpected error!");
									}
								}
							} else {
								log("PathTransitionProbRequest disappeared!");
							}
						}
					}
					toSearched[toPlaceID] = true;
					unsearched--;
				}
			}
		}
		if (found) {
			list<int> pathIDs;
			int i = nexus;
			while (predecessor[i] != i) {
				pathIDs.push_front(i);
				i = predecessor[i];
			}
			i = nexus;
			while (successor[i] != i) {
				pathIDs.push_back(successor[i]);
				i = successor[i];
			}

			string pathDescription;
			for (list<int>::iterator it = pathIDs.begin(); it != pathIDs.end(); it++) {
				char buffer[256];
				sprintf(buffer, " %i", *it);
				pathDescription = pathDescription + buffer;
			}
			log(string("Path: " + pathDescription));

			//if ((unsigned int)pathIDs.front() == currPlaceID){
			//pathIDs.pop_front();
			//}

			for (list<int>::iterator it = pathIDs.begin(); it != pathIDs.end(); it++) {

				string navCmdId = newDataID();
				Rendezvous *rv = new Rendezvous(*this);

				rv->addChangeFilter(createIDFilter(navCmdId, cdl::OVERWRITE)); // local

				SpatialData::NavCommandPtr navCommand = new SpatialData::NavCommand;
				navCommand->prio = SpatialData::NORMAL;
				navCommand->cmd = SpatialData::GOTOPLACE;
				navCommand->destId.push_back(*it);
				navCommand->comp = SpatialData::COMMANDPENDING;
				navCommand->status = SpatialData::NONE;

				addToWorkingMemory<SpatialData::NavCommand> (navCmdId, navCommand);

				bool done = false;
				while (!done) {
					try {
						//log("Waiting on %s", navCmdId.c_str());
						cdl::WorkingMemoryChange change = rv->wait();

						navCommand = getMemoryEntry<SpatialData::NavCommand> (navCmdId);

						if (navCommand->comp == SpatialData::COMMANDINPROGRESS) {
							log("Registered command start");
						} else {
							done = true;
							if (navCommand->comp != SpatialData::COMMANDSUCCEEDED) {
								log("Registered command failure");
							}
							log("Deleting NavCommand struct %s", navCmdId.c_str());
							deleteFromWorkingMemory(navCmdId);
						}
					} catch (DoesNotExistOnWMException) {
						log("The NavCommand suddenly disappeared...");
						done = true;
					}
				}
			}
		} else {
			log("Could not find a path to an unxeplored Place. Sleeping...");
			sleepComponent(15000);
		}

		//	//Can't use iterators in this loop, since m_places may grow (at the end
		//	//only) unexpectedly. Since it can only grow, the index won't be invalid.
		//	debug("Locking 2");
		//	lockComponent();
		//	SpatialData::PlacePtr testPlace = m_places[i];
		//	debug("Unlocking 2");
		//	unlockComponent();
		//	if (testPlace->status != SpatialData::PLACEHOLDER) {
		//	  SpatialData::PathTransitionRequestPtr req = 
		//	    doTransitionQuery(testPlace->id, place->id);
		//	  if(req){
		//	    if (req->successProb > 0.5) {
		//	      m_explorationPairs.push(pair<SpatialData::PlacePtr, SpatialData::PlacePtr>(testPlace, place));
		//	    }
		//	  }
		//	  else{
		//	    log("The request suddenly disappeared...");
		//	    return 0;
		//	  }
		//	} //if (status != PLACEHOLDER...)
		//      } //if (!m_newPlaces.empty())
		//      debug("Unlocking 1a");
		//      unlockComponent();
		//      sleepComponent(1000);
		//    } 
		//
		//    debug("Locking 3");
		//    lockComponent(); //Prevent reading m_explorationPairs during resize or something
		//    pair<SpatialData::PlacePtr, SpatialData::PlacePtr> nextExploration =
		//      m_explorationPairs.front();
		//    m_explorationPairs.pop();
		//    debug("Unlocking 3");
		//    unlockComponent();
		//
		//    {
		//Go to the starting place of the next step
		//      string navCmdId = newDataID();
		//      Rendezvous *rv = new Rendezvous(*this);
		//
		//      rv->addChangeFilter(
		//	  createIDFilter(navCmdId, cdl::OVERWRITE)); // local
		//
		//      SpatialData::NavCommandPtr navCommand = 
		//	new SpatialData::NavCommand;
		//      navCommand->prio = SpatialData::NORMAL;
		//      navCommand->cmd = SpatialData::GOTOPLACE;
		//      navCommand->destId.push_back(nextExploration.first->id);
		//      navCommand->comp = SpatialData::COMMANDPENDING;
		//      navCommand->status = SpatialData::NONE;
		//
		//      addToWorkingMemory<SpatialData::NavCommand>(navCmdId, navCommand);
		//
		//      bool done = false;
		//      while (!done) {
		//	try {
		//	  debug("Waiting on %s", navCmdId.c_str());
		//	  cdl::WorkingMemoryChange change = rv->wait();
		//
		//	  navCommand = getMemoryEntry<SpatialData::NavCommand>(navCmdId);
		//
		//	  if (navCommand->comp == SpatialData::COMMANDINPROGRESS) {
		//	    log ("Registered command start");
		//	  }
		//	  else {
		//	    done = true;
		//	    if (navCommand->comp != SpatialData::COMMANDSUCCEEDED) {
		//	      log ("Registered command failure");
		//	      nextExploration.second = 0;
		//	    }
		//	    debug("Deleting NavCommand struct %s", navCmdId.c_str());
		//	    deleteFromWorkingMemory(navCmdId);
		//	  }
		//	} catch (DoesNotExistOnWMException) {
		//	  log("The NavCommand suddenly disappeared...");
		//	  done = true;
		//	  nextExploration.second = 0;
		//	}				
		//      }
		//    }
		//
		//    {
		//      if (nextExploration.second != 0) {
		//	//Issue the final exploration nav command
		//	string navCmdId = newDataID();
		//	Rendezvous *rv = new Rendezvous(*this);
		//
		//	rv->addChangeFilter(
		//	    createIDFilter(navCmdId, cdl::OVERWRITE));
		//
		//	SpatialData::NavCommandPtr navCommand = 
		//	  new SpatialData::NavCommand;
		//	navCommand->cmd = SpatialData::GOTOPLACE;
		//	navCommand->destId.push_back(nextExploration.second->id);
		//	navCommand->comp = SpatialData::COMMANDPENDING;
		//
		//	addToWorkingMemory<SpatialData::NavCommand>(navCmdId, navCommand);
		//
		//	bool done = false;
		//	while (!done) {
		//	  try {
		//	    debug("Waiting on %s", navCmdId.c_str());
		//	    cdl::WorkingMemoryChange change = rv->wait();
		//
		//	    navCommand = getMemoryEntry<SpatialData::NavCommand>(navCmdId);
		//
		//	    if (navCommand->comp == SpatialData::COMMANDINPROGRESS) {
		//	      log ("Registered command start");
		//	    }
		//	    else {
		//	      done = true;
		//	      if (navCommand->comp != SpatialData::COMMANDSUCCEEDED) {
		//		log ("Registered command failure");
		//	      }
		//	      debug("Deleting NavCommand struct %s", navCmdId.c_str());
		//	      deleteFromWorkingMemory(navCmdId);
		//	    }
		//	  }
		//	  catch (DoesNotExistOnWMException) {
		//	    log ("Nav command disappeared from WM!");
		//	    done = true;
		//	  }
		//
		//	} //while (!done)
		//      } //if (nextExploration != 0)
		//    }
	} //while(isRunning())
}

void ExplorationTester::newPlace(const cdl::WorkingMemoryChange &objID) {
	SpatialData::PlacePtr place = getMemoryEntry<SpatialData::Place> (
			objID.address);

	m_places[objID.address.id] = place;
	if (place->status == SpatialData::PLACEHOLDER) {
		m_newPlaces.push(place);
	}
}

void ExplorationTester::PlaceChanged(const cdl::WorkingMemoryChange &objID) {
	SpatialData::PlacePtr place = getMemoryEntry<SpatialData::Place> (
			objID.address);

	m_places[objID.address.id] = place;
	log("PlaceChanged; Placeholder is %i", place->status);
}

void ExplorationTester::placeDeleted(const cdl::WorkingMemoryChange &objID) {
	string deletedID = objID.address.id;

	map<string, SpatialData::PlacePtr>::iterator it = m_places.find(deletedID);
	if (it != m_places.end()) {
		m_places.erase(it);
	}
}

SpatialData::PathTransitionProbRequestPtr ExplorationTester::doTransitionQuery(
		int startPlaceID, int goalPlaceID) {
	Rendezvous *rv = new Rendezvous(*this);

	string pathRequestID = newDataID();
	rv->addChangeFilter(createIDFilter(pathRequestID, cdl::OVERWRITE));

	SpatialData::PathTransitionProbRequestPtr pathProbRequest =
			new SpatialData::PathTransitionProbRequest;
	pathProbRequest->startPlaceID = startPlaceID;
	pathProbRequest->goalPlaceID = goalPlaceID;
	pathProbRequest->status = SpatialData::QUERYPENDING;

	addToWorkingMemory<SpatialData::PathTransitionProbRequest> (pathRequestID,
			pathProbRequest);
	//log("Issuing path prob request between %i and %i", startPlaceID, goalPlaceID);
	//log("Waiting on %s", pathRequestID.c_str());

	rv->wait();
	//log("Wait ended");
	try {
		pathProbRequest = getMemoryEntry<SpatialData::PathTransitionProbRequest> (
				pathRequestID);
		deleteFromWorkingMemory(pathRequestID);

		return pathProbRequest;

	} catch (DoesNotExistOnWMException) {
		log("The request suddenly disappeared...");
		return 0;
	}
}

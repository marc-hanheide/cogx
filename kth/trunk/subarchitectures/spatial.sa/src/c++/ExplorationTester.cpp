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

ExplorationTester::ExplorationTester()
{
}

ExplorationTester::~ExplorationTester()
{
}

void
ExplorationTester::configure(const map<string, string>& _config)
{
  log("Configure entered");
}

void 
ExplorationTester::start()
{
  //unlockComponent();
  addChangeFilter(createLocalTypeFilter<SpatialData::Place>(cdl::ADD),
      new MemberFunctionChangeReceiver<ExplorationTester>(this,
	&ExplorationTester::newPlace));
}

void 
ExplorationTester::stop()
{
}

void 
ExplorationTester::runComponent()
{
  while(isRunning()){
    log("I am running");

    while(m_explorationPairs.empty()) {//No synch required: can't become 
      //empty outside this function
      debug("Locking 1");
      //lockComponent(); //Protect m_newPlaces while reading it
      if (!m_newPlaces.empty()) {
	SpatialData::PlacePtr place = m_newPlaces.front();
	m_newPlaces.pop();

	log("New Place %i is placeholder; find exploration path", place->id);
	//Find an existing Place where the new place can be reached
	unsigned int nPlaces = m_places.size();
	debug("Unlocking 1a");
	//unlockComponent();

	for (unsigned int i = 0; i < nPlaces; i++) {
	  //Can't use iterators in this loop, since m_places may grow (at the end
	  //only) unexpectedly. Since it can only grow, the index won't be invalid.
	  debug("Locking 2");
	  //lockComponent();
	  SpatialData::PlacePtr testPlace = m_places[i];
	  debug("Unlocking 2");
	  //unlockComponent();
	  if (testPlace->status != SpatialData::PLACEHOLDER) {

	    Rendezvous *rv = new Rendezvous(*this);

	    string pathRequestID = newDataID();
	    rv->addChangeFilter(
		createIDFilter(pathRequestID, cdl::OVERWRITE));

	    SpatialData::PathTransitionProbRequestPtr
	      pathProbRequest = new SpatialData::PathTransitionProbRequest;
	    pathProbRequest->startPlaceID = testPlace->id;
	    pathProbRequest->goalPlaceID = place->id;
	    pathProbRequest->status = SpatialData::QUERYPENDING;

	    addToWorkingMemory<SpatialData::PathTransitionProbRequest>
	      (pathRequestID, pathProbRequest);
	    log("Issuing path prob request between %i and %i", (int)testPlace->id, (int)place->id);
	    debug("Waiting on %s", pathRequestID.c_str());

	    rv->wait();
	    log("Wait ended");
	    try {
	      pathProbRequest =
		getMemoryEntry<SpatialData::PathTransitionProbRequest>(pathRequestID);

	      if(pathProbRequest){
		if (pathProbRequest->successProb > 0.5) {
		  m_explorationPairs.push(pair<SpatialData::PlacePtr, SpatialData::PlacePtr>(testPlace, place));
		}
	      }
	      else{
		log("The request suddenly disappeared...");
	      }
	    } catch (DoesNotExistOnWMException) {
	      log("The request suddenly disappeared...");
	    }				
	  } //if (status != PLACEHOLDER...)
	} //for (places...)
      } //if (!m_newPlaces.empty())
      debug("Unlocking 1a");
      //unlockComponent();
      sleepComponent(1000);
    } 

    debug("Locking 3");
    //lockComponent(); //Prevent reading m_explorationPairs during resize or something
    pair<SpatialData::PlacePtr, SpatialData::PlacePtr> nextExploration =
      m_explorationPairs.front();
    m_explorationPairs.pop();
    debug("Unlocking 3");
    //unlockComponent();

    {
      //Go to the starting place of the next step
      string navCmdId = newDataID();
      Rendezvous *rv = new Rendezvous(*this);

      rv->addChangeFilter(
	  createIDFilter(navCmdId, cdl::OVERWRITE)); // local

      SpatialData::NavCommandPtr navCommand = 
	new SpatialData::NavCommand;
      navCommand->prio = SpatialData::NORMAL;
      navCommand->cmd = SpatialData::GOTOPLACE;
      navCommand->destId.push_back(nextExploration.first->id);
      navCommand->comp = SpatialData::COMMANDPENDING;
      navCommand->status = SpatialData::NONE;

      addToWorkingMemory<SpatialData::NavCommand>(navCmdId, navCommand);

      bool done = false;
      while (!done) {
	try {
	  debug("Waiting on %s", navCmdId.c_str());
	  cdl::WorkingMemoryChange change = rv->wait();

	  navCommand = getMemoryEntry<SpatialData::NavCommand>(navCmdId);

	  if (navCommand->comp == SpatialData::COMMANDINPROGRESS) {
	    log ("Registered command start");
	  }
	  else {
	    done = true;
	    if (navCommand->comp != SpatialData::COMMANDSUCCEEDED) {
	      log ("Registered command failure");
	      nextExploration.second = 0;
	    }
	    debug("Deleting NavCommand struct %s", navCmdId.c_str());
	    deleteFromWorkingMemory(navCmdId);
	  }
	} catch (DoesNotExistOnWMException) {
	  log("The NavCommand suddenly disappeared...");
	  done = true;
	  nextExploration.second = 0;
	}				
      }
    }

    {
      if (nextExploration.second != 0) {
	//Issue the final exploration nav command
	string navCmdId = newDataID();
	Rendezvous *rv = new Rendezvous(*this);

	rv->addChangeFilter(
	    createIDFilter(navCmdId, cdl::OVERWRITE));

	SpatialData::NavCommandPtr navCommand = 
	  new SpatialData::NavCommand;
	navCommand->cmd = SpatialData::GOTOPLACE;
	navCommand->destId.push_back(nextExploration.second->id);
	navCommand->comp = SpatialData::COMMANDPENDING;

	addToWorkingMemory<SpatialData::NavCommand>(navCmdId, navCommand);

	bool done = false;
	while (!done) {
	  try {
	    debug("Waiting on %s", navCmdId.c_str());
	    cdl::WorkingMemoryChange change = rv->wait();

	    navCommand = getMemoryEntry<SpatialData::NavCommand>(navCmdId);

	    if (navCommand->comp == SpatialData::COMMANDINPROGRESS) {
	      log ("Registered command start");
	    }
	    else {
	      done = true;
	      if (navCommand->comp != SpatialData::COMMANDSUCCEEDED) {
		log ("Registered command failure");
	      }
	      debug("Deleting NavCommand struct %s", navCmdId.c_str());
	      deleteFromWorkingMemory(navCmdId);
	    }
	  }
	  catch (DoesNotExistOnWMException) {
	    log ("Nav command disappeared from WM!");
	    done = true;
	  }

	} //while (!done)
      } //if (nextExploration != 0)
    }
  } //while(isRunning())
}

void
ExplorationTester::newPlace(const cdl::WorkingMemoryChange &objID)
{
  SpatialData::PlacePtr place = getMemoryEntry<SpatialData::Place>(objID.address);

  debug("Locking 5");
  //lockComponent(); //Lock access to m_places and m_newPlaces;
  m_places.push_back(place);
  if (place->status == SpatialData::PLACEHOLDER) {
    m_newPlaces.push(place);
  }
  debug("Unlocking 5");
  //unlockComponent();
}

//
// = FILENAME
//    SpatialTranslation.cpp
//
// = FUNCTION
//
// = AUTHOR(S)
//    Chandana Paul
//    Dorian Galvez
//
// = COPYRIGHT
//    Copyright (c) 2007 Chandana Paul
//    		    2009 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/

//NOTE: This file "adapted" (nicked) from nav.sa version with NavCommand structs.
//SpatialData doesn't have everything that NavData does; the extraneous
//portions of this code have been commented out. /KS

#include "SpatialTranslation.hpp"
#include <NavData.hpp>
#include <Rendezvous.h>
#include <cast/architecture/ChangeFilterFactory.hpp>
#include <list>
#include <cmath>
#include <FrontierInterface.hpp>

using namespace cast;
using namespace std;
using namespace boost;
using namespace spatial;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
cast::interfaces::CASTComponentPtr newComponent() {
	return new SpatialTranslation();
}
}

// ----------------------------------------------------------------------------

SpatialTranslation::SpatialTranslation() {

	//setOntology(NavOntologyFactory::getOntology());

	pthread_cond_init(&m_MutexCond, NULL);

	m_navGraphChanged = true;

}

// ----------------------------------------------------------------------------

SpatialTranslation::~SpatialTranslation() {
}

// ----------------------------------------------------------------------------

void SpatialTranslation::configure(const map<string, string>& config) {
	println("configure entered");

	m_StartupDelay = 4000;

	map<string, string>::const_iterator it;
	it = config.find("--startup-delay");
	if (it != config.end()) {
		std::istringstream str(it->second);
		str >> m_StartupDelay;
	}

	m_bNoNavGraph = false;
	if (config.find("--no-graph") != config.end()) {
		m_bNoNavGraph = true;
	}
	if (config.find("--visual-exploration") != config.end()) {
		m_issueVisualExplorationActions = true;
	}
	m_generatePlaceholdersOnPlace = false;
	if (config.find("--generate-placeholders-on-place") != config.end()) {
		m_generatePlaceholdersOnPlace = true;
	}

}

void SpatialTranslation::start() {
	ManagedComponent::start();

	addChangeFilter(createLocalTypeFilter<SpatialData::NavCommand> (cdl::ADD),
			new MemberFunctionChangeReceiver<SpatialTranslation> (this,
					&SpatialTranslation::newNavCommand));

	if (!m_bNoNavGraph) {
		addChangeFilter(createLocalTypeFilter<NavData::NavGraph> (cdl::ADD),
				new MemberFunctionChangeReceiver<SpatialTranslation> (this,
						&SpatialTranslation::newNavGraph));

		addChangeFilter(createLocalTypeFilter<NavData::NavGraph> (cdl::OVERWRITE),
				new MemberFunctionChangeReceiver<SpatialTranslation> (this,
						&SpatialTranslation::owtNavGraph));
	}

	m_placeInterface = FrontierInterface::PlaceInterfacePrx(getIceServer<
			FrontierInterface::PlaceInterface> ("place.manager"));

	log("SpatialTranslation started");

}

// ----------------------------------------------------------------------------

void SpatialTranslation::stop() {
	ManagedComponent::stop();
	pthread_cond_broadcast(&m_MutexCond);
}

// ----------------------------------------------------------------------------

void SpatialTranslation::runComponent() {

	sleepComponent( m_StartupDelay);
	if (m_issueVisualExplorationActions) {
		Rendezvous rv(*this);
		issueVisualExplorationCommand(rv);
		rv.wait();
		issueEndPlaceTransitionCommand(rv, false, true);
		rv.wait();
	}

	while (isRunning()) {
		log("I am running, but I wait until getting work to do");

		std::vector<boost::shared_ptr<CASTData<NavData::RobotPose2d> > > poseVector;
		while (poseVector.empty()) {
			lockComponent();
			getWorkingMemoryEntries<NavData::RobotPose2d> (0, poseVector);
			unlockComponent();
			sleepComponent(200); // wait a little...
		}
		if (!m_bNoNavGraph) {
			std::vector<boost::shared_ptr<CASTData<NavData::NavGraph> > > graphVector;
			while (graphVector.empty()) {
				lockComponent();
				getWorkingMemoryEntries<NavData::NavGraph> (0, graphVector);
				unlockComponent();
				sleepComponent(200); // wait a little...
			}
		}

		log("Got the data needed to start, now wating for task");

		m_Tasks.lock();
		while (isRunning() && m_Tasks.size() == 0) {
			pthread_cond_wait(&m_MutexCond, &m_Tasks.m_Mutex);
		}

		log("I am awake");

		if (isRunning()) {
			// ok, work to do
			tpNavCommandWithId cmd = m_Tasks.front();
			m_Tasks.pop_front(); // we take out the task
			m_Tasks.m_Abort = false;
			m_Tasks.m_Executing = true;
			m_Tasks.m_ExecutingId = cmd.first;
			m_Tasks.unlock();

			if (cmd.second->cmd == SpatialData::BLOCKCONTROL)
				// This command is special
				executeBlockCommand(cmd);
			else
				// Normal command
				executeCommand(cmd);
		} else
			m_Tasks.unlock();
	}
}

// ----------------------------------------------------------------------------

void SpatialTranslation::executeCommand(const tpNavCommandWithId &cmd) {
	// Note: this function must listen to abort events and to m_Tasks.m_Abort

	// Lets see if we can carry out the command
	NavData::InternalNavCommandPtr ctrl = new NavData::InternalNavCommand;
	//must init all enums, otherwise crashes await
	ctrl->status = NavData::NONE;
	ctrl->comp = NavData::PENDING;

	SpatialData::Completion result;
	SpatialData::StatusError status;
	log("SpatialTranslation: %i", __LINE__);
	if (translateCommand(cmd.second, *ctrl, status)) {
		// ok, execution in progress
		result = SpatialData::COMMANDINPROGRESS;
		log("SpatialTranslation: %i", __LINE__);
	} else {
		// fail :(
		result = SpatialData::COMMANDFAILED;
		log("SpatialTranslation: %i", __LINE__);
	}

	// Report fail/in_progress
	log("SpatialTranslation: %i", __LINE__);
	m_Tasks.lock();
	log("SpatialTranslation: %i", __LINE__);
	if (!m_Tasks.m_Abort) {
		// This way we make sure we dont overwrite an "abort" completion
		log("SpatialTranslation: %i", __LINE__);
		changeNavCmdCompletion(cmd.first, result, status);
	}
	log("SpatialTranslation: %i", __LINE__);
	if (result == SpatialData::COMMANDFAILED)
		m_Tasks.m_Executing = false;
	m_Tasks.unlock();
	log("SpatialTranslation: %i", __LINE__);

	// If task failed, dont go on
	if (result == SpatialData::COMMANDFAILED) {
		return;
	}

	log("SpatialTranslation: %i", __LINE__);
	// Post the nav ctrl command then
	string navCtrlCmdId = newDataID();
	string navCmdId = cmd.first;

	Rendezvous *rv = new Rendezvous(*this);

	log("SpatialTranslation: %i", __LINE__);
	rv->addChangeFilter(createLocalTypeFilter<NavData::InternalNavCommand> (
			cdl::OVERWRITE)); // local

	log("SpatialTranslation: %i", __LINE__);
	// This listens to the current nav command completion field
	rv->addChangeFilter(createLocalTypeFilter<SpatialData::NavCommand> (
			cdl::OVERWRITE));
	log("SpatialTranslation: %i", __LINE__);

	if (m_isExplorationAction) {
		rv->addChangeFilter(createLocalTypeFilter<NavData::FNode> (cdl::ADD));
		rv->addChangeFilter(createLocalTypeFilter<NavData::FNode> (cdl::OVERWRITE));
	}

	bool some_error = false;
	bool aborted = false;

	// This is in case the abort signal arrives before adding the last filter
	log("SpatialTranslation: %i", __LINE__);
	if (!m_Tasks.m_Abort) {
		log("SpatialTranslation: %i", __LINE__);
		// Send command
		addToWorkingMemory<NavData::InternalNavCommand> (navCtrlCmdId, ctrl);

		// Wait until resolution...
		cdl::WorkingMemoryChange change;
		bool finished = false;
		while (!aborted && !finished) {
			log("before rv wait...");
			change = rv->wait();
			log("after rv wait");

			debug("Change:");
			debug("::type");
			debug(change.type);
			debug("::op");
			switch (change.operation) {
			case cdl::ADD:
				debug("add");
				break;
			case cdl::OVERWRITE:
				debug("overwrite");
				break;
			case cdl::DELETE:
				debug("delete");
				break;
			case cdl::GET:
				debug("get");
				break;
			default:
				debug("wildcard??");
				break;
			}
			debug("::src");
			debug(change.src);

			string type = change.type;
			if (type == typeName<SpatialData::NavCommand> ()) {
				debug("abort?");
				// abort got?
				try {
					debug("locking");
					lockEntry(navCmdId, cdl::LOCKEDOD);
					debug("locked");
					try {

						shared_ptr<CASTData<SpatialData::NavCommand> > pcmd =
								getWorkingMemoryEntry<SpatialData::NavCommand> (navCmdId);

						if (pcmd) {
							status = pcmd->getData()->status;
							aborted = (status != SpatialData::NONE);
							log("%s overwrote current command with status %d (aborted? %d)",
									change.src.c_str(), status, aborted);
							// I.e. someone outside decided we're done now - the internal cmd needs
							// to be cancelled either way.
						} else {
							log("The NavCommand suddenly disappeared...");
							some_error = true;
							finished = true;
						}
					} catch (DoesNotExistOnWMException) {
						log("The NavCommand suddenly disappeared...");
						some_error = true;
						finished = true;
					}
					debug("unlocking");
					unlockEntry(navCmdId);
				} catch (DoesNotExistOnWMException) {
					log("The NavCommand disappeared while waiting to lock.");
					some_error = true;
					finished = true;
				} catch (ConsistencyException e) {
					log(
							"Error! ConsistencyException in SpatialTranslation::executeCommand!");
				}
				debug(aborted ? "yes" : "no");
			} else if (type == typeName<NavData::InternalNavCommand> ()) {
				if (change.address.id == navCtrlCmdId) {
					// nav ctrl cmd finished?
					shared_ptr<CASTData<NavData::InternalNavCommand> > pcmd =
							getWorkingMemoryEntry<NavData::InternalNavCommand> (navCtrlCmdId);

					if (pcmd) {
						switch (pcmd->getData()->comp) {
						case NavData::SUCCEEDED:

							log("nav ctrl cmd succeeded");
							if (m_isExplorationAction) {
								if (m_issueVisualExplorationActions) {
									log("issuing exploration command");
									issueVisualExplorationCommand(*rv);
								} else {
									log("issuing placeholder enumerating command");
									issueEndPlaceTransitionCommand(*rv, false, true);
								}
							} else if (m_generatePlaceholdersOnPlace) {
								log("issuing placeholder enumerating command");
								issueEndPlaceTransitionCommand(*rv, false, true);
							} else {
								log("NavCommand finished");
								issueEndPlaceTransitionCommand(*rv, false, false);

								finished = true;
							}

							m_isExplorationAction = false;

							navCtrlCmdId = "";
							break;
						case NavData::ABORTED:
						case NavData::FAILED:
							log("nav ctrl command failed");

							some_error = true;
							issueEndPlaceTransitionCommand(*rv, true, true);

							status = SpatialData::TARGETUNREACHABLE;

							navCtrlCmdId = "";
							break;
						default:
							break;
						}

					} else {
						log("The InternalNavCommand suddenly disappeared...");
						some_error = true;
						issueEndPlaceTransitionCommand(*rv, true, true);
					}
				}
			} else if (type == typeName<NavData::VisualExplorationCommand> ()) {
				log("Visual exploration cmd finished");
				issueEndPlaceTransitionCommand(*rv, false, true);
			} else if (type == typeName<NavData::EndPlaceTransitionCommand> ()) {
				log("EndPlaceTransition cmd finished");
				finished = true;
			}

			else if (type == typeName<NavData::FNode> ()) {
				if (navCtrlCmdId != "") {
					log("Halting movement: exploration reached new node");

					navCtrlCmdId = newDataID();
					NavData::InternalNavCommandPtr stopCommand =
							new NavData::InternalNavCommand;

					stopCommand->cmd = NavData::lSTOPROBOT;
					stopCommand->x = 0.0;
					stopCommand->y = 0.0;
					stopCommand->status = NavData::NONE;
					stopCommand->comp = NavData::PENDING;
					stopCommand->theta = 0.0;

					addToWorkingMemory(navCtrlCmdId, stopCommand);
				}
			}
		} // while(...)

		// make sure you delete rv before cancelCurrentTask, in case this rv
		// and that function have similar filters...
		delete rv;

		debug("before cancel current task");
		if (aborted) {
			cancelCurrentTask(true, navCtrlCmdId);
			result = SpatialData::COMMANDABORTED;
			changeNavCmdCompletion(navCmdId, result, status);
		}
		debug("after cancel current task");

	} else { // if(!m_Tasks.m_Abort)
		delete rv;
	}

	// Disable executing flag
	debug("before lock");
	m_Tasks.lock();
	m_Tasks.m_Executing = false; // this means it cannot be marked as aborted
	if (m_Tasks.m_Abort)
		aborted = true;
	m_Tasks.unlock();
	debug("after lock");

	if (!aborted) {
		// Post the final status of the NavCommand
		if (some_error) {
			result = SpatialData::COMMANDFAILED;
		} else {
			result = SpatialData::COMMANDSUCCEEDED;
		}

		changeNavCmdCompletion(navCmdId, result, status);

	} // else, the ABORT completion was filled by cancelQueueTasks

	log("Task execution finished");

}

void SpatialTranslation::issueVisualExplorationCommand(Rendezvous &rv) {
	std::string ID = newDataID();
	NavData::VisualExplorationCommandPtr cmd =
			new NavData::VisualExplorationCommand;
	cmd->comp = NavData::PENDING;

	rv.addChangeFilter(createLocalTypeFilter<NavData::VisualExplorationCommand> (
			cdl::OVERWRITE)); // local
	addToWorkingMemory<NavData::VisualExplorationCommand> (ID, cmd);
}

void SpatialTranslation::issueEndPlaceTransitionCommand(Rendezvous &rv,
		bool failed, bool generate_placeholders) {
	std::string ID = newDataID();
	NavData::EndPlaceTransitionCommandPtr cmd =
			new NavData::EndPlaceTransitionCommand;
	cmd->comp = NavData::PENDING;
	cmd->failed = failed;
	cmd->generatePlaceholders = generate_placeholders;

	rv.addChangeFilter(
			createLocalTypeFilter<NavData::EndPlaceTransitionCommand> (cdl::OVERWRITE)); // local
	addToWorkingMemory<NavData::EndPlaceTransitionCommand> (ID, cmd);
}

// ----------------------------------------------------------------------------

void SpatialTranslation::executeBlockCommand(const tpNavCommandWithId &cmd) {

	log("Executing block command");

	// Listen to abortions or deleting of the block command
	Rendezvous rv = Rendezvous(*this);

	rv.addChangeFilter(createChangeFilter<SpatialData::NavCommand> (
			cdl::OVERWRITE, "", // srcdsa
			cmd.first, // address id
			subarchitectureID(), // address sa
			cdl::LOCALSA)); // local

	rv.addChangeFilter(createChangeFilter<SpatialData::NavCommand> (cdl::DELETE,
			"", // src
			cmd.first, // change id
			subarchitectureID(), // change sa 
			cdl::LOCALSA)); // local (object.sa, probably)

	// Mark the command as in_progress	
	m_Tasks.lock();
	if (!m_Tasks.m_Abort) {
		// This way we make sure we dont overwrite an "abort" completion

		// If the abortion and the removing are done exactly now,
		// changeNavCmdCompletion will take care of it
		changeNavCmdCompletion(cmd.first, SpatialData::COMMANDINPROGRESS,
				SpatialData::NONE);
		debug("Blocking command marked as In progress");
	}
	m_Tasks.unlock();

	debug("Waiting until something happens on the blocking command...");

	// And wait until something happens...
	bool finished = m_Tasks.m_Abort;
	while (!finished) {
		rv.wait();

		if (existsOnWorkingMemory(cmd.first)) {
			shared_ptr<CASTData<SpatialData::NavCommand> > oobj =
					getWorkingMemoryEntry<SpatialData::NavCommand> (cmd.first);

			if (oobj)
				finished = (oobj->getData()->comp == SpatialData::COMMANDABORTED);
			else
				finished = true;
		} else
			finished = true;
	}

	// Command finished, disable executing flag
	m_Tasks.lock();
	m_Tasks.m_Executing = false; // this means it cannot be marked as aborted
	m_Tasks.unlock();

	// if aborted, the ABORT completion was filled by cancelQueueTasks

	log("Block command execution finished");

}

// ----------------------------------------------------------------------------

void SpatialTranslation::cancelCurrentTask(bool stop_robot, string navCtrlCmdId) {

	// Send a stop InternalNavCommand
	if (stop_robot) {
		NavData::InternalNavCommandPtr stop = new NavData::InternalNavCommand;
		stop->cmd = NavData::lSTOPROBOT;
		//must init all enums, otherwise crashes await
		stop->status = NavData::NONE;
		stop->comp = NavData::PENDING;

		Rendezvous rv(*this);
		string aux_id = newDataID();

		rv.addChangeFilter(createChangeFilter<NavData::InternalNavCommand> (
				cdl::OVERWRITE, "", aux_id, subarchitectureID(), cdl::LOCALSA));
		addToWorkingMemory<NavData::InternalNavCommand> (aux_id, stop);

		cdl::WorkingMemoryChange change;
		bool terminal = false;
		while (!terminal) {
			debug("cancel current task: before wait");
			change = rv.wait();
			debug("cancel current task: after wait");

			shared_ptr<CASTData<NavData::InternalNavCommand> > oobj =
					getWorkingMemoryEntry<NavData::InternalNavCommand> (change.address);

			if (oobj) {
				switch (oobj->getData()->comp) {
				case NavData::FAILED:
				case NavData::SUCCEEDED:
				case NavData::ABORTED:
					terminal = true;
					break;
				default:
					break;
				}
			} else
				terminal = true;
		}

		// Remove the stop command
		deleteFromWorkingMemory(aux_id);

	} // if(stop_robot)

	if (navCtrlCmdId != "") {
		deleteFromWorkingMemory(navCtrlCmdId);
		debug("Deleting this nav ctrl command");
		debug(navCtrlCmdId.c_str());
	}

}

// ----------------------------------------------------------------------------

void SpatialTranslation::newNavCommand(const cdl::WorkingMemoryChange & objID) {

	log("New NavCommand got");

	shared_ptr<CASTData<SpatialData::NavCommand> > oobj = getWorkingMemoryEntry<
			SpatialData::NavCommand> (objID.address);

	if (oobj != 0) {
		string navId = objID.address.id;
		// this adds the task and signals the m_MutexCond
		addTaskToQueue(navId, oobj->getData());
	}
}

//void
//SpatialTranslation::newNavNode(const WorkingMemoryChange &wmc)
//{
//  debug("Received new navNode");
//  try {
//    NavData::VisualExplorationCommandPtr obj =
//      getMemoryEntry<NavData::FNode>(objID.address);
//
//    if (m_isExplorationAction) {
//      // A new node was created while carrying out an explorative movement
//      // 
//    }
//  }
//  catch (DoesNotExistOnWMException)
//  {
//    log("Error: new FNode disappeared from WM!");
//  }
//}


// ----------------------------------------------------------------------------

void SpatialTranslation::addTaskToQueue(const std::string &id,
		const SpatialData::NavCommandPtr &cmd) {
	tpNavCommandWithId idded(id, cmd);
	bool first = true;

	m_Tasks.lock();
	switch (cmd->prio) {
	case SpatialData::NORMAL:
	default:
		log("Adding task at the end of the queue");
		first = (m_Tasks.size() == 0);
		m_Tasks.push_back(idded);
		break;
	case SpatialData::HIGH:
		log("Adding task at the front of the queue");
		m_Tasks.push_front(idded);
		break;
	case SpatialData::URGENT:
		log("Adding task after cancelling all the others");
		m_Tasks.m_Abort = true;
		cancelQueueTasks();
		m_Tasks.push_front(idded);
		break;
	}

	// if the task is not going to be executed now, we mark it as pending
	if (m_Tasks.m_Executing || (!first && m_Tasks.size() > 1)) {
		changeNavCmdCompletion(id, SpatialData::COMMANDPENDING, cmd->status);
	}

	pthread_cond_signal(&m_MutexCond);
	m_Tasks.unlock();
}

// ----------------------------------------------------------------------------

void SpatialTranslation::cancelQueueTasks() {
	// Remember: m_Tasks is locked and the ongoing task is being stopped
	tpQueue::iterator it;
	for (it = m_Tasks.begin(); it != m_Tasks.end(); it++) {
		// Cancel NavCommand on WM 
		changeNavCmdCompletion(it->first, SpatialData::COMMANDABORTED,
				SpatialData::NONE);
	}
	m_Tasks.clear();

	// Now the ongoing task, if any
	if (m_Tasks.m_Executing) {
		// actually, this is only true when the action is finished in
		// executeCommand or executeBlockCommand
		m_Tasks.m_Executing = false;
		changeNavCmdCompletion(m_Tasks.m_ExecutingId, SpatialData::COMMANDABORTED,
				SpatialData::NONE);
	}
}

// ----------------------------------------------------------------------------

void SpatialTranslation::newNavGraph(const cdl::WorkingMemoryChange &objID) {
	// Get the first navgraph

	shared_ptr<CASTData<NavData::NavGraph> > oobj = getWorkingMemoryEntry<
			NavData::NavGraph> (objID.address);

	if (oobj) {
		m_NavGraphMutex.lock();
		m_TmpNavGraph = oobj->getData();
		m_navGraphChanged = false;
		m_NavGraphMutex.unlock();
	}

}

// ----------------------------------------------------------------------------

void SpatialTranslation::owtNavGraph(const cdl::WorkingMemoryChange &objID) {
	m_NavGraphMutex.lock();
	m_navGraphChanged = true;
	m_NavGraphMutex.unlock();
}

// ----------------------------------------------------------------------------

bool SpatialTranslation::translateCommand(
		const SpatialData::NavCommandPtr &nav, NavData::InternalNavCommand &ctrl,
		SpatialData::StatusError &status) {
	status = SpatialData::NONE;

	if (nav->cmd == SpatialData::BLOCKCONTROL) {
		log("read navcommand: BLOCKCONTROL");
		// nothing needs being done

	} else if (nav->cmd == SpatialData::GOTOPOSITION) {

		log("read navcommand: GOTOPOSITION");

		if (nav->pose.size() < 2) {
			println("cmd syntax error, need NavCommand.pose (size 2)");
			status = SpatialData::CMDMALFORMATTED;
			return false;
		} else {
			ctrl.x = nav->pose[0];
			ctrl.y = nav->pose[1];
			ctrl.tolerance = nav->tolerance;

			if (nav->pose.size() == 2) {
				ctrl.cmd = NavData::lGOTOXY;
			} else {
				ctrl.cmd = NavData::lGOTOXYA;
				ctrl.theta = nav->pose[2];
			}
		}

	} else if (nav->cmd == SpatialData::GOTOPLACE) {

		log("read navcommand: GOTOPLACE");
		if (!m_bNoNavGraph) {

			if (nav->destId.size() < 1) {
				println("cmd syntax error, need NavCommand.destId (size 1)");
				status = SpatialData::CMDMALFORMATTED;
				return false;
			} else {
				//TODO: make component name a config parameter
				NavData::FNodePtr destNode = m_placeInterface->getNodeFromPlaceID(
						nav->destId[0]);

				if (destNode != 0) {
					ctrl.cmd = NavData::lGOTONODE;
					ctrl.nodeId = destNode->nodeId;
					if (nav->tolerance.empty()) {
						vector<double> tol;
						tol.push_back(0.15);
						ctrl.tolerance = tol;
					} else
						ctrl.tolerance = nav->tolerance;
				} else {
					SpatialData::NodeHypothesisPtr destHyp =
							m_placeInterface->getHypFromPlaceID(nav->destId[0]);
					if (destHyp != 0) {

						ctrl.cmd = NavData::lGOTOXY;
						ctrl.x = destHyp->x;
						ctrl.y = destHyp->y;
						ctrl.tolerance = nav->tolerance;
						m_isExplorationAction = true;
					} else {//destNode == 0 && destHyp == 0
						log("cmd error; could not find destination Place");
						status = SpatialData::CMDMALFORMATTED;
						return false;
					}
				}
				m_placeInterface->beginPlaceTransition((int) nav->destId[0]);
				log("Sending transition start signal (place ID=%i)",
						(int) nav->destId[0]);
			}
		}
	} else if (nav->cmd == SpatialData::GOFORWARD) {

		log("read navcommand: GOFORWARD");

		// find the position of the robot
		NavData::RobotPose2dPtr pose;
		if (!getRobotPose(pose) && pose) {
			println("Could not find robot pose and cannot GOFORWARD");
			status = SpatialData::CMDMALFORMATTED;
			return false;
		}

		if (nav->distance.empty()) {
			println("Need distance in GOFOWARD command");
			status = SpatialData::CMDMALFORMATTED;
			return false;
		}

		// add a small increment in the forward direction
		// send the x,y,theta to NavController
		ctrl.cmd = NavData::lGOTOXYA;
		ctrl.x = pose->x + nav->distance[0] * cos(pose->theta);
		ctrl.y = pose->y + nav->distance[0] * sin(pose->theta);
		ctrl.theta = pose->theta;
		ctrl.tolerance = nav->tolerance;

	} else if (nav->cmd == SpatialData::GOBACK) {

		log("read navcommand: GO_BACK");

		if (nav->distance.empty()) {
			println("Need distance in GOBACK command");
			status = SpatialData::CMDMALFORMATTED;
			return false;
		}

		ctrl.cmd = NavData::lBACKOFF;
		ctrl.distance = nav->distance[0];
		ctrl.tolerance = nav->tolerance;

	} else if (nav->cmd == SpatialData::TURN) {

		log("read navcommand: TURN");

		if (!nav->angle.empty()) {
			ctrl.cmd = NavData::lROTATEREL;
			ctrl.theta = nav->angle[0];
			ctrl.tolerance = nav->tolerance;
			/*} else if (!nav->target.empty()) {

			 // interpret the string which says "left" or "right"
			 string direction = nav->target[0];
			 
			 //send the command to the NavController
			 ctrl.cmd = NavData::lROTATEREL;
			 if(direction == "left")
			 ctrl.theta = M_PI*45.0/180.0;
			 else if(direction == "right")
			 ctrl.theta = -M_PI*45.0/180.0;
			 else{
			 status = SpatialData::TARGETUNREACHABLE;
			 return false;
			 }
			 ctrl.tolerance = nav->tolerance;*/
		} else {
			println("NavCommand TURN lacks target spec");
			status = SpatialData::CMDMALFORMATTED;
			return false;
		}

	} else if (nav->cmd == SpatialData::TURNTO) {

		log("read navcommand: TURNTO");

		if (nav->angle.empty()) {
			println("NavCommand TURNTO lacks angle spec");
			status = SpatialData::CMDMALFORMATTED;
			return false;
		}

		//send the command to the NavController
		ctrl.cmd = NavData::lROTATEABS;
		ctrl.theta = nav->angle[0];
		ctrl.tolerance = nav->tolerance;

		//  }else if (nav->cmd == NavData::PERSONFOLLOW) {
		//	
		//    // call follow person, which wil write InternalNavCommand 
		//    // to working memory
		//    ctrl.cmd = NavData::lFOLLOWPERSON;		
		//
		//    log("read navcommand: PERSON_FOLLOW");
		//
		//  }else if (nav->cmd == SpatialData::EXPLORE) {
		//	
		//    log("read navcommand: EXPLORE");
		//
		//    // call explore process, which will write InternalNavCommands
		//    // to working memory
		//    ctrl.cmd = NavData::lEXPLORE;
		//    ctrl.SetExplorerConfinedByGateways = true;
		//
		//
	} else if (nav->cmd == SpatialData::STOP) {
		log("read navcommand: STOP");

		// send the stop command to the NavController
		ctrl.cmd = NavData::lSTOPROBOT;
		ctrl.x = 0.0;
		ctrl.y = 0.0;
		ctrl.theta = 0.0;
	}
	//  else if (nav->cmd == NavData::EXPLOREFLOOR){
	//    log("read navcommand: EXPLORE FLOOR");
	//    ctrl.cmd = NavData::lEXPLOREFLOOR;
	//    ctrl.SetExplorerConfinedByGateways = false;
	//  }


	return true;
}

// ----------------------------------------------------------------------------

bool SpatialTranslation::getRobotPose(NavData::RobotPose2dPtr &pose) {

	vector<shared_ptr<CASTData<NavData::RobotPose2d> > > v;
	getWorkingMemoryEntries<NavData::RobotPose2d> (1, v);
	if (v.size() == 0)
		return false;
	else {
		pose = v[0]->getData();
		return true;
	}
}

// ----------------------------------------------------------------------------

NavData::NavGraphPtr SpatialTranslation::getNavGraph() {
	if (m_navGraphChanged) {
		vector<shared_ptr<CASTData<NavData::NavGraph> > > v;
		getWorkingMemoryEntries<NavData::NavGraph> (1, v);
		m_NavGraphMutex.lock();
		if (v.size() > 0) {
			m_TmpNavGraph = v[0]->getData();
			m_navGraphChanged = false;
		}
		m_NavGraphMutex.unlock();
	}
	return m_TmpNavGraph;
}

// ----------------------------------------------------------------------------

bool SpatialTranslation::findObjectNode(const std::string &objectName,
		const NavData::NavGraphPtr &graph, double &x, double &y) {
	for (unsigned long i = 0; i < graph->objects.size(); i++) {
		if (graph->objects[i]->category == objectName) {
			x = graph->objects[i]->x;
			y = graph->objects[i]->y;
			return true;
		}
	}
	return false;
}

// ----------------------------------------------------------------------------

bool SpatialTranslation::findObjectBestNode(const std::string &objectName,
		const NavData::NavGraphPtr &graph, long &nodeId) {
	long areaId = -1;
	nodeId = -1;

	NavData::ObjDataPtr objData;
	for (unsigned long i = 0; i < graph->objects.size(); i++) {
		if (graph->objects[i]->category == objectName) {
			areaId = graph->objects[i]->areaId;
			objData = graph->objects[i];
			break;
		}
	}

	if (areaId != -1) {
		double minD = 1e06;
		for (unsigned long i = 0; i < graph->fNodes.size(); i++) {
			if (graph->fNodes[i]->areaId == areaId) {
				const NavData::FNode &node = *(graph->fNodes[i]);
				double dx = node.x - objData->x;
				double dy = node.y - objData->y;
				double d = sqrt(dx * dx + dy * dy);

				if (d < minD) {
					nodeId = graph->fNodes[i]->nodeId;
					minD = d;
				}
			}
		}
	}

	return (nodeId != -1);
}

// ----------------------------------------------------------------------------


void SpatialTranslation::changeNavCmdCompletion(const std::string &id,
		const SpatialData::Completion &completion,
		const SpatialData::StatusError &status) {
	shared_ptr<CASTData<SpatialData::NavCommand> > pcmd;
	SpatialData::NavCommandPtr newcmd;
	bool stop = false;

	log("SpatialTranslation.cpp: %i", __LINE__);
	try {
		lockEntry(id, cdl::LOCKEDOD);
		log("SpatialTranslation.cpp: %i", __LINE__);
		pcmd = getWorkingMemoryEntry<SpatialData::NavCommand> (id);
		log("SpatialTranslation.cpp: %i", __LINE__);
	} catch (DoesNotExistOnWMException) {
		debug("changeNavCmdCompletion called for nonexistent NavCommand");
		log("SpatialTranslation.cpp: %i", __LINE__);
		return;
	}
	while (!stop) {
		newcmd = new SpatialData::NavCommand(*pcmd->getData());
		newcmd->comp = completion;
		newcmd->status = status;

		try {
			log("SpatialTranslation.cpp: %i", __LINE__);
			overwriteWorkingMemory<SpatialData::NavCommand> (id, newcmd);
			log(std::string("Overwrote Nav Command: ") + id);
			stop = true;
		} catch (ConsistencyException) {

			log("SpatialTranslation.cpp: %i", __LINE__);
			// repeat only if the completion we are setting is 
			// more important than the one set				
			pcmd = getWorkingMemoryEntry<SpatialData::NavCommand> (id);
			stop = (completion <= pcmd->getData()->comp);
		} catch (DoesNotExistOnWMException) {
			log("SpatialTranslation.cpp: %i", __LINE__);
			// we can get an exception for trying to update an already
			// deleted entry (because it was aborted)
			stop = true;
		}
	}
	log("SpatialTranslation.cpp: %i", __LINE__);
	unlockEntry(id);
	log("SpatialTranslation.cpp: %i", __LINE__);
}

// ----------------------------------------------------------------------------


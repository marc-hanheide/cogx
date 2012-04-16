#include "WMControl.hpp"

#include <iostream>
#include <sstream>
#include <cassert>
#include <boost/foreach.hpp>
#include "beliefs_cogx.hpp"

using namespace std;
using namespace de::dfki::lt::tr::beliefs::slice;
using namespace de::dfki::lt::tr::beliefs::slice::sitbeliefs;
using namespace de::dfki::lt::tr::beliefs::slice::logicalcontent;
using namespace eu::cogx::beliefs::slice;
using namespace cast::cdl;

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new WMControl();
    }
}

static const int MAX_PLANNING_RETRIES = 2;
static const int MAX_EXEC_RETRIES = 3;
static const int REPLAN_DELAY = 3000;
static const int PLANNER_UPDATE_DELAY = 200;
static int BELIEF_TIMEOUT = 500;

static const string BINDER_SA = "binder";

void WMControl::configure(const cast::cdl::StringMap& _config, const Ice::Current& _current) {
    m_lastUpdate = getCASTTime();
    m_new_updates = false;
    m_active_task_id = -1;
    gettimeofday(&m_belief_activity_timeout, NULL);

    cast::ManagedComponent::configure(_config, _current);

    cast::cdl::StringMap::const_iterator it = _config.begin();
    it = _config.find("--server");
    if (it != _config.end()) {
        m_python_server = it->second;
    }
    else {
        m_python_server = "PlannerPythonServer";
    }
    m_continual_state_updates = (_config.find("--continual_updates") != _config.end());

    it = _config.find("--belief-timeout");
    if (it != _config.end()) {
        BELIEF_TIMEOUT = atoi(it->second.c_str());
    }
}

void WMControl::start() {
    log("Planner WMControl: initializing");
    addChangeFilter(cast::createLocalTypeFilter<autogen::Planner::PlanningTask>(cast::cdl::ADD), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::receivePlannerCommands));

    addChangeFilter(cast::createLocalTypeFilter<autogen::Planner::PlanningTask>(cast::cdl::OVERWRITE), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::taskChanged));

    addChangeFilter(cast::createLocalTypeFilter<autogen::Planner::PlanningTask>(cast::cdl::DELETE), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::taskRemoved));

    addChangeFilter(cast::createLocalTypeFilter<Action>(cast::cdl::OVERWRITE), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::actionChanged));

    addChangeFilter(cast::createGlobalTypeFilter<dBelief>(cast::cdl::WILDCARD),
            new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::stateChanged));

    addChangeFilter(cast::createGlobalTypeFilter<PerceptBelief>(cast::cdl::ADD),
            new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::newPercept));
    addChangeFilter(cast::createGlobalTypeFilter<PerceptBelief>(cast::cdl::OVERWRITE),
            new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::newPercept));

    connectToPythonServer();
}

void WMControl::connectToPythonServer() {
    log("Planner WMControl: connecting to Python Server");
    try	{
		pyServer = getIceServer<autogen::Planner::PythonServer>(m_python_server);
    }
    catch (const Ice::Exception& ex) {
	cerr << ex << endl;
    }
    catch (const char* msg) {
	cerr << msg << endl;
    }
    
    autogen::Planner::CppServerPtr servant = new InternalCppServer(this);
    registerIceServer<autogen::Planner::CppServer,autogen::Planner::CppServer>(servant);
}

void WMControl::runComponent() {
    log("Planner WMControl: running");
    std::vector<int> execute;
    std::vector<int> timed_out;
    while (isRunning()) {
        // log("Running...");
        m_queue_mutex.lock();
        // log("Got lock");
        bool waiting = !m_waiting_tasks.empty();
        if (!m_runqueue.empty() || !m_waiting_tasks.empty()) {
            // log("planning is scheduled");
            timeval tval;
            gettimeofday(&tval, NULL);
            log("planning is scheduled. Time: %ld / %ld", tval.tv_sec, tval.tv_usec);

            if (later_than(tval, m_belief_activity_timeout)) {
                for(std::map<int, timeval>::iterator it=m_runqueue.begin(); it != m_runqueue.end(); ++it) {
                    if (later_than(tval, it->second)) {
                        log("Scheduling task %d: %ld / %ld", it->first, it->second.tv_sec, it->second.tv_usec);
                        execute.push_back(it->first);
                    } else {
                        log("Waiting a bit longer for task %d: %ld / %ld", it->first, it->second.tv_sec, it->second.tv_usec);
                    }
                }
                for(std::map<int, timeval>::iterator it=m_waiting_tasks.begin(); it != m_waiting_tasks.end(); ++it) {
                    if (later_than(tval, it->second)) {
                        log("Scheduling waiting task %d: %ld / %ld", it->first, it->second.tv_sec, it->second.tv_usec);
                        timed_out.push_back(it->first);
                    } else {
                        log("Waiting a bit longer for waiting task %d: %ld / %ld", it->first, it->second.tv_sec, it->second.tv_usec);
                    }
                }
                for (std::vector<int>::iterator it=execute.begin(); it != execute.end(); ++it) {
                    log("Removing task %d from runqueue", *it);
                    m_runqueue.erase(*it);
                }
                for (std::vector<int>::iterator it=timed_out.begin(); it != timed_out.end(); ++it) {
                    log("Removing waiting task %d from runqueue", *it);
                    m_waiting_tasks.erase(*it);
                }
            } else {
                log("Waiting for belief state to settle down. Timeout: %ld / %ld", m_belief_activity_timeout.tv_sec, m_belief_activity_timeout.tv_usec);
            }
        }
        m_queue_mutex.unlock();
        // log("Released lock");

        if (!execute.empty() || !timed_out.empty() || (waiting && m_new_updates)) {
            log("Something to do.");
            lockComponent();
            log("Component locked to get beliefs");
            m_new_updates = false;
            vector<BeliefEntry> state;
            for (BeliefMap::const_iterator i=m_currentState.begin(); i != m_currentState.end(); ++i) {
                state.push_back(i->second);
            }
            unlockComponent();
            log("Component unlocked");
            log("Sending state update");
            pyServer->updateState(state, m_percepts);
            m_percepts.clear();

            for (std::vector<int>::iterator it=execute.begin(); it != execute.end(); ++it) {
                log("Sending task update for task %d", *it);
                lockComponent();
                if (activeTasks.find(*it) != activeTasks.end()) {
                    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[*it]);
                    unlockComponent();
                    log("sending task update...");
                    pyServer->updateTask(task);
                    log("task returned");
                }
                else {
                    unlockComponent();
                    log("Task %d is no longer active", *it);
                }

            }
            for (std::vector<int>::iterator it=timed_out.begin(); it != timed_out.end(); ++it) {
                log("Sending task timeout for task %d", *it);
                lockComponent();
                if (activeTasks.find(*it) != activeTasks.end()) {
                    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[*it]);
                    unlockComponent();
                    log("task timeout reached.");
                    pyServer->taskTimedOut(task);
                    log("task returned");
                }
                else {
                    unlockComponent();
                    log("Task %d is no longer active", *it);
                }
            }
            execute.clear();
            timed_out.clear();
        }
        sleepComponent(200);
    }
}

static int TASK_ID = 0;

void WMControl::receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc) {
    log("Planner WMControl: new PlanningTask received:");
    TASK_ID++;

    autogen::Planner::PlanningTaskPtr task = getMemoryEntry<autogen::Planner::PlanningTask>(wmc.address);

    log("Goals:");
    BOOST_FOREACH(GoalPtr g, task->goals) {
        log("    importance %.2f: %s", g->importance, g->goalString.c_str());
    }

    if (task->executePlan) {
        m_active_task_id = TASK_ID;
    }

    task->id = TASK_ID;
    task->plan = vector<ActionPtr>();
    task->executionStatus = PENDING;
    task->firstActionID = "";
    task->planningStatus = PENDING;
    task->planningRetries = 0;
    task->executionRetries = 0;
    activeTasks[task->id] = wmc.address;

    overwriteWorkingMemory(wmc.address, task);
    
    vector<BeliefEntry> state;
    for (BeliefMap::const_iterator i=m_currentState.begin(); i != m_currentState.end(); ++i) {
        state.push_back(i->second);
    }
    pyServer->updateState(state, m_percepts);
    m_percepts.clear();
    
    pyServer->registerTask(task);

    //TODO: Store the PlanningTaskPtr with it's taskID till deliverPlan(taskID) is called.
}

void WMControl::taskChanged(const cast::cdl::WorkingMemoryChange& wmc) {
    PlanningTaskPtr task;
    try {
         task = getMemoryEntry<PlanningTask>(wmc.address);
    }
    catch (cast::DoesNotExistOnWMException) {
        log("task has vanished");
        taskRemoved(wmc);
        return;
    }

    if (task->executionStatus != PENDING) {
        return;
    }

    assert(activeTasks.find(task->id) != activeTasks.end());

    log("task update from: %s", wmc.src.c_str());
    if (task->executePlan && task->planningStatus == SUCCEEDED && task->firstActionID == "" && task->plan.size() > 0 ) {
        log("Starting execution of task %d", task->id);
        m_active_task_id = task->id;
        deliverPlan(task->id, task->plan, task->goals);
    }
}

void WMControl::taskRemoved(const cast::cdl::WorkingMemoryChange& wmc) {
    BOOST_FOREACH(taskMap::value_type entry, activeTasks) {
        if (entry.second == wmc.address) {
            //Delete the associated action (if it exists))
            vector<cast::CASTData<Action> > actions;
            getMemoryEntriesWithData<Action>(actions);
            BOOST_FOREACH(cast::CASTData<Action> wme, actions) {
                if (wme.getData()->taskID == entry.first) {
                    deleteFromWorkingMemory(wme.getID());
                    break;
                }
            }
            activeTasks.erase(entry.first);
            break;
        }
    }
}

/*void WMControl::generateState() {
    log("Planner WMControl:: generating state");
    vector<BeliefPtr>* state = new vector<BeliefPtr>(m_currentState.size());

    //task->state = vector<BeliefPtr>(m_currentState);
    }*/

void WMControl::actionChanged(const cast::cdl::WorkingMemoryChange& wmc) {
    log("Action changed.");
    ActionPtr action = getMemoryEntry<Action>(wmc.address);

    if (action->status == PENDING) { // We just added this action ourselves
        return;
    }
    log("Action %s changed to status %d", action->name.c_str(), action->status);

    assert(activeTasks.find(action->taskID) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[action->taskID]);
    log("Action is associated with task %d", action->taskID);

    assert(task->plan.size() > 0);
    task->plan[0]->status = action->status;
    
    if (action->status == ABORTED) {
        task->executionStatus = ABORTED;
        overwriteWorkingMemory(activeTasks[task->id], task);
    }
    else if (action->status == FAILED) {
        if (task->executionRetries >= MAX_EXEC_RETRIES) {
            log("Action %s failed, retries exhausted.", action->name.c_str());
            task->executionStatus = FAILED;
            overwriteWorkingMemory(activeTasks[task->id], task);
            pyServer->notifyFailure(task, EXECUTION);
        }
        else {
            log("Action %s failed, replanning.", action->name.c_str());
            task->executionStatus = PENDING;
            //generateState(task);
            task->executionRetries++;
            overwriteWorkingMemory(activeTasks[task->id], task);
            dispatchPlanning(task, PLANNER_UPDATE_DELAY);
        }
    }
    else if (action->status == SUCCEEDED) {
        /*task->plan.erase(task->plan.begin());
        if (task->plan.size() > 0) {
            ActionPtr first_action = task->plan[0];
            first_action->status = PENDING;
            writeAction(first_action, task);
        }
        else {
            task->status = SUCCEEDED;
        }*/
        task->executionRetries = 0;
        //generateState(task);
        overwriteWorkingMemory(activeTasks[task->id], task);
        //pyServer->updateTask(task);
        dispatchPlanning(task, PLANNER_UPDATE_DELAY);
    }
}

void WMControl::newPercept(const cast::cdl::WorkingMemoryChange& wmc) {
    log("new percept");
    BeliefEntry entry;
    entry.address = wmc.address;
    entry.belief = getMemoryEntry<dBelief>(wmc.address);
    m_percepts.push_back(entry);
}        


void WMControl::stateChanged(const cast::cdl::WorkingMemoryChange& wmc) {
    log("state change...");
    if (wmc.operation == cast::cdl::ADD || wmc.operation == cast::cdl::OVERWRITE) {
        log("added/changed belief at %s@%s", wmc.address.id.c_str(), wmc.address.subarchitecture.c_str());
        try {
            BeliefEntry entry;
            entry.address = wmc.address;
            entry.belief = getMemoryEntry<dBelief>(wmc.address);
            m_currentState[wmc.address.id] = entry;
            log("%s->id = %s", wmc.address.id.c_str(), entry.belief->id.c_str());
        }
        catch(cast::DoesNotExistOnWMException) {
            log("%s vanished.", wmc.address.id.c_str());
            m_currentState.erase(wmc.address.id);
        }
    }
    else {
        log("deleted belief at %s", wmc.address.id.c_str());
        m_currentState.erase(wmc.address.id);
    }
    m_new_updates = true;

    timeval tval;
    gettimeofday(&tval, NULL);
    tval.tv_sec += BELIEF_TIMEOUT / 1000;
    tval.tv_usec += 1000 * (BELIEF_TIMEOUT % 1000);
    m_belief_activity_timeout = tval;
    log("belief timeout tval: %ld / %ld", tval.tv_sec, tval.tv_usec);

    m_lastUpdate = wmc.timestamp;
}


void WMControl::dispatchPlanning(PlanningTaskPtr& task, int msecs) {
    if (task->id != m_active_task_id) {
        log("Not triggering replanning for task %d, it is not the active task (%d is).", task->id, m_active_task_id);
        return;
    }

    timeval tval;
    gettimeofday(&tval, NULL);
    tval.tv_sec += msecs / 1000;
    tval.tv_usec += 1000 * (msecs % 1000);
    m_queue_mutex.lock();
    if (m_runqueue.find(task->id) != m_runqueue.end()) {
        if (later_than(m_runqueue[task->id], tval)) {
            //Task already in queue for a later time
            m_queue_mutex.unlock();
            return;
        }
    }
    timeval tval2;
    gettimeofday(&tval2, NULL);
    log("current tval: %ld / %ld", tval2.tv_sec, tval2.tv_usec);
    log("scheduled tval: %ld / %ld", tval.tv_sec, tval.tv_usec);
    m_runqueue[task->id] = tval;
    m_queue_mutex.unlock();
}



void WMControl::deliverPlan(int id, const ActionSeq& plan, const GoalSeq& goals) {
    log("Plan delivered");
    if (activeTasks.find(id) == activeTasks.end()) {
        log("Task %d not found.", id);
        return;
    }

    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[id]);

    double total_costs = 0;
    BOOST_FOREACH(ActionPtr action, plan) {
        total_costs += action->cost;
    }

    task->plan = plan;
    task->goals = goals;
    task->costs = total_costs;
    task->planningRetries = 0;
    task->planningStatus = SUCCEEDED;

    log("Task %d has costs %.2f.", task->id, task->costs);

    if (!task->executePlan) {
        log("Execution flag is not set.");
        overwriteWorkingMemory(activeTasks[id], task);
        return;
    }

    //remove task from pending queues
    m_queue_mutex.lock();
    m_waiting_tasks.erase(id);
    m_runqueue.erase(id);
    m_queue_mutex.unlock();

    if (plan.size() > 0) {
        ActionPtr first_action = plan[0];
        log("first action: %s", first_action->fullName.c_str());
        first_action->status = PENDING;
        task->executionStatus = INPROGRESS;
	//nah: don't change this, as ut should be handled by executor
        //task->executionStatus = PENDING;
        writeAction(first_action, task);
    }
    else {
        log("Task %d succeeded.", task->id);
        task->executionStatus = SUCCEEDED;
        overwriteWorkingMemory(activeTasks[id], task);
        activeTasks.erase(id);
    }

}


void WMControl::deliverPOPlan(int task_id, const POPlanPtr& po_plan) {
    log("delivering PO-plan.");

    POPlanMap* planmap = 0;
    if (po_plan->status == RUNNING) {
        planmap = &m_running_poplans;
        log("Plan is the current plan.");
    }
    else {
        planmap = &m_completed_poplans;
        log("Plan is the completed plan.");
    }

    if (planmap->find(task_id) == planmap->end()) {
        string wm_id = newDataID();
        addToWorkingMemory(wm_id, po_plan);
        (*planmap)[task_id] = wm_id;
    }
    else {
        overwriteWorkingMemory((*planmap)[task_id], po_plan);
    }
}



void WMControl::deliverHypotheses(int id, const BeliefSeq& hypotheses) {
    log("Hypotheses delivered");
    if (activeTasks.find(id) == activeTasks.end()) {
        log("Task %d not found.", id);
        BOOST_FOREACH(dBeliefPtr belief, hypotheses) {
            WorkingMemoryAddress wma;
            wma.id = belief->id;
            wma.subarchitecture = subarchitectureID();
            log("adding hypothesis %s", wma.id.c_str());
            addToWorkingMemory(wma, belief);
        }
        return;
    }

    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[id]);
    task->executionStatus = FAILED;
    task->hypotheses = vector<WorkingMemoryAddress>();
    BOOST_FOREACH(dBeliefPtr belief, hypotheses) {
        WorkingMemoryAddress wma;
        wma.id = belief->id;
        wma.subarchitecture = subarchitectureID();
        log("adding hypothesis %s", wma.id.c_str());
        addToWorkingMemory(wma, belief);
        task->hypotheses.push_back(wma);
    }

    overwriteWorkingMemory(activeTasks[id], task);
}

void WMControl::updateBeliefState(const BeliefEntrySeq& beliefs) {
    BOOST_FOREACH(BeliefEntry entry, beliefs) {
        dBeliefPtr bel = entry.belief;
        try {
            if (bel->id == "temporary") {
                CASTTime time = getCASTTime();
                framing::CASTTemporalIntervalPtr interval = new framing::CASTTemporalInterval();
                interval->start = time;
                interval->end = time;
                // framing::SimpleSpatioTemporalFramePtr frame = 
                bel->frame = new framing::SpatioTemporalFrame("here", interval, 1.0);
                bel->id = newDataID();

                WorkingMemoryAddress wma;
                wma.id = bel->id;
                wma.subarchitecture = BINDER_SA;
                addToWorkingMemory(wma, bel);
                log("added belief  %s to working memory", bel->id.c_str());
            }
            else {
                WorkingMemoryAddress wma = entry.address;
                if (existsOnWorkingMemory(wma)) {
                    dBeliefPtr oldBelief = getMemoryEntry<dBelief>(wma);
				
                    history::CASTBeliefHistoryPtr hist = dynamic_cast<history::CASTBeliefHistory*>(bel->hist.get());
                    history::CASTBeliefHistoryPtr oldHist = dynamic_cast<history::CASTBeliefHistory*>(oldBelief->hist.get());
                    if (hist && oldHist) {
                        hist->offspring = oldHist->offspring;
                        overwriteWorkingMemory(wma, bel);
                        log("existing belief  %s updated on the working memory", bel->id.c_str());
                    }
                    else {
                        log("ERROR: no history found.");
                    }
                }
            }
        //     WorkingMemoryAddress wma;
        //     wma.id = bel->id;
        //     wma.subarchitecture = BINDER_SA;
        //     overwriteWorkingMemory(wma, bel);
        // }
		} 
		catch (cast::UnknownSubarchitectureException e) {
			log("ERROR: problem with the subarchitecture identifier for the binder");
		} 
        catch (cast::ConsistencyException e) {
            log("Consistency exception when trying to update belief %s", bel->id.c_str());
        }
    }
}

void WMControl::updateStatus(int id, Completion status) {
    log("entering method updateStatus()");
    assert(activeTasks.find(id) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[id]);
    task->planningStatus = status;
    log("Settings planning status of task %d to %d", id, status);
    if (status == ABORTED) {
        log("Planning aborted, setting status of task %d to %d", id, status);
        task->executionStatus = status;
    }
    else if (status == SUCCEEDED) {
            task->planningRetries = 0;
    }
    else if (status == FAILED) {
        if (task->planningRetries >= MAX_PLANNING_RETRIES && task->executionStatus != FAILED) {
            log("Planning failed %d times, setting status of task %d to %d", MAX_PLANNING_RETRIES, id, status);
            task->executionStatus = FAILED;
            overwriteWorkingMemory(activeTasks[id], task);
            pyServer->notifyFailure(task, PLANNING);
            return;
        }
        else if (task->id != m_active_task_id) {
            log("Planning failed for non-active task.");
            task->executionStatus = FAILED;
        } else {
            log("Planning failed, waiting and replanning.");
            task->planningStatus = PENDING;
            task->planningRetries++;
            overwriteWorkingMemory(activeTasks[id], task);
            //pyServer->updateTask(task);
            dispatchPlanning(task, REPLAN_DELAY);
            return;
        }
    }
    overwriteWorkingMemory(activeTasks[id], task);
}


void WMControl::waitForChanges(int id, int timeout) {
    assert(activeTasks.find(id) != activeTasks.end());
    timeval tval;
    gettimeofday(&tval, NULL);
    tval.tv_sec += timeout / 1000;
    tval.tv_usec += 1000 * (timeout % 1000);
    m_queue_mutex.lock();
    log("set timeout to: %ld / %ld", tval.tv_sec, tval.tv_usec);
    m_waiting_tasks[id] = tval;
    m_queue_mutex.unlock();
}

bool WMControl::queryGoal(const string& goal) {
    vector<BeliefEntry> state;
    for (BeliefMap::const_iterator i=m_currentState.begin(); i != m_currentState.end(); ++i) {
        state.push_back(i->second);
    }
    return pyServer->queryGoal(state, goal);
}

void WMControl::writeAction(ActionPtr& action, PlanningTaskPtr& task) {
    string id = task->firstActionID;
    action->taskID = task->id;

    if (id == "") {
        id = newDataID();
        task->firstActionID = id;
        addToWorkingMemory(id, action);
        overwriteWorkingMemory(activeTasks[task->id], task);
    }
    else {
        overwriteWorkingMemory(activeTasks[task->id], task);
        overwriteWorkingMemory(id, action);
    }
}

void WMControl::verbalise(const string& phrase) {
    string id = newDataID();
    PlannerVerbalisationPtr verb = new PlannerVerbalisation(phrase);
    addToWorkingMemory(id, verb);
    log("saying: %s", phrase.c_str());
}

WMControl::InternalCppServer::InternalCppServer(WMControl* Parent) {
    parent = Parent;
}

void WMControl::InternalCppServer::deliverPlan(int id, const ActionSeq& plan, const GoalSeq& goals, const Ice::Current&) {
    parent->deliverPlan(id, plan, goals);
}

void WMControl::InternalCppServer::deliverPOPlan(int id, const POPlanPtr& poplan, const Ice::Current&) {
    parent->deliverPOPlan(id, poplan);
}

void WMControl::InternalCppServer::deliverHypotheses(int id, const BeliefSeq& hypotheses, const Ice::Current&) {
    parent->deliverHypotheses(id, hypotheses);
}

void WMControl::InternalCppServer::updateBeliefState(const BeliefEntrySeq& beliefs, const Ice::Current&) {
    parent->updateBeliefState(beliefs);
}

void WMControl::InternalCppServer::updateStatus(int id, Completion status, const Ice::Current&) {
    parent->updateStatus(id, status);
}

void WMControl::InternalCppServer::setChangeFilter(int id, const StateChangeFilterPtr& filter, const Ice::Current&) {

}

void WMControl::InternalCppServer::waitForChanges(int id, int timeout, const Ice::Current&) {
    parent->waitForChanges(id, timeout);
}

bool WMControl::InternalCppServer::queryGoal(const string& goal, const Ice::Current&) {
    return parent->queryGoal(goal);
}

void WMControl::InternalCppServer::verbalise(const string& phrase, const Ice::Current&) {
    return parent->verbalise(phrase);
}

WorkingMemoryAddress WMControl::InternalCppServer::newAddress(const Ice::Current&) {
    WorkingMemoryAddress wma;
    wma.id = parent->newDataID();
    wma.subarchitecture = parent->subarchitectureID();
    return wma;
}

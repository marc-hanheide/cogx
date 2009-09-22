#include "WMControl.hpp"
#include "BinderEssentials.hpp"

#include <iostream>
#include <sstream>
#include <cassert>

using namespace std;
using namespace binder::autogen::core;

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new WMControl();
    }
}

static const int MAX_PLANNING_RETRIES = 1;
static const int MAX_EXEC_RETRIES = 1;
static const int REPLAN_DELAY = 3000;

void WMControl::configure(const cast::cdl::StringMap& _config, const Ice::Current& _current) {
    m_lastUpdate = 0;
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
}

void WMControl::start() {
    log("Planner WMControl: initializing");
    addChangeFilter(cast::createLocalTypeFilter<autogen::Planner::PlanningTask>(cast::cdl::ADD), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::receivePlannerCommands));

    addChangeFilter(cast::createLocalTypeFilter<Action>(cast::cdl::OVERWRITE), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::actionChanged));

    addChangeFilter(cast::createGlobalTypeFilter<UnionConfiguration>(),
            new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::stateChanged));

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
    while (true) {
        m_queue_mutex.lock();
        if (!m_runqueue.empty()) {
            log("planning is scheduled");
            timeval tval;
            gettimeofday(&tval, NULL);
            std::map<int, timeval>::iterator it=m_runqueue.begin();
            std::vector<int> executed;
            for(; it != m_runqueue.end(); ++it) {
                if ((it->second.tv_sec < tval.tv_sec) 
                    || ((it->second.tv_sec == tval.tv_sec) 
                        && (it->second.tv_usec < tval.tv_usec))) {
                    log("accessing wm");
                    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[it->first].address);
                    task->state = m_currentState;
                    pyServer->updateTask(task);
                    log("returning..:");
                    executed.push_back(task->id);
                }
            }
            for (std::vector<int>::iterator it=executed.begin(); it != executed.end(); ++it) {
                m_runqueue.erase(*it);
            }
        }
        m_queue_mutex.unlock();
        sleepComponent(1000);
    }
}

static int TASK_ID = 0;

void WMControl::receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc) {
    log("Planner WMControl: new PlanningTask received:");
    TASK_ID++;

    autogen::Planner::PlanningTaskPtr task = getMemoryEntry<autogen::Planner::PlanningTask>(wmc.address);
    log(task->goal);

    generateInitialState(task);

    task->id = TASK_ID;
    task->plan = vector<ActionPtr>();
    task->executionStatus = PENDING;
    task->firstActionID = "";
    task->planningStatus = PENDING;
    task->planningRetries = 0;
    task->executionRetries = 0;
    activeTasks[task->id] = wmc;

    overwriteWorkingMemory(wmc.address, task);
    pyServer->registerTask(task);

    //TODO: Store the PlanningTaskPtr with it's taskID till deliverPlan(taskID) is called.
}

void WMControl::generateInitialState(autogen::Planner::PlanningTaskPtr& task) {
    log("Planner WMControl:: generating Initial State");

    vector<cast::CASTData<UnionConfiguration> > configs;
    getMemoryEntriesWithData<UnionConfiguration>(configs, "binder");

    if (configs.size() == 0) {
        log("Planner WMControl:: No union configurations on binder. Doing nothing.");
        return;
    }
    log("Planner WMControl:: %d union configurations on binder. Using first one.", configs.size());

    binder::autogen::core::UnionConfigurationPtr config = configs[0].getData();

    task->state = vector<UnionPtr>();
    for (UnionSequence::iterator i = config->includedUnions.begin(); 
         i < config->includedUnions.end() ; ++i) {
        task->state.push_back(*i);
    }
}

void WMControl::actionChanged(const cast::cdl::WorkingMemoryChange& wmc) {
    ActionPtr action = getMemoryEntry<Action>(wmc.address);

    if (action->status == PENDING) { // We just added this action ourselves
        return;
    }
    log("Action %s changed to status %d", action->name.c_str(), action->status);

    assert(activeTasks.find(action->taskID) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[action->taskID].address);

    assert(task->plan.size() > 0);
    task->plan[0]->status = action->status;
    
    if (action->status == ABORTED) {
        task->executionStatus = ABORTED;
        overwriteWorkingMemory(activeTasks[task->id].address, task);
    }
    else if (action->status == FAILED) {
        if (task->executionRetries >= MAX_EXEC_RETRIES) {
            log("Action %s failed, retries exhausted.", action->name.c_str());
            task->executionStatus = FAILED;
        }
        else {
        log("Action %s failed, replanning.", action->name.c_str());
             task->executionStatus = PENDING;
             task->state = m_currentState;
             task->executionRetries++;
        }
        overwriteWorkingMemory(activeTasks[task->id].address, task);
        pyServer->updateTask(task);
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
        task->state = m_currentState;
        overwriteWorkingMemory(activeTasks[task->id].address, task);
        pyServer->updateTask(task);
    }
    //dispatchPlanning(task, 0);
}

void WMControl::stateChanged(const cast::cdl::WorkingMemoryChange& wmc) {
    UnionConfigurationPtr config = getMemoryEntry<UnionConfiguration>(wmc.address);
    log("Recevied state change...");

    long timestamp = 0;
    m_currentState.clear();
    vector<UnionPtr> changed;
    for (UnionSequence::iterator i = config->includedUnions.begin(); 
         i < config->includedUnions.end() ; ++i) {
        m_currentState.push_back(*i);
        if ((*i)->timeStamp > m_lastUpdate) {
            changed.push_back(*i);
        }
        if ((*i)->timeStamp > timestamp) {
            timestamp = (*i)->timeStamp;
        }
    }

    if (m_continual_state_updates) {
        for (std::map<int,cast::cdl::WorkingMemoryChange>::iterator it=activeTasks.begin(); it != activeTasks.end(); ++it) {
            int id = it->first;
            StateChangeFilterPtr* filter = 0;
            std::map<int, StateChangeFilterPtr>::iterator f_iter = m_stateFilters.find(id);
            if (f_iter != m_stateFilters.end()) {
                filter = &(f_iter->second);
            }
            sendStateChange(id, changed, timestamp, filter);
        }
    }

    m_lastUpdate = timestamp;

}


void WMControl::sendStateChange(int id, std::vector<UnionPtr>& changedUnions, long newTimeStamp, StateChangeFilterPtr* filter) {
    assert(activeTasks.find(id) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[id].address);
    /*if (task->planningStatus == INPROGRESS) {
        println("Task %d is still planning. Don't send updates.", id);
        return;
    }*/

    if (filter) {

    }

    log("Sending state update for task %d", id);
    task->state = m_currentState;
    overwriteWorkingMemory(activeTasks[id].address, task);
    //pyServer->updateTask(task);
    dispatchPlanning(task, 0);
}

void WMControl::dispatchPlanning(PlanningTaskPtr& task, int msecs) {
    timeval tval;
    gettimeofday(&tval, NULL);
    tval.tv_sec += msecs / 1000;
    tval.tv_usec += 1000 * (msecs % 1000);
    m_queue_mutex.lock();
    if (m_runqueue.find(task->id) != m_runqueue.end()) {
        if ((m_runqueue[task->id].tv_sec > tval.tv_sec) 
            || ((m_runqueue[task->id].tv_sec == tval.tv_sec) 
                && (m_runqueue[task->id].tv_usec > tval.tv_usec))) {
            //Task already in queue for a later time
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


void WMControl::deliverPlan(int id, const ActionSeq& plan) {
    log("Plan delivered");
    assert(activeTasks.find(id) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[id].address);
    task->plan = plan;
    task->planningRetries = 0;
    task->planningStatus = SUCCEEDED;

    if (plan.size() > 0) {
        ActionPtr first_action = plan[0];
        first_action->status = PENDING;
        writeAction(first_action, task);
	//nah: don't change this, as ut should be handled by executor
        //task->executionStatus = PENDING;
        overwriteWorkingMemory(activeTasks[id].address, task);
    }
    else {
        log("Task %d succeeded.", task->id);
        task->executionStatus = SUCCEEDED;
        overwriteWorkingMemory(activeTasks[id].address, task);
        activeTasks.erase(id);
    }

}


void WMControl::updateStatus(int id, Completion status) {
    assert(activeTasks.find(id) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[id].address);
    task->planningStatus = status;
    log("Settings planning status of task %d to %d", id, status);
    if (status == ABORTED) {
        log("Planning aborted, setting status of task %d to %d", id, status);
        task->executionStatus = status;
    }
    else if (status == FAILED) {
        if (task->planningRetries >= MAX_PLANNING_RETRIES) {
            log("Planning failed %d times, setting status of task %d to %d", MAX_PLANNING_RETRIES, id, status);
            task->executionStatus = FAILED;
        }
        else {
            log("Planning failed, waiting and replanning.");
            task->planningStatus = PENDING;
            task->planningRetries++;
            overwriteWorkingMemory(activeTasks[id].address, task);
            //pyServer->updateTask(task);
            dispatchPlanning(task, REPLAN_DELAY);
            return;
        }
    }
    overwriteWorkingMemory(activeTasks[id].address, task);
}


void WMControl::setChangeFilter(int id, const StateChangeFilterPtr& filter) {
    m_stateFilters[id] = filter;
}

void WMControl::writeAction(ActionPtr& action, PlanningTaskPtr& task) {
    string id = task->firstActionID;
    action->taskID = task->id;

    if (id == "") {
        id = newDataID();
        task->firstActionID = id;
        addToWorkingMemory(id, action);
    }
    else {
        overwriteWorkingMemory(id, action);
    }
}


WMControl::InternalCppServer::InternalCppServer(WMControl* Parent) {
    parent = Parent;
}

void WMControl::InternalCppServer::deliverPlan(int id, const ActionSeq& plan, const Ice::Current&) {
    parent->deliverPlan(id, plan);
}

void WMControl::InternalCppServer::updateStatus(int id, Completion status, const Ice::Current&) {
    parent->updateStatus(id, status);
}

void WMControl::InternalCppServer::setChangeFilter(int id, const StateChangeFilterPtr& filter, const Ice::Current&) {
    parent->setChangeFilter(id, filter);
}

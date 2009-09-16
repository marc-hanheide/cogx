#include "WMControl.hpp"
#include "BinderEssentials.hpp"

#include <iostream>
#include <sstream>
#include <cassert>

using namespace std;

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new WMControl();
    }
}

void WMControl::configure(const cast::cdl::StringMap& _config, const Ice::Current& _current) {
    cast::ManagedComponent::configure(_config, _current);

    cast::cdl::StringMap::const_iterator it = _config.begin();
    it = _config.find("--server");
    if (it != _config.end()) {
        m_python_server = it->second;
    }
    else {
        m_python_server = "PlannerPythonServer";
    }
}

void WMControl::start() {
    println("Planner WMControl: initializing");
    addChangeFilter(cast::createLocalTypeFilter<autogen::Planner::PlanningTask>(cast::cdl::ADD), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::receivePlannerCommands));

    addChangeFilter(cast::createLocalTypeFilter<Action>(cast::cdl::OVERWRITE), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::actionChanged));

    connectToPythonServer();
}

void WMControl::connectToPythonServer() {
    println("Planner WMControl: connecting to Python Server");
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
    println("Planner WMControl: running");
}

static int TASK_ID = 0;

void WMControl::receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc) {
    println("Planner WMControl: new PlanningTask received:");
    TASK_ID++;

    autogen::Planner::PlanningTaskPtr task = getMemoryEntry<autogen::Planner::PlanningTask>(wmc.address);
    println(task->goal);

    generateInitialState(task);

    task->id = TASK_ID;
    task->plan = vector<ActionPtr>();
    task->status = PENDING;
    task->planningStatus = PENDING;
    activeTasks[task->id] = wmc;

    overwriteWorkingMemory(wmc.address, task);
    pyServer->registerTask(task);

    //TODO: Store the PlanningTaskPtr with it's taskID till deliverPlan(taskID) is called.
}

void WMControl::generateInitialState(autogen::Planner::PlanningTaskPtr& task) {
    println("Planner WMControl:: generating Initial State");

    vector<cast::CASTData<binder::autogen::core::UnionConfiguration> > configs;
    getMemoryEntriesWithData<binder::autogen::core::UnionConfiguration>(configs, "binder");

    if (configs.size() == 0) {
        println("Planner WMControl:: No union configurations on binder. Doing nothing.");
        return;
    }
    println("Planner WMControl:: %d union configurations on binder. Using first one.", configs.size());

    binder::autogen::core::UnionConfigurationPtr config = configs[0].getData();

    task->state = vector<binder::autogen::core::UnionPtr>();
    for (binder::autogen::core::UnionSequence::iterator i = config->includedUnions.begin(); 
         i < config->includedUnions.end() ; ++i) {
        task->state.push_back(*i);
    }
}

void WMControl::actionChanged(const cast::cdl::WorkingMemoryChange& wmc) {
    ActionPtr action = getMemoryEntry<Action>(wmc.address);
    println("Action %s changed to status %d", action->name.c_str(), action->status);

    assert(activeTasks.find(action->taskID) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[action->taskID].address);
    
    if (action->status == ABORTED || action->status == FAILED) {
        task->status = action->status;
        overwriteWorkingMemory(activeTasks[task->id].address, task);
    }
    else if (action->status == SUCCEEDED) {
        task->plan.erase(task->plan.begin());
        if (task->plan.size() > 0) {
            ActionPtr first_action = task->plan[0];
            first_action->status = PENDING;
            first_action->taskID = task->id;
            string id = newDataID();
            addToWorkingMemory(id, first_action);
        }
        else {
            task->status = SUCCEEDED;
        }
        overwriteWorkingMemory(activeTasks[task->id].address, task);
    }
}

void WMControl::deliverPlan(int id, const ActionSeq& plan) {
    println("Plan delivered");
    assert(activeTasks.find(id) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[id].address);
    task->plan = plan;
    task->planningStatus = SUCCEEDED;

    if (plan.size() > 0) {
        ActionPtr first_action = plan[0];
        first_action->status = PENDING;
        first_action->taskID = task->id;
        string id = newDataID();
        addToWorkingMemory(id, first_action);
        task->status = INPROGRESS;
    }
    else {
        task->status = SUCCEEDED;
    }

    overwriteWorkingMemory(activeTasks[id].address, task);
}

void WMControl::updateStatus(int id, Completion status) {
    assert(activeTasks.find(id) != activeTasks.end());
    PlanningTaskPtr task = getMemoryEntry<PlanningTask>(activeTasks[id].address);
    task->planningStatus = status;
    println("Settings planning status of task %d to %d", id, status);
    if (status == ABORTED || status == FAILED) {
        println("Planning failed, setting status of task %d to %d", id, status);
        task->status = status;
    }
    overwriteWorkingMemory(activeTasks[id].address, task);
}

void WMControl::setChangeFilter(int id, const StateChangeFilterPtr& filter) {

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

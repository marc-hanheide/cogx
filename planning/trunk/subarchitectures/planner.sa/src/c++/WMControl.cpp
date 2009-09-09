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

void WMControl::start() {
    println("Planner WMControl: initializing");
    addChangeFilter(cast::createLocalTypeFilter<autogen::Planner::PlanningTask>(cast::cdl::ADD), 
		    new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::receivePlannerCommands));

    connectToPythonServer();
}

void WMControl::connectToPythonServer() {
    println("Planner WMControl: connecting to Python Server");
    try	{
		pyServer = getIceServer<autogen::Planner::PythonServer>("PlannerPythonServer");
	// Ice::Identity idty;
	// idty.name = "PlannerPythonServer";
	// //idty.category = "PythonServer.PythonServerI";
	// idty.category = "PythonServer";

	// ostringstream tmp;
	// tmp << getCommunicator()->identityToString(idty) << ":default -h 127.0.0.1 -p 10411";

	// Ice::ObjectPrx base = getCommunicator()->stringToProxy(tmp.str());
	// pyServer = autogen::Planner::PythonServerPrx::checkedCast(base);
    }
    catch (const Ice::Exception& ex) {
	cerr << ex << endl;
    }
    catch (const char* msg) {
	cerr << msg << endl;
    }

    Ice::ObjectPrx basePrx(getObjectAdapter()->addFacet(new InternalCppServer(this), getIceIdentity(), "PlannerInternalCppServer")); 
    
    autogen::Planner::CppServerPrx clientPrx = autogen::Planner::CppServerPrx::uncheckedCast(basePrx);

    if(!clientPrx)
	println("Planner WMControl: error while initializing python server");
    else
	pyServer->registerClient(clientPrx);
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
    activeTasks[task->id] = wmc;

    pyServer->registerTask(task);

    //TODO: Store the PlanningTaskPtr with it's taskID till deliverPlan(taskID) is called.
}

void WMControl::generateInitialState(autogen::Planner::PlanningTaskPtr& task) {
    sleep(5);
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
    //task->state.push_back(unions[0].getData());
    //task->objects = stateProvider[0].getData()->objects;
    //task->state = stateProvider[0].getData()->state;

}

void WMControl::deliverPlan(const autogen::Planner::PlanningTaskPtr& task) {
    println("Plan delivered");
    assert(activeTasks.find(task->id) != activeTasks.end());

    overwriteWorkingMemory(activeTasks[task->id].address,task);
}

WMControl::InternalCppServer::InternalCppServer(WMControl* Parent) {
    parent = Parent;
}

void WMControl::InternalCppServer::deliverPlan(const autogen::Planner::PlanningTaskPtr& task, const Ice::Current&) {
    parent->deliverPlan(task);
}


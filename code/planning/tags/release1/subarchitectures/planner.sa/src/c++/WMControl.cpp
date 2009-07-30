#include "WMControl.hpp"
#include "FakeBinderData.hpp"
//#include <CASTData.hpp>

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
	Ice::Identity idty;
	idty.name = "ComponentFactory";
	idty.category = "ComponentFactory";

	ostringstream tmp;
	tmp << getCommunicator()->identityToString(idty) << ":default -h 127.0.0.1 -p 10411";

	Ice::ObjectPrx base = getCommunicator()->stringToProxy(tmp.str());
	cast::interfaces::ComponentFactoryPrx cmpFac = cast::interfaces::ComponentFactoryPrx::checkedCast(base);

	cast::interfaces::CASTComponentPrx comp = cmpFac->newComponent("PlannerPythonServer","PythonServer.PythonServerI");
	pyServer = autogen::Planner::PythonServerPrx::checkedCast(comp);
    
	if (!cmpFac)
	    throw "Invalid proxy";
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
    println("Planner WMControl:: generating Initial State");

    vector<cast::CASTData<autogen::FakeBinderData::FakeBinderActualStateProvider> > stateProvider;
    getMemoryEntriesWithData<autogen::FakeBinderData::FakeBinderActualStateProvider>(stateProvider, "fakebinder.sa");

    task->objects = stateProvider[0].getData()->objects;
    task->state = stateProvider[0].getData()->state;

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


#include "DummySA.hpp"
#include "Planner.hpp"

using namespace std;

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new DummySA();
  }
}

void DummySA::runComponent() {
    println("DummySA: running");
    string id = newDataID();

    autogen::Planner::PlanningTaskPtr plan = new autogen::Planner::PlanningTask();
    plan->goal = "(pos coffee : living_room)";

    addChangeFilter(cast::createAddressFilter(id,"planner.sa",cast::cdl::OVERWRITE),
		    new cast::MemberFunctionChangeReceiver<DummySA>(this, &DummySA::planGenerated));
    addToWorkingMemory(id,"planner.sa", plan);
}

void DummySA::planGenerated(const cast::cdl::WorkingMemoryChange& wmc) {
    println("DummySA: plan received:");

    autogen::Planner::PlanningTaskPtr planData = getMemoryEntry<autogen::Planner::PlanningTask>(wmc.address);
    println(planData->plan);
}

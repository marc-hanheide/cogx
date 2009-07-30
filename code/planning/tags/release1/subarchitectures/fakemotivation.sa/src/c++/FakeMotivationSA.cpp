#include "FakeMotivationSA.hpp"
#include "Planner.hpp"

using namespace std;

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new FakeMotivationSA();
  }
}

void FakeMotivationSA::runComponent() {
    println("FakeMotivationSA: running");
    string id = newDataID();

    autogen::Planner::PlanningTaskPtr plan = new autogen::Planner::PlanningTask();
    plan->goal = "(pos coffee : living_room)";

    addChangeFilter(cast::createAddressFilter(id,"planner.sa",cast::cdl::OVERWRITE),
		    new cast::MemberFunctionChangeReceiver<FakeMotivationSA>(this, &FakeMotivationSA::planGenerated));
    addToWorkingMemory(id,"planner.sa", plan);
}

void FakeMotivationSA::planGenerated(const cast::cdl::WorkingMemoryChange& wmc) {
    println("FakeMotivationSA: plan received:");

    autogen::Planner::PlanningTaskPtr planData = getMemoryEntry<autogen::Planner::PlanningTask>(wmc.address);
    println(planData->plan);
}

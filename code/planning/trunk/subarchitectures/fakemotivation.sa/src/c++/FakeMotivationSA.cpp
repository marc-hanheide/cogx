#include "FakeMotivationSA.hpp"
#include "Planner.hpp"

using namespace std;

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new FakeMotivationSA();
  }
}

FakeMotivationSA::FakeMotivationSA() : 
    m_goal ("(forall (?n - node) (= (place_type ?n) place))") 
{ }

void FakeMotivationSA::configure(const cast::cdl::StringMap& _config, const Ice::Current& _current) {
    cast::cdl::StringMap::const_iterator it = _config.begin();
    it = _config.find("--goal");
    if (it != _config.end()) {
        m_goal = it->second;
    }
}

void FakeMotivationSA::runComponent() {
    println("FakeMotivationSA: running");
    string id = newDataID();

    autogen::Planner::PlanningTaskPtr plan = new autogen::Planner::PlanningTask();
    plan->goal = m_goal;

    addChangeFilter(cast::createAddressFilter(id,"planner.sa",cast::cdl::OVERWRITE),
		    new cast::MemberFunctionChangeReceiver<FakeMotivationSA>(this, &FakeMotivationSA::planGenerated));
    addToWorkingMemory(id,"planner.sa", plan);
}

void FakeMotivationSA::planGenerated(const cast::cdl::WorkingMemoryChange& wmc) {
    println("FakeMotivationSA: plan received:");

    autogen::Planner::PlanningTaskPtr planData = getMemoryEntry<autogen::Planner::PlanningTask>(wmc.address);
    println(planData->plan);
}

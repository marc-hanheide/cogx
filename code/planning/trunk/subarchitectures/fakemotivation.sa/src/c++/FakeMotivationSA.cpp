#include "FakeMotivationSA.hpp"
#include "Planner.hpp"

using namespace std;

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new FakeMotivationSA();
  }
}

FakeMotivationSA::FakeMotivationSA() : 
    m_goal ("(forall (?p - place) (= (explored ?p) true))") 
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
    sleep(10);
    string id = newDataID();

    autogen::Planner::PlanningTaskPtr plan = new autogen::Planner::PlanningTask();
    plan->goal = m_goal;

    addChangeFilter(cast::createAddressFilter(id,"planner.sa",cast::cdl::OVERWRITE),
		    new cast::MemberFunctionChangeReceiver<FakeMotivationSA>(this, &FakeMotivationSA::planGenerated));
    addToWorkingMemory(id,"planner.sa", plan);
}

void FakeMotivationSA::planGenerated(const cast::cdl::WorkingMemoryChange& wmc) {
    autogen::Planner::PlanningTaskPtr planData = getMemoryEntry<autogen::Planner::PlanningTask>(wmc.address);
    if (planData->planningStatus == autogen::Planner::SUCCEEDED) {
        println("FakeMotivationSA: plan received:");

        for (autogen::Planner::ActionSeq::iterator it=planData->plan.begin(); it != planData->plan.end(); ++it) {
            println((*it)->fullName);
        }
    }
}

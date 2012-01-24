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
    cast::ManagedComponent::configure(_config, _current);
    
    cast::cdl::StringMap::const_iterator it = _config.begin();
    it = _config.find("--goal");
    if (it != _config.end()) {
        m_goal = it->second;
    }
}

void FakeMotivationSA::runComponent() {
    log("FakeMotivationSA: running");
    sleepComponent(3000);
    string id = newDataID();
    log("step1");
    autogen::Planner::PlanningTaskPtr plan = new autogen::Planner::PlanningTask();
    autogen::Planner::GoalPtr goal = new autogen::Planner::Goal();
    goal->importance = -1;
    goal->goalString = m_goal;
    goal->isInPlan = false;

    plan->goals.push_back(goal);

    plan->executionStatus = autogen::Planner::PENDING;
    plan->planningStatus = autogen::Planner::PENDING;
    log("step2");

    addChangeFilter(cast::createAddressFilter(id,"planner.sa",cast::cdl::OVERWRITE),
		    new cast::MemberFunctionChangeReceiver<FakeMotivationSA>(this, &FakeMotivationSA::planGenerated));
    log("step3");
    addToWorkingMemory(id, plan);
    log("step4");
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

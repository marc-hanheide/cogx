#include "DummySA.hpp"
#include "PlannerData.hpp"

using namespace std;

extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new DummySA();
  }
}

void DummySA::runComponent()
{
  println("DummySA: running");
  string id = newDataID();
  PlannerData::PlannerCommandPtr plan = new PlannerData::PlannerCommand();
  addChangeFilter(cast::createAddressFilter(id,"planner.sa",cast::cdl::OVERWRITE), new cast::MemberFunctionChangeReceiver<DummySA>(this, &DummySA::planGenerated));
  addToWorkingMemory(id,"planner.sa", plan);
}

void DummySA::planGenerated(const cast::cdl::WorkingMemoryChange& wmc)
{
  println("DummySA: plan received:");

  PlannerData::PlannerCommandPtr planData = getMemoryEntry<PlannerData::PlannerCommand>(wmc.address);
  println(planData->plan);
}

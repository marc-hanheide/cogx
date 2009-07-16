#include "WMControl.hpp"

#include <iostream>

using namespace std;

extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new WMControl();
  }
}

void WMControl::start()
{
  println("Planner WMControl: initializing");
  //addChangeFilter(cast::createLocalTypeFilter<autogen::Planner::PlannerCommand>(cast::cdl::ADD), new cast::MemberFunctionChangeReceiver<WMControl>(this, &WMControl::receivePlannerCommands));

  connectToPythonServer();
}

void WMControl::connectToPythonServer()
{
  println("Planner WMControl: connecting to Python Server");
  try
  {
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
  catch (const Ice::Exception& ex)
  {
    cerr << ex << endl;
  }
  catch (const char* msg)
  {
    cerr << msg << endl;
  }

  Ice::ObjectPrx basePrx(getObjectAdapter()->addFacet(new InternalCppServer(this), getIceIdentity(), "PlannerInternalCppServer")); 

  autogen::Planner::CppServerPrx clientPrx = autogen::Planner::CppServerPrx::uncheckedCast(basePrx);

  if(!clientPrx)
    println("Planner WMControl: error while initializing python server");
  else
    pyServer->registerClient(clientPrx);
}

void WMControl::runComponent()
{
  println("Planner WMControl: running");

  //I'm not sure we even need this, since there'll probably be only reactive actions that follow certain wm-changes
}

void WMControl::receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc)
{
  println("Planner WMControl: new PlannerCommand received");
  
  //autogen::Planner::PlannerCommandPtr planData = getMemoryEntry<autogen::Planner::PlannerCommand>(wmc.address);

  //int taskID = pyServer->addTask(planData->task);

  //TODO: Store the PlannerCommandPtr with it's taskID till deliverPlan(taskID) is called.
}

void WMControl::deliverPlan(int taskID)
{
  println("Plan delivered");
  //TODO: Get the PlannerCommanPtr that's stored with taskID , then overwrite the WorkingMemory with that plan
  //planData->plan = "Well that's some plan, ain't it?";
  //overwriteWorkingMemory(wmc.address,planData);
}

WMControl::InternalCppServer::InternalCppServer(WMControl* Parent)
{
  parent = Parent;
}

void WMControl::InternalCppServer::deliverPlan(int taskID, const Ice::Current&)
{
  parent->deliverPlan(taskID);
}


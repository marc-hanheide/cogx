#ifndef WM_CONTROL_HPP_
#define WM_CONTROL_HPP_

#include <cast/architecture.hpp>
#include <Ice/Ice.h>
#include "Planner.hpp"

class WMControl : public cast::ManagedComponent
{
 public:
  virtual ~WMControl() {}

 protected:
  autogen::Planner::PythonServerPrx pyServer;

  virtual void start();
  virtual void runComponent();

  void connectToPythonServer();
  void deliverPlan(const autogen::Planner::PlanningTaskPtr& task);

  void receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc);

  class InternalCppServer : public autogen::Planner::CppServer
  {
   public:
    InternalCppServer(WMControl* Parent);
    virtual void deliverPlan(const autogen::Planner::PlanningTaskPtr& task, const Ice::Current&);

   protected:
    WMControl* parent;
  };
};

#endif

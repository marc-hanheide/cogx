#ifndef WM_CONTROL_HPP_
#define WM_CONTROL_HPP_

#include <cast/architecture.hpp>
#include <Ice/Ice.h>
#include "Planner.hpp"
#include <map>

class MainWindow;

class WMControl : public cast::ManagedComponent {
public:
    virtual ~WMControl() {}

protected:
    MainWindow* gui;
    autogen::Planner::PythonServerPrx pyServer;
    std::map<int,cast::cdl::WorkingMemoryChange> activeTasks;

    virtual void start();
    virtual void runComponent();

    void connectToPythonServer();
    void generateInitialState(autogen::Planner::PlanningTaskPtr& task);
    void deliverPlan(const autogen::Planner::PlanningTaskPtr& task);
    void receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc);

    class InternalCppServer : public autogen::Planner::CppServer {
    public:
	InternalCppServer(WMControl* Parent);
	virtual void deliverPlan(const autogen::Planner::PlanningTaskPtr& task, const Ice::Current&);

    protected:
	WMControl* parent;
    };
};

#endif

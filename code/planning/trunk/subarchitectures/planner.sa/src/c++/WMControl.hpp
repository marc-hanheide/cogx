#ifndef WM_CONTROL_HPP_
#define WM_CONTROL_HPP_

#include <cast/architecture.hpp>
#include <Ice/Ice.h>
#include "Planner.hpp"
#include <map>

using namespace autogen::Planner;

class WMControl : public cast::ManagedComponent {
public:
    virtual ~WMControl() {}

protected:
    PythonServerPrx pyServer;
    std::map<int,cast::cdl::WorkingMemoryChange> activeTasks;

    virtual void configure(const cast::cdl::StringMap& _config, const Ice::Current& _current);
    virtual void start();
    virtual void runComponent();

    void connectToPythonServer();
    void generateInitialState(PlanningTaskPtr& task);
    //void deliverPlan(const autogen::Planner::PlanningTaskPtr& task);
    void deliverPlan(int id, const ActionSeq& plan);
    void updateStatus(int id, Completion status);
    void setChangeFilter(int id, const StateChangeFilterPtr& filter);

    void receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc);
    void actionChanged(const cast::cdl::WorkingMemoryChange& wmc);

    class InternalCppServer : public CppServer {
    public:
        InternalCppServer(WMControl* Parent);
        //virtual void deliverPlan(const PlanningTaskPtr& task, const Ice::Current&);
        virtual void deliverPlan(int id, const ActionSeq& plan, const Ice::Current&);
        virtual void updateStatus(int id, Completion status, const Ice::Current&);
        virtual void setChangeFilter(int id, const StateChangeFilterPtr& filter, const Ice::Current&);

    protected:
        WMControl* parent;
    };

private:
    void writeAction(ActionPtr& action, PlanningTaskPtr& task);

    std::string m_python_server;

};

#endif

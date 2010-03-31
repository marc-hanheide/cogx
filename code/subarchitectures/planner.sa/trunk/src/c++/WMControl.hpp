#ifndef WM_CONTROL_HPP_
#define WM_CONTROL_HPP_

#include <tr1/unordered_map>
#include <cast/architecture.hpp>
#include <Ice/Ice.h>
#include "Planner.hpp"
#include <map>
#include <sys/time.h>
#include <boost/thread.hpp>

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
    void generateState(PlanningTaskPtr& task);
    //void deliverPlan(const autogen::Planner::PlanningTaskPtr& task);
    void deliverPlan(int id, const ActionSeq& plan);
    void updateStatus(int id, Completion status);
    void setChangeFilter(int id, const StateChangeFilterPtr& filter);

    void receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc);
    void actionChanged(const cast::cdl::WorkingMemoryChange& wmc);
    void stateChanged(const cast::cdl::WorkingMemoryChange& wmc);

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
    void sendStateChange(int id, std::vector<binder::autogen::beliefs::BeliefPtr>& changedBeliefs, const cast::cdl::CASTTime & newTimeStamp, StateChangeFilterPtr* filter);
    void writeAction(ActionPtr& action, PlanningTaskPtr& task);
    void dispatchPlanning(PlanningTaskPtr& task, int msecs=0);

    typedef std::tr1::unordered_map<std::string, binder::autogen::beliefs::BeliefPtr> BeliefMap;
    BeliefMap m_currentState;
    std::map<int, StateChangeFilterPtr> m_stateFilters;
    cast::cdl::CASTTime m_lastUpdate;

    std::map<int, timeval> m_runqueue;
    boost::mutex m_queue_mutex;

    std::string m_python_server;
    bool m_continual_state_updates;

};

#endif

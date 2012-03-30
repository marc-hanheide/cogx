#ifndef WM_CONTROL_HPP_
#define WM_CONTROL_HPP_

#include <tr1/unordered_map>
#include <cast/architecture.hpp>
#include <Ice/Ice.h>
#include "Planner.hpp"
#include <map>
#include <set>
#include <sys/time.h>
#include <boost/thread.hpp>

using namespace autogen::Planner;

class WMControl : public cast::ManagedComponent {
public:
    virtual ~WMControl() {}

protected:
    PythonServerPrx pyServer;

    virtual void configure(const cast::cdl::StringMap& _config, const Ice::Current& _current);
    virtual void start();
    virtual void runComponent();

    void connectToPythonServer();
    //vector<beliefmodels::autogen::beliefs::BeliefPtr>* generateState();
    //void deliverPlan(const autogen::Planner::PlanningTaskPtr& task);
    void deliverPlan(int id, const ActionSeq& plan, const GoalSeq& goals);
    void deliverPOPlan(int id, const POPlanPtr& po_plan=0);
    void deliverHypotheses(int id, const BeliefSeq& hypotheses);
    void updateBeliefState(const BeliefEntrySeq& beliefs);
    void updateStatus(int id, Completion status);
    void waitForChanges(int id, int timeout);
    bool queryGoal(const std::string& goal);

    void receivePlannerCommands(const cast::cdl::WorkingMemoryChange& wmc);
    void actionChanged(const cast::cdl::WorkingMemoryChange& wmc);
    void stateChanged(const cast::cdl::WorkingMemoryChange& wmc);
    void newPercept(const cast::cdl::WorkingMemoryChange& wmc);
    void taskChanged(const cast::cdl::WorkingMemoryChange& wmc);
    void taskRemoved(const cast::cdl::WorkingMemoryChange& wmc);
    void verbalise(const std::string& phrase);

    class InternalCppServer : public CppServer {
    public:
        InternalCppServer(WMControl* Parent);
        //virtual void deliverPlan(const PlanningTaskPtr& task, const Ice::Current&);
        virtual void deliverPlan(int id, const ActionSeq& plan, const GoalSeq& goals, const Ice::Current&);
        virtual void deliverPOPlan(int id, const POPlanPtr& orderedPlan, const Ice::Current&);
        virtual void deliverHypotheses(int id, const BeliefSeq& hypotheses, const Ice::Current&);
        virtual void updateBeliefState(const BeliefEntrySeq& beliefs, const Ice::Current&);
        virtual void updateStatus(int id, Completion status, const Ice::Current&);
        virtual void setChangeFilter(int id, const StateChangeFilterPtr& filter, const Ice::Current&);
        virtual void waitForChanges(int id, int timeout, const Ice::Current&);
        virtual bool queryGoal(const std::string& goal, const Ice::Current&);
        virtual void verbalise(const std::string& phrase, const Ice::Current&);
        virtual cast::cdl::WorkingMemoryAddress newAddress(const Ice::Current&);

    protected:
        WMControl* parent;
    };

private:
    void writeAction(ActionPtr& action, PlanningTaskPtr& task);
    void dispatchPlanning(PlanningTaskPtr& task, int msecs=0);

    inline bool later_than(const timeval& t1, const timeval& t2);

    typedef std::tr1::unordered_map<int,cast::cdl::WorkingMemoryAddress> taskMap;
    taskMap activeTasks;

    typedef std::tr1::unordered_map<int, std::string> POPlanMap;
    POPlanMap m_running_poplans;
    POPlanMap m_completed_poplans;

    typedef std::tr1::unordered_map< std::string, BeliefEntry > BeliefMap;
    typedef std::vector< BeliefEntry > PerceptList;
    BeliefMap m_currentState;
    PerceptList m_percepts;
    cast::cdl::CASTTime m_lastUpdate;
    bool m_new_updates;
    timeval m_belief_activity_timeout;

    std::map<int, timeval> m_runqueue;
    std::map<int, timeval> m_waiting_tasks;
    boost::mutex m_queue_mutex;

    std::string m_python_server;
    bool m_continual_state_updates;
    int m_active_task_id;

};

bool WMControl::later_than(const timeval& t1, const timeval& t2) {
    return ((t2.tv_sec < t1.tv_sec) 
            || ((t2.tv_sec == t1.tv_sec) && (t2.tv_usec < t1.tv_usec)));
}

#endif

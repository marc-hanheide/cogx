#ifndef DTP_CONTROL
#define DTP_CONTROL


/*Comment this out if you want the old behaviour. */
#define EXPOSING_DTP 1

#include <tr1/memory>
//#include <cast/architecture.hpp>
#include <Ice/Ice.h>

#include <cast/core/CASTComponent.hpp>

#include "../Planner.hpp"

#include <map>
#include <memory>

#include "planning_symbols.hh"

#include "solver.hh"
#include "two_phase_solver.hh"
#include "policy_iteration_over_information_state_space.hh"
#include "policy_iteration_over_information_state_space__GMRES.hh"

using namespace autogen::Planner;

#include <pthread.h>


class DTPCONTROL : 
    public autogen::Planner::DTPServer,
    public cast::CASTComponent 
{
public:
    enum Turn {observer, actor};
    
    DTPCONTROL();
    ~DTPCONTROL();

    //~DTPCONTROL();
    void improvePlanQuality(Ice::Int, const Ice::Current&);
    void deliverObservation(Ice::Int, const autogen::Planner::ObservationSeq& observationSeq, const Ice::Current&);
    void newTask(Ice::Int, const std::string& probleFile, const std::string& domainFile, const Ice::Current&);
    
    /* Joins the thread associated with DTP planning task indexed by
     * \argument{id}. (see \member{cancelTask} and \member{stop})*/
    void cancelTask(Ice::Int, const Ice::Current&);

    /* Running in a thread. Is woken up every time an observation comes
     * in on the relevant task.*/
    void post_action(Ice::Int id);

    
    void get_turn(Ice::Int id, Turn turn);
    void swap_turn(Ice::Int id, Turn turn);
    
    /*(see \method{deliverObservation})*/
    void get_observation(Ice::Int id,
                    const autogen::Planner::ObservationSeq& observationSeq);
protected:

    
    void configure(const cast::cdl::StringMap& _config, const Ice::Current& _current);
    
    void start();
    void stop();

    /*Does nothing.*/
    void runComponent();

    /* Can a call to ~DTPCONTROL be made?*/
    bool cannot_be_killed() const;
private:
    /* Freiburg python planning server.*/
    PythonServerPrx pyServer;
    
//     /* Joins the thread associated with DTP planning task indexed by
//      * \argument{id}. (see \member{cancelTask} and \member{stop})*/
//     void _cancelTask(Ice::Int);
    
    /* Called by \member{newTask}. Creates a thread that executes
     * \member{post_action(id)}. \members{thread_statuus,
     * thread_mutex, threads, thread_attributes} are configured during
     * this call.*/ 
    void spawn__post_action__thread(Ice::Int id);
    void spawn__get_observation__thread(Ice::Int id,
                                        const autogen::Planner::ObservationSeq& osequence);
    
    /* Name of the CAST planning component (see \module{WMControl} and
     * \method{configure})*/
    std::string m_python_server;
    
    typedef std::tr1::shared_ptr<pthread_t> Thread;
    typedef std::tr1::shared_ptr<pthread_attr_t> Thread_Attributes;
    typedef std::tr1::shared_ptr<pthread_mutex_t> Mutex;

    /* Domain associated with task \index{Ice::Int}*/
    std::map<Ice::Int, Planning::Domain_Name> thread_to_domain;
    
    /* Problem associated with task \index{Ice::Int}*/
    std::map<Ice::Int, Planning::Problem_Name> thread_to_problem;

    /* Thread status (false == being killed, true == alive) associated
     * with task \index{Ice::Int}*/
    std::map<Ice::Int, bool> thread_statuus;
    
//     /* Mutex (action executes, then observation is received, then
//      * action executes, etc...) associated with task
//      * \index{Ice::Int}*/
//     std::map<Ice::Int, Mutex> thread_mutex;
    
    /* Should be be observing or acting on thread \argument{Ice::Int}.*/
    std::map<Ice::Int, Turn> whose_turn_is_it;
//     std::map<Ice::Int, Mutex> whose_turn_is_it__mutex;
        
    
    /* Can this class be destructed.*/
    mutable Mutex destructor_mutex;
    
    /* Thread associated with task \index{Ice::Int}*/
    std::map<Ice::Int, Thread> threads;
    
    /* Thread attributes (i.e., joinable) associated with task
     * \index{Ice::Int}*/
    std::map<Ice::Int, Thread_Attributes> thread_attributes;

#ifdef EXPOSING_DTP
    /*Solver on each thread.*/
#ifdef LAO_STAR
    std::map<Ice::Int, Planning::Two_Phase_Solver*> solvers;
#else
    std::map<Ice::Int, Planning::Solver*> solvers;
#endif

    /*Current information-state on each thread.*/
    std::map<Ice::Int, Planning::POMDP_State*> current_state;
    
    /*Action last executed on each thread.*/
    std::map<Ice::Int, uint> action_index;
#endif

};


#endif

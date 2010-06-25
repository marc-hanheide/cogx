#ifndef DTP_CONTROL
#define DTP_CONTROL


#include <tr1/memory>
//#include <cast/architecture.hpp>
#include <Ice/Ice.h>

#include <cast/core/CASTComponent.hpp>

#include "../Planner.hpp"

#include <map>
#include <memory>

#include "planning_symbols.hh"

using namespace autogen::Planner;

#include <pthread.h>


class DTPCONTROL : 
    public autogen::Planner::DTPServer,
    public cast::CASTComponent 
{
public:
    DTPCONTROL();
    ~DTPCONTROL();

    //~DTPCONTROL();
    void deliverObservation(Ice::Int, const autogen::Planner::ObservationSeq& observationSeq, const Ice::Current&);
    void newTask(Ice::Int, const std::string& probleFile, const std::string& domainFile, const Ice::Current&);  
    void cancelTask(Ice::Int, const Ice::Current&);

    /* Running in a thread. Is woken up every time an observation comes
     * in on the relevant task.*/
    void post_action(Ice::Int id);
protected:

    
    void configure(const cast::cdl::StringMap& _config, const Ice::Current& _current);
    
    void start();
    void stop();

    /*Does nothing.*/
    void runComponent();

    
  
private:
    /* Freiburg python planning server.*/
    PythonServerPrx pyServer;
    
    /* Joins the thread associated with DTP planning task indexed by
     * \argument{id}. (see \member{cancelTask} and \member{stop})*/
    void _cancelTask(Ice::Int);
    
    /* Called by \member{newTask}. Creates a thread that executes
     * \member{post_action(id)}. \members{thread_statuus,
     * thread_mutex, threads, thread_attributes} are configured during
     * this call.*/ 
    void spawn__post_action__thread(Ice::Int id);

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
    
    /* Mutex (action executes, then observation is received, then
     * action executes, etc...) associated with task
     * \index{Ice::Int}*/
    std::map<Ice::Int, Mutex> thread_mutex;
    
    /* When shutting things down, we have to ensure that an action is
     * not executing.*/
    std::map<Ice::Int, Mutex> thread_running_mutex;
    
    /* Thread associated with task \index{Ice::Int}*/
    std::map<Ice::Int, Thread> threads;
    
    /* Thread attributes (i.e., joinable) associated with task
     * \index{Ice::Int}*/
    std::map<Ice::Int, Thread_Attributes> thread_attributes;

};


#endif

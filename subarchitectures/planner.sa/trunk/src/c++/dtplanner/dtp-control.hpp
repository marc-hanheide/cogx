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
    ~DTPCONTROL(){};

    //~DTPCONTROL();
    void deliverObservation(Ice::Int, const autogen::Planner::ObservationSeq& observationSeq, const Ice::Current&);
    void newTask(Ice::Int, const std::string& probleFile, const std::string& domainFile, const Ice::Current&);  
    void cancelTask(Ice::Int, const Ice::Current&);

    /* Running in a thread. Is woken up every time an observation comes
     * in on the relevant task.*/
    void post_action(Ice::Int id);
    
protected:
    void configure(const cast::cdl::StringMap& _config, const Ice::Current& _current){}
    void start(){}
    void runComponent(){}
    void stop(){};
  
private:
    typedef std::tr1::shared_ptr<pthread_mutex_t> Mutex;
    
    std::map<Ice::Int, Planning::Domain_Name> thread_to_domain;
    std::map<Ice::Int, Planning::Problem_Name> thread_to_problem;

    std::map<Ice::Int, bool> thread_statuus;
    std::map<Ice::Int, Mutex> thread_mutex;
    std::map<Ice::Int, pthread_t> threads;
    std::map<Ice::Int, pthread_attr_t> thread_attributes;

};


#endif

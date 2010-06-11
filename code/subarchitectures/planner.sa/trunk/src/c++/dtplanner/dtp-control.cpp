
#include "dtp-control.hpp"

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new DTPCONTROL();
    }
}

#include "dtp_pddl_parsing_interface.hh"
#include "dtp_pddl_parsing_data.hh"
#include "dtp_pddl_parsing_data_domain.hh"
#include "dtp_pddl_parsing_data_problem.hh"

#include <sstream>

#define A_CAST_SPECIAL__MUTEX_INITIALISATION(X) pthread_mutex_t* X;     \
    {                                                                   \
        pthread_mutex_t a_rose_by_any_other_name                        \
            = PMUTEX_MUTEX_INITIALIZER;                                 \
        X = new pthread_mutex_t;                                        \
        *X = a_rose_by_any_other_name;                                  \
    }                                                                   \


namespace CAST_THREADS {
    
    pthread_mutex_t* give_me_a_new__pthread_mutex_t()
    {
        pthread_mutex_t* X;
        X = new pthread_mutex_t; 
#ifdef PMUTEX_MUTEX_INITIALIZER
        pthread_mutex_t a_rose_by_any_other_name  
            = PMUTEX_MUTEX_INITIALIZER;                   
        *X = a_rose_by_any_other_name;     
#else
        pthread_mutex_init(X, 0);
#endif
        
        return X;
    }
}

DTPCONTROL::DTPCONTROL()
{
}


void DTPCONTROL::deliverObservation(Ice::Int id,
                                    const autogen::Planner::ObservationSeq& observationSeq,
                                    const Ice::Current&){
    if(thread_to_domain.find(id) == thread_to_domain.end()) return;

    std::vector<std::string> observations;
    
    for(auto obs = observationSeq.begin()
            ; obs != observationSeq.end()
            ; obs++){
        std::ostringstream oss;
        oss<<(*obs)->predicate<<"("<<(*obs)->arguments<<")";
        observations.push_back(oss.str());
    }
    
    
    Planning::Parsing::Problem_Identifier pi(thread_to_domain[id], thread_to_problem[id]);
    Planning::Parsing::problems[pi]->report__observations(observations);
    
    pthread_mutex_unlock(thread_mutex[id].get());
}

void  DTPCONTROL::post_action(Ice::Int id)
{
    while(thread_statuus[id]){
        pthread_mutex_lock(thread_mutex[id].get());
        
        if(thread_to_domain.find(id) == thread_to_domain.end()) return;
        
        
        Planning::Parsing::Problem_Identifier pi(thread_to_domain[id], thread_to_problem[id]);
        auto action = Planning::Parsing::problems[pi]->get__prescribed_action();
        std::ostringstream oss;
        oss<<action;
        auto action_as_string = oss.str();
        
        /*Moritz, how do I go about this???*/
        autogen::Planner::PDDLAction* pddlaction = new autogen::Planner::PDDLAction();
        pddlaction->name = action_as_string;
        /*Moritz, how do I go about this???*/
        //SOMETHINGORRATHERNOTHING.deliverAction(id, pddlaction)
    }
}

void DTPCONTROL::newTask(Ice::Int id,
                         const std::string& probleFile,
                         const std::string& domainFile, const Ice::Current&)
{
    Planning::Parsing::parse_domain(domainFile);
    Planning::Parsing::parse_problem(probleFile);

    thread_to_domain[id] = Planning::Parsing::domain_Stack->get__domain_Name();
    thread_to_problem[id] = Planning::Parsing::problem_Stack->get__problem_Name();
    
    /*Planning is complete, now start \member{post_action} in a thread.*/
    
    
    
}

void DTPCONTROL::cancelTask(Ice::Int id, const Ice::Current&)
{
    
    thread_to_domain.erase(id);
    thread_to_domain.erase(id);
}

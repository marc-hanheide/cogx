
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



void* DTPCONTROL__pthread_METHOD_post_action__CALLBACK(void* _argument)
{
    VERBOSER(1001, "DTP starting thread for posting actions.");
    
    typedef std::tr1::tuple<int, DTPCONTROL*> Argument;
    auto argument = reinterpret_cast< Argument*>(_argument);
    auto task_id = std::tr1::get<0>(*argument);
    auto dtpcontrol = std::tr1::get<1>(*argument);

    VERBOSER(1001, "DTP making post_action call for :: "<<task_id);
    
    dtpcontrol->post_action(task_id);

    delete argument;
    
    pthread_exit(_argument);
}

void* DTPCONTROL__pthread_METHOD_get_observation__CALLBACK(void* _argument)
{
    VERBOSER(1001, "DTP starting thread for posting actions.");
    
    typedef std::tr1::tuple<int,  DTPCONTROL*, autogen::Planner::ObservationSeq> Argument;
    auto argument = reinterpret_cast< Argument*>(_argument);
    auto task_id = std::tr1::get<0>(*argument);
    auto dtpcontrol = std::tr1::get<1>(*argument);
    auto osequence = std::tr1::get<2>(*argument);

    VERBOSER(1001, "DTP making post_action call for :: "<<task_id);
    
    dtpcontrol->get_observation(task_id, osequence);

    delete argument;
    
    pthread_exit(_argument);
}

void DTPCONTROL::spawn__post_action__thread(Ice::Int id)
{
    
    auto thread = Thread(new pthread_t());
    auto attributes = Thread_Attributes(new pthread_attr_t());
    
    
    /*The thread at $id$ is wanted.*/
    thread_statuus[id] = true;
//     Mutex mutex(CAST_THREADS::give_me_a_new__pthread_mutex_t());
//     thread_mutex[id] = mutex;
    
//     Mutex _mutex(CAST_THREADS::give_me_a_new__pthread_mutex_t());
//     whose_turn_is_it__mutex[id] = _mutex;
    whose_turn_is_it[id] = Turn::actor;
    
    
    threads[id] = thread;
    thread_attributes[id] = attributes;
    
    QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_init(attributes.get()),
                              "Failed to initialise thread attributes for DTP planning task :: "
                              <<id);

    
    QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setdetachstate(attributes.get(), PTHREAD_CREATE_DETACHED),//PTHREAD_CREATE_JOINABLE),
                              "Failed to make detached thread attributes for DTP planning task :: "
//                               "Failed to make joinable thread attributes for DTP planning task :: "
                              <<id);
    
    QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setscope(attributes.get(), PTHREAD_SCOPE_SYSTEM),
                              "Failed to make thread attributes with SYSTEM scope for DTP planning task :: "
                              <<id);

    
    QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setinheritsched(attributes.get(), PTHREAD_INHERIT_SCHED),
                              "Failed to make thread attributes with the execution scheduling"<<std::endl
                              <<"of its parent for task :: "
                              <<id);
    
    
    typedef std::tr1::tuple<int, DTPCONTROL*> Argument;
    Argument* argument = new Argument(id, this);
    
    /*Attempt to spawn a new thread.*/
    auto result = 0;
    auto number_of_attempted_thread_invocations = 0;
    while(0 != (result = pthread_create(thread.get(),
                                        attributes.get(),
                                        DTPCONTROL__pthread_METHOD_post_action__CALLBACK,
                                        argument))){
        
#ifdef _POSIX_THREAD_THREADS_MAX
        WARNING("Failed to spawn a thread. "
                <<"Perhaps the upper limit on the number we can spawn is :: "
                <<_POSIX_THREAD_THREADS_MAX<<".");
#endif

                
        switch(result){
            case EAGAIN:
                WARNING("Your system lacked the necessary resources"
                        <<" to create another thread, OR your system-imposed"
                        <<" limit  on  the  total  number  of threads in"
                        <<" a process {PTHREAD_THREADS_MAX} would be exceeded. Nice grammar!!"
                        <<" Anyway, I will try again after waiting for threads"
                        <<" I am perhaps responsible for...");
                usleep(100 * number_of_attempted_thread_invocations);
                break;
            case EINVAL:
                UNRECOVERABLE_ERROR("My bad. I specified thread attribute values that are invalid."
                                    <<" Nothing to be done.. so I am killing myself "
                                    <<"-- i.e., \"pthread_exit(0)\".");

                pthread_exit(0);
                break;
            case EPERM:
                UNRECOVERABLE_ERROR("The caller does not have appropriate permission to"
                                    <<" set the required scheduling parameters or"
                                    <<" scheduling policy. Whatever that means ;)"
                                    <<" Nothing to be done, I am killing myself -- "
                                    <<"-- i.e., \"pthread_exit(0)\".");
                        
                pthread_exit(0);
                break;
                /*#include<bits/local_lim.h>*/

            case ENOMEM:
                UNRECOVERABLE_ERROR("Oh great! pthread_create() just gave me a ENOMEM. "
                                    <<"This undocumented behaviour occurs under Redhat Linux 2.4 "
                                    <<"when too many threads have been created in the non-detached "
                                    <<"mode, and the limited available memory in some system stack "
                                    <<"is consumed. At that point no new threads can be created "
                                    <<"in non-detached mode until those threads are "
                                    <<"detached/killed, or the parent process(es) killed and restarted. "
                                    <<"My behaviour here is to crash, better luck next time chief!");
                        
                pthread_exit(0);
                break;
            default:
                WARNING("No idea why we couldn't spawn a thread. The error code we were given is :: "
                        <<result<<" Trying again...");
                usleep((10 * number_of_attempted_thread_invocations));
                break;
        }
           
        number_of_attempted_thread_invocations++;     
                
        QUERY_UNRECOVERABLE_ERROR(number_of_attempted_thread_invocations > 4,/*FIX : Magic number*/
                                  "Can't start a thread for task :: "<<id
                                  <<" Already tried :: "
                                  <<number_of_attempted_thread_invocations
                                  <<" times... and still nothing!");
        
    }
}

void DTPCONTROL::spawn__get_observation__thread(Ice::Int id,  const autogen::Planner::ObservationSeq& osequence)
{
    
    auto thread = Thread(new pthread_t());
    auto attributes = Thread_Attributes(new pthread_attr_t());
    
    QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_init(attributes.get()),
                              "Failed to initialise thread attributes for DTP planning task :: "
                              <<id);

    
    QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setdetachstate(attributes.get(), PTHREAD_CREATE_DETACHED),//PTHREAD_CREATE_JOINABLE),
                              "Failed to make detached thread attributes for DTP planning task :: "
//                               "Failed to make joinable thread attributes for DTP planning task :: "
                              <<id);
    
    QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setscope(attributes.get(), PTHREAD_SCOPE_SYSTEM),
                              "Failed to make thread attributes with SYSTEM scope for DTP planning task :: "
                              <<id);

    
    QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setinheritsched(attributes.get(), PTHREAD_INHERIT_SCHED),
                              "Failed to make thread attributes with the execution scheduling"<<std::endl
                              <<"of its parent for task :: "
                              <<id);
    
    
    typedef std::tr1::tuple<Ice::Int, DTPCONTROL*, autogen::Planner::ObservationSeq> Argument;
    Argument* argument = new Argument(id, this, osequence);
    
    /*Attempt to spawn a new thread.*/
    auto result = 0;
    auto number_of_attempted_thread_invocations = 0;
    while(0 != (result = pthread_create(thread.get(),
                                        attributes.get(),
                                        DTPCONTROL__pthread_METHOD_get_observation__CALLBACK,
                                        argument))){
        
#ifdef _POSIX_THREAD_THREADS_MAX
        WARNING("Failed to spawn a thread. "
                <<"Perhaps the upper limit on the number we can spawn is :: "
                <<_POSIX_THREAD_THREADS_MAX<<".");
#endif

                
        switch(result){
            case EAGAIN:
                WARNING("Your system lacked the necessary resources"
                        <<" to create another thread, OR your system-imposed"
                        <<" limit  on  the  total  number  of threads in"
                        <<" a process {PTHREAD_THREADS_MAX} would be exceeded. Nice grammar!!"
                        <<" Anyway, I will try again after waiting for threads"
                        <<" I am perhaps responsible for...");
                usleep(100 * number_of_attempted_thread_invocations);
                break;
            case EINVAL:
                UNRECOVERABLE_ERROR("My bad. I specified thread attribute values that are invalid."
                                    <<" Nothing to be done.. so I am killing myself "
                                    <<"-- i.e., \"pthread_exit(0)\".");

                pthread_exit(0);
                break;
            case EPERM:
                UNRECOVERABLE_ERROR("The caller does not have appropriate permission to"
                                    <<" set the required scheduling parameters or"
                                    <<" scheduling policy. Whatever that means ;)"
                                    <<" Nothing to be done, I am killing myself -- "
                                    <<"-- i.e., \"pthread_exit(0)\".");
                        
                pthread_exit(0);
                break;
                /*#include<bits/local_lim.h>*/

            case ENOMEM:
                UNRECOVERABLE_ERROR("Oh great! pthread_create() just gave me a ENOMEM. "
                                    <<"This undocumented behaviour occurs under Redhat Linux 2.4 "
                                    <<"when too many threads have been created in the non-detached "
                                    <<"mode, and the limited available memory in some system stack "
                                    <<"is consumed. At that point no new threads can be created "
                                    <<"in non-detached mode until those threads are "
                                    <<"detached/killed, or the parent process(es) killed and restarted. "
                                    <<"My behaviour here is to crash, better luck next time chief!");
                        
                pthread_exit(0);
                break;
            default:
                WARNING("No idea why we couldn't spawn a thread. The error code we were given is :: "
                        <<result<<" Trying again...");
                usleep((10 * number_of_attempted_thread_invocations));
                break;
        }
           
        number_of_attempted_thread_invocations++;     
                
        QUERY_UNRECOVERABLE_ERROR(number_of_attempted_thread_invocations > 4,/*FIX : Magic number*/
                                  "Can't start an observation thread for task :: "<<id
                                  <<" Already tried :: "
                                  <<number_of_attempted_thread_invocations
                                  <<" times... and still nothing!");   
    }
}

#define METHOD_PREFIX                           \
    if(!isRunning()){return;}                   \
    VERBOSER(1000, "DTP -- METHOD_PREFIX -- isRunning");       \
    pthread_mutex_lock(destructor_mutex.get()); \
    if(!isRunning()){return;}                   \
    VERBOSER(1000, "DTP -- METHOD_PREFIX -- destructor");       \
    lockComponent();                            \
    VERBOSER(1000, "DTP -- METHOD_PREFIX -- CAST");       \
    

#define METHOD_RETURN                                       \
    {                                                       \
        pthread_mutex_unlock(destructor_mutex.get());       \
        VERBOSER(1000, "DTP -- METHOD_RETURN -- destructor");      \
        unlockComponent();                                  \
        VERBOSER(1000, "DTP -- METHOD_RETURN -- CAST");      \
        return ;                                            \
    }                                                       \
        
#define LOOP_CLOSE                                          \
    {                                                       \
        pthread_mutex_unlock(destructor_mutex.get());       \
        VERBOSER(1000, "DTP -- LOOP_CLOSE -- destructor");      \
        unlockComponent();                                  \
        VERBOSER(1000, "DTP -- LOOP_CLOSE -- CAST");        \
    }                                                       \
        
#define LOOP_OPEN                                       \
    {                                                   \
        pthread_mutex_lock(destructor_mutex.get());     \
        VERBOSER(1000, "DTP -- LOOP_OPEN -- destructor");      \
        lockComponent();                                \
        VERBOSER(1000, "DTP -- LOOP_OPEN -- CAST");      \
    }                                                   \
        



DTPCONTROL::DTPCONTROL()
 :destructor_mutex(CAST_THREADS::give_me_a_new__pthread_mutex_t())
{
}

DTPCONTROL::~DTPCONTROL()
{
    while(cannot_be_killed()){
        std::ostringstream oss;

        LOOP_OPEN;
        for(auto thread = threads.begin()
                ; thread != threads.end()
                ; thread++){
            oss<<thread->first<<" ";
        }
        LOOP_CLOSE;

        std::string tmp = oss.str();
        
        WARNING("Unable to destroy DTP component. Still has active tasls."<<std::endl
                <<"Those tasks are :: "<<tmp<<std::endl
                <<"Please kill those by some calls to cancelTask(TASK_ID)"<<std::endl);
        usleep(100);
    }
    
    pthread_mutex_destroy(destructor_mutex.get());
}

bool DTPCONTROL::cannot_be_killed() const
{
    bool answer = false;
    pthread_mutex_lock(destructor_mutex.get());
    answer = threads.size();
    pthread_mutex_unlock(destructor_mutex.get());
    return answer;
}

void DTPCONTROL::get_observation(Ice::Int id,
                                 const autogen::Planner::ObservationSeq& observationSeq)
{

    VERBOSER(1001, "DTP got an observation posted for task :: "<<id);
    log("OBSERVER :: DTP got an observation posted for task");
    get_turn(id, Turn::observer);
    log("OBSERVER :: HAS CONTROL");
    
    METHOD_PREFIX;
    VERBOSER(2000, "DTP dealing with an observation for task "<<id);
    log("OBSERVER:: COMPLETED METHOD PREFIX");


    
    /* Supposed to be ignoring signals on to $id$.*/
    if(!thread_statuus[id]){
        log("OBSERVER:: TASK IS DEAD RETURNING");
        VERBOSER(1001, "DTP was requested to ignore observations on task :: "<<id);
        METHOD_RETURN;
    }
    
    /* Task is being killed.*/
    if(observationSeq.size() == 0){
        log("OBSERVER:: KILLING");
        thread_statuus[id] = false;

        
        //delete solvers[id];
        solvers[id]->cleanup();
        
        VERBOSER(2000, "DTP observation was to kill task "<<id);
//         pthread_mutex_unlock(thread_mutex[id].get());
        swap_turn(id, Turn::observer);
        METHOD_RETURN;
    }

    if(thread_to_domain.find(id) == thread_to_domain.end()){
        WARNING("Posting observation to DTP for task :: "<<id<<std::endl
                <<", however we don't have a domain for that task."<<std::endl
                <<"Will sleep a while and see if a domain appears."<<std::endl);
        for(auto i = 1; i < 4; i++){
            usleep(10);
            if(thread_to_domain.find(id) != thread_to_domain.end()){
                WARNING("Sleeping worked, got a domain for DTP task :: "<<id<<std::endl);
                break;
            } 
            WARNING("Sleeping in order to get a DTP for task :: "<<id<<std::endl
                    <<"Failed for :: "<<i<<" sleeps");
        }
    }
    
    QUERY_UNRECOVERABLE_ERROR
        (thread_to_domain.find(id) == thread_to_domain.end()
         , "Could not find domain for task :: "<<id<<std::endl);

    VERBOSER(1001, "DTP has a domain for task :: "<<id<<std::endl
             <<"And is therefore at a point to take the observation seriously.");
    
#ifdef EXPOSING_DTP

    Planning::Solver::Percept_List percepts;
    for(auto obs = observationSeq.begin()
            ; obs != observationSeq.end()
            ; obs++){
        Planning::Solver::Precept percept;
        percept.first = (*obs)->predicate;
        percept.second = (*obs)->arguments;

        percepts.push_back(percept);
    }
    
    assert(current_state.find(id) != current_state.end());
    assert(solvers.find(id) != solvers.end());
    assert(action_index.find(id) != action_index.end());
    
    Planning::POMDP_State* successor_state
        = solvers[id]->take_observation(current_state[id],
                                        percepts,
                                        action_index[id]);

    
    auto& available_observations = current_state[id]
        ->get__possible_observations_given_action(action_index[id]);
    
    current_state[id] = successor_state;
    
    if(1 < available_observations.size()){
        current_state[id] = solvers[id]->solve__for_new_starting_state(successor_state);
    }
    
    
#else
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

#endif
    
    VERBOSER(1001, "DTP observation now triggering action on task  :: "<<id);
    log("OBSERVER:: OBSERVATION TRIGGERING ACTION.");
    swap_turn(id, Turn::observer);
//     pthread_mutex_unlock(thread_mutex[id].get());

    
    log("OBSERVER:: METHOD COMPLETION");
    VERBOSER(2000, "DTP observation has been delt with, time to post an action for task "<<id);
    METHOD_RETURN; 
}

void DTPCONTROL::deliverObservation(Ice::Int id,
                                    const autogen::Planner::ObservationSeq& _observationSeq,
                                    const Ice::Current&){
    
    VERBOSER(1001, "DTP THREAD SPAWNING got an observation posted for task :: "<<id);

    autogen::Planner::ObservationSeq observationSeq = _observationSeq;
    spawn__get_observation__thread(id, observationSeq);
}

void DTPCONTROL::swap_turn(Ice::Int id, Turn turn)
{
    assert(whose_turn_is_it.find(id) != whose_turn_is_it.end());
//     pthread_mutex_lock(whose_turn_is_it__mutex[id].get());
    whose_turn_is_it[id] = ((turn==Turn::actor)?(Turn::observer):(Turn::actor));
//     pthread_mutex_unlock(whose_turn_is_it__mutex[id].get());
}

void DTPCONTROL::get_turn(Ice::Int id, Turn turn)
{
    
    bool my_turn = false;
    while(!my_turn){
        LOOP_OPEN;
        
        if(whose_turn_is_it.find(id) == whose_turn_is_it.end()){
            LOOP_CLOSE;
            break;
        }
        
        
//         pthread_mutex_lock(whose_turn_is_it__mutex[id].get());
        my_turn = (whose_turn_is_it[id] == ((turn==Turn::actor)?(Turn::actor):(Turn::observer)));
//         pthread_mutex_unlock(whose_turn_is_it__mutex[id].get());
        LOOP_CLOSE;
        
        usleep(10);
    }
}

void  DTPCONTROL::post_action(Ice::Int id)
{
    VERBOSER(2000, "DTP START-UP posting actions for task "<<id);
    
    while(true){//thread_statuus[id]){
        VERBOSER(1001, "DTP action posting trying for mutex :: "<<id);

        log("ACTOR:: GETTING TURN");        
        get_turn(id, Turn::actor);
//         pthread_mutex_lock(thread_mutex[id].get());
        
        LOOP_OPEN;
        
        log("ACTOR:: IN MAIN LOOP");
        VERBOSER(2000, "DTP posting  an  action for task "<<id);
        
        if(!thread_statuus[id]){
            
            log("ACTOR:: KILLED"); 
            VERBOSER(2000, "DTP action posting withdrawn for task "<<id<<std::endl
                     <<"That was cancelled.");
            break;
        }

        QUERY_UNRECOVERABLE_ERROR
            (thread_to_domain.find(id) == thread_to_domain.end()
             , "DTP Could not find domain for task :: "<<id<<std::endl);
        
        Planning::Parsing::Problem_Identifier pi(thread_to_domain[id], thread_to_problem[id]);
        QUERY_UNRECOVERABLE_ERROR
            (Planning::Parsing::problems.find(pi) == Planning::Parsing::problems.end()
             , "DTP Could not find problem for task :: "<<id<<std::endl);


#ifdef EXPOSING_DTP
        assert(current_state.find(id) != current_state.end());
        assert(solvers.find(id) != solvers.end());
        
        std::pair<Planning::Formula::Action_Proposition, uint> _action
            = solvers[id]->get_prescribed_action(current_state[id]);

        auto action = _action.first;
        action_index[id] = _action.second;
        
        assert(action_index.find(id) != action_index.end());
        
#else  
        auto action = Planning::Parsing::problems[pi]->get__prescribed_action();
#endif
        
        auto name = std::tr1::get<0>(action.contents());
        std::ostringstream oss;
        oss<<name;
        auto action_name = oss.str();
        
        auto _arguments = std::tr1::get<1>(action.contents());
        std::vector<std::string> action_arguments;

        VERBOSER(1001, "DTP posting an action :: "<<action_name<<std::endl
                 <<"Arguments are :: "<<_arguments);
        for(auto argument = _arguments.begin()
                ; argument != _arguments.end()
                ; argument++){
            std::ostringstream oss;
            oss<<*argument;
            auto _argument = oss.str();
            
            action_arguments.push_back(_argument);
            VERBOSER(1001, "Arg :: "<<_argument);
        }
        
        /*Moritz, how do I go about this???*/
        autogen::Planner::PDDLAction* _pddlaction = new autogen::Planner::PDDLAction();
        autogen::Planner::PDDLActionPtr  pddlaction(_pddlaction);
//         pddlaction->name = action_as_string;
        pddlaction->name = action_name;
        pddlaction->arguments = action_arguments;

        
        VERBOSER(1001, "DTP Posting the action :: "<<pddlaction->name);
        pyServer->deliverAction(id, pddlaction, current_state[id]->get__expected_value());
//         pyServer->deliverAction(id, pddlaction);
        VERBOSER(1001, "DTP Done posting the action :: "<<pddlaction->name);


        log("ACTOR:: SWAPPING TURN"); 
        swap_turn(id, Turn::actor);
        
        log("ACTOR:: CLOSING LOOP"); 
        VERBOSER(2000, "DTP done action post for task "<<id);
        LOOP_CLOSE;
    }

    VERBOSER(2000, "DTP killing action posting thread for task  :: "<<id);
    
    pthread_attr_destroy(thread_attributes[id].get());
    thread_attributes.erase(id);
    threads.erase(id);
    thread_to_domain.erase(id);
    thread_to_problem.erase(id);
    whose_turn_is_it.erase(id);
    
    METHOD_RETURN;
}

void DTPCONTROL::start()
{
    try	{
        log("Trying to connect to python server...");
        pyServer = getIceServer<autogen::Planner::PythonServer>(m_python_server);
    }
    catch (const Ice::Exception& ex) {
        std::cerr << ex << std::endl;
    }
    catch (const char* msg) {
        std::cerr << msg << std::endl;
    }
    log("Planner DTPCONTROL: running");
}

void DTPCONTROL::stop()
{
    if(cannot_be_killed()){
        WARNING("CRITICAL ERROR -- Attempt to stop DTP component before\n"
                <<"all the planning tasks have been cancelled.\n"
                <<"DTP component is now UNSTABLE.");
    }
}

void DTPCONTROL::runComponent(){
    VERBOSER(1001, "CAST runComponent called for DTP.");
}

void DTPCONTROL::configure(const cast::cdl::StringMap& _config, const Ice::Current& _current)
{
    log("Planner DTPCONTROL: connecting to Python Server");
    
    cast::CASTComponent::configure(_config, _current);

    cast::cdl::StringMap::const_iterator it = _config.begin();
    it = _config.find("--server");
    if (it != _config.end()) {
        m_python_server = it->second;
    }
    else {
        m_python_server = "PlannerPythonServer";
    }

    registerIceServer<autogen::Planner::DTPServer, autogen::Planner::DTPServer>(this);
}


void DTPCONTROL::improvePlanQuality(Ice::Int id, const Ice::Current&)
{
    METHOD_PREFIX;

    
//     Planning::Policy_Iteration policy_Iteration(solvers[id]->belief_state__space);
    Planning::Policy_Iteration__GMRES policy_Iteration(solvers[id]->belief_state__space
                                                       , solvers[id]->get__sink_state_penalty());
    for(uint i = 0; i < 10; i++){
        if(!solvers[id]->expand_belief_state_space()){
            break;
            VERBOSER(10017, "No starting state!"<<std::endl);
        } else {
            VERBOSER(10017, "Expanding!"<<std::endl);
//             policy_Iteration();
        }
    }
    
    for(uint i = 0; i < 5; i++){
        policy_Iteration();
        VERBOSER(10017, "Expected reward is :: "
                 <<current_state[id]->get__expected_value()<<std::endl);
    }
    
#ifdef EXPOSING_DTP
    assert(current_state.find(id) != current_state.end());
    assert(solvers.find(id) != solvers.end());
        
    std::pair<Planning::Formula::Action_Proposition, uint> _action
        = solvers[id]->get_prescribed_action(current_state[id]);

    auto action = _action.first;
    action_index[id] = _action.second;
        
    assert(action_index.find(id) != action_index.end());
        
#else  
    auto action = Planning::Parsing::problems[pi]->get__prescribed_action();
#endif
        
    auto name = std::tr1::get<0>(action.contents());
    std::ostringstream oss;
    oss<<name;
    auto action_name = oss.str();
        
    auto _arguments = std::tr1::get<1>(action.contents());
    std::vector<std::string> action_arguments;

    VERBOSER(1001, "DTP posting an action :: "<<action_name<<std::endl
             <<"Arguments are :: "<<_arguments);
    for(auto argument = _arguments.begin()
            ; argument != _arguments.end()
            ; argument++){
        std::ostringstream oss;
        oss<<*argument;
        auto _argument = oss.str();
            
        action_arguments.push_back(_argument);
        VERBOSER(1001, "Arg :: "<<_argument);
    }
        
    /*Moritz, how do I go about this???*/
    autogen::Planner::PDDLAction* _pddlaction = new autogen::Planner::PDDLAction();
    autogen::Planner::PDDLActionPtr  pddlaction(_pddlaction);
    //         pddlaction->name = action_as_string;
    pddlaction->name = action_name;
    pddlaction->arguments = action_arguments;

        
    VERBOSER(1001, "DTP Posting the action :: "<<pddlaction->name);
    pyServer->deliverAction(id, pddlaction, current_state[id]->get__expected_value());
    
    METHOD_RETURN;
}


void DTPCONTROL::newTask(Ice::Int id,
                         const std::string& problemFile,
                         const std::string& domainFile, const Ice::Current&)
{
    METHOD_PREFIX;
    
    VERBOSER(1001, "DTP for task ::"<<id
             <<" got problem at :: "<<problemFile
             <<" and domain at :: "<<domainFile);


    VERBOSER(1001, "DTP Parsing domain."<<std::endl);
    Planning::Parsing::parse_domain(domainFile);
    
    VERBOSER(1001, "DTP Parsing problem."<<std::endl);
    Planning::Parsing::parse_problem(problemFile);
    
    VERBOSER(2000, "DTP START-UP New task... \n"
             <<"Thread :: "<<id<<std::endl
             <<"Problem :: "<<Planning::Parsing::problem_Stack->get__problem_Name()<<std::endl
             <<"Domain :: "<<Planning::Parsing::problem_Stack->get__domain_Data()->get__domain_Name()<<std::endl);
//     thread_to_domain[id] = Planning::Parsing::domain_Stack->get__domain_Name();
    thread_to_domain[id] = Planning::Parsing::problem_Stack->get__domain_Data()->get__domain_Name();
    thread_to_problem[id] = Planning::Parsing::problem_Stack->get__problem_Name();


    QUERY_UNRECOVERABLE_ERROR
        (thread_to_domain.find(id) == thread_to_domain.end()
         , "DTP just finished parsing, and yet could not find domain for task :: "<<id<<std::endl);
        
    Planning::Parsing::Problem_Identifier pi(thread_to_domain[id], thread_to_problem[id]);
    QUERY_UNRECOVERABLE_ERROR
        (Planning::Parsing::problems.find(pi) == Planning::Parsing::problems.end()
         , "DTP Could not find problem for task :: "<<id<<std::endl);

#ifdef EXPOSING_DTP
    auto actual_problem = Planning::Parsing::problems.find(pi);
    VERBOSER(10050, "Problem is :: "<<*actual_problem->second<<std::endl);

#ifdef LAO_STAR
#define CHANGE_PHASE 1
            solvers[id] = new Planning::Two_Phase_Solver(*actual_problem->second);//Planning::Solver*
#else       
            solvers[id] = new Planning::Solver(*actual_problem->second);//Planning::Solver*
#endif
//     solvers[id] = new Planning::Solver(*actual_problem->second);//thread_to_problem[id]);
            
    solvers[id]->set__sink_state_penalty(-1.0);
    
    solvers[id]->preprocess();
    
    
#ifdef LAO_STAR
            if(true){/*START -- TRYING THE MDP HEURISTIC.*/            
                solvers[id]->empty__belief_states_for_expansion();
                solvers[id]->generate_markov_decision_process_starting_states();

                Planning::Policy_Iteration__GMRES policy_Iteration__for_MDP_states
                    (solvers[id]->belief_state__space, solvers[id]->get__sink_state_penalty(), .65);

                auto current_state = solvers[id]->peek__next_belief_state_for_expansion();
                
                INTERACTIVE_VERBOSER(true, 15000, "MDP state expansion :: "<<*current_state<<std::endl);
                

                while(solvers[id]->expand_belief_state_space()){
                    
                    VERBOSER(15000, "MDP state expansion :: "<<solvers[id]->belief_state__space.size()<<std::endl);
                }
                
                
                while(policy_Iteration__for_MDP_states()){
                    VERBOSER(15000, "Policy iteration."<<std::endl);
                }
                
                for(auto mdp_state = solvers[id]->belief_state__space.begin()
                        ; mdp_state != solvers[id]->belief_state__space.end()
                        ; mdp_state++){

                    if(*mdp_state == solvers[id]->get__starting_belief_state()){
                        continue;
                    }

                    double state_value = (*mdp_state)->get__expected_value();

                    auto& belief = (*mdp_state)->get__belief_state();

                    QUERY_UNRECOVERABLE_ERROR(belief.size() != 1,
                                              "Each belief state should contain one element, but we have "
                                              <<*mdp_state<<std::endl
                                              <<"That contains :: "<<belief.size()<<std::endl);
                    /*This is an MDP state, so there should only be one atom in the belief.*/
                    assert(1 == belief.size());

                    auto state = belief.begin();
                    assert(dynamic_cast<Planning::State*>(state->first));
                    state->first->set__value(state_value);
                }

                solvers[id]->reset__pomdp_state_hash_table();

#ifdef CHANGE_PHASE
                solvers[id]->change_phase();//HERE -- MONDAY
#endif
                
                solvers[id]->reinstate__starting_belief_state();
            }/*STOP -- TRYING THE MDP HEURISTIC.*/
#endif
            
#ifdef CHANGE_PHASE
            INTERACTIVE_VERBOSER(true, 15000, "Done MDP state expansion :: "<<std::endl);
#endif

    
    QUERY_UNRECOVERABLE_ERROR(0 == solvers[id]->peek__next_belief_state_for_expansion()//!solvers[id]->expansion_queue.size()
                              , "The problem is NULL, there is no starting state."<<std::endl
                              <<*actual_problem->second<<std::endl);
//     solvers[id]->expand_belief_state_space();

    
//     current_state[id] = solvers[id]->expansion_queue.front();
    current_state[id] = solvers[id]->peek__next_belief_state_for_expansion();//expansion_queue.front();
//     current_state[id] = solvers[id]->solve__for_new_starting_state(current_state[id]);

#ifdef LAO_STAR
            while(solvers[id]->lao_star()){};
#else
            current_state[id] = solvers[id]->solve__for_new_starting_state(current_state[id]);
#endif

    
// //     Planning::Policy_Iteration policy_Iteration(solvers[id]->belief_state__space);
//     Planning::Policy_Iteration__GMRES policy_Iteration(solvers[id]->belief_state__space
//                                                        , solvers[id]->get__sink_state_penalty());
//     for(uint i = 0; i < 100000; i++){
//         if(!solvers[id]->expand_belief_state_space()){
//             break;
//             VERBOSER(10017, "No starting state!"<<std::endl);
//         } else {
//             VERBOSER(10017, "Expanding!"<<std::endl);
//             //policy_Iteration();
// //             policy_Iteration.reset__converged();
// //             if(!(i % 10))policy_Iteration();
//         }
        
//         if(solvers[id]->belief_state__space.size() > 10000)break;
//     }
    
//     for(uint i = 0; i < 100; i++){
//         policy_Iteration();
//         VERBOSER(10017, "Expected reward is :: "
//                  <<current_state[id]->get__expected_value()<<std::endl);
//     }
    
// //     if(!solvers[id]->expand_belief_state_space()){
// //         UNRECOVERABLE_ERROR("I don't seem to have a starting state on the expansion queue."<<std::endl
// //                             <<*actual_problem->second<<std::endl);
// //     }
#endif
    
    VERBOSER(1001, "DTP Spawning the thread that posts actions to Moritz's system."<<std::endl);
    /*Planning is complete, now start \member{post_action} in a thread.*/
    spawn__post_action__thread(id);

    VERBOSER(2000, "DTP  START-UP  task "<<id<<" spawned");
    METHOD_RETURN;
}


void DTPCONTROL::cancelTask(Ice::Int id, const Ice::Current& ice_current)
{
    VERBOSER(2000, "DTP SHUT-DOWN cancel task "<<id<<std::endl);
    deliverObservation(id,
                       autogen::Planner::ObservationSeq(),
                       ice_current);
    VERBOSER(2000, "DTP  SHUT-DOWN cancel task "<<id<<" observation has been delivered."<<std::endl);
}

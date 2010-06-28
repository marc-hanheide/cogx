
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



void* DTPCONTROL__pthread_METHOD_CALLBACK(void* _argument)
{
    typedef std::tr1::tuple<int, DTPCONTROL*> Argument;
    auto argument = reinterpret_cast< Argument*>(_argument);
    auto task_id = std::tr1::get<0>(*argument);
    auto dtpcontrol = std::tr1::get<1>(*argument);
    
    dtpcontrol->post_action(task_id);

    delete argument;
    
    pthread_exit(_argument);
}

void DTPCONTROL::spawn__post_action__thread(Ice::Int id)
{
    
    auto thread = Thread(new pthread_t());
    auto attributes = Thread_Attributes(new pthread_attr_t());
    
    
    /*The thread at $id$ is wanted.*/
    thread_statuus[id] = true;
    Mutex mutex(CAST_THREADS::give_me_a_new__pthread_mutex_t());
    thread_mutex[id] = mutex;
    
    
    
    
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
                                        DTPCONTROL__pthread_METHOD_CALLBACK,
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
                sleep(1 * number_of_attempted_thread_invocations);
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

#define METHOD_PREFIX                           \
    if(!isRunning()){return;}                   \
    pthread_mutex_lock(destructor_mutex.get()); \
    lockComponent();                            \


#define METHOD_RETURN                                   \
    {                                                   \
    pthread_mutex_unlock(destructor_mutex.get());       \
    unlockComponent();                                  \
    return ;                                            \
    }                                                   \

#define LOOP_CLOSE                                      \
    {                                                   \
    pthread_mutex_unlock(destructor_mutex.get());       \
    unlockComponent();                                  \
    }                                                   \


DTPCONTROL::DTPCONTROL()
 :destructor_mutex(CAST_THREADS::give_me_a_new__pthread_mutex_t())
{
}

DTPCONTROL::~DTPCONTROL()
{
    while(cannot_be_killed()){
        sleep(1);
    }
    
    pthread_mutex_destroy(destructor_mutex.get());
}

bool DTPCONTROL::cannot_be_killed() const
{
    pthread_mutex_lock(destructor_mutex.get());
    threads.size();
    pthread_mutex_unlock(destructor_mutex.get());
}

void DTPCONTROL::deliverObservation(Ice::Int id,
                                    const autogen::Planner::ObservationSeq& observationSeq,
                                    const Ice::Current&){
    VERBOSER(1000, "DTP got an observation on  :: "<<id);
    METHOD_PREFIX;
    
    /* Supposed to be ignoring signals on to $id$.*/
    if(!thread_statuus[id]){
        METHOD_RETURN;
    }
    
    /* Task is being killed.*/
    if(observationSeq.size() == 0){
        thread_statuus[id] = false;
        VERBOSER(1000, "DTP observation was to kill task  :: "<<id);
        pthread_mutex_unlock(thread_mutex[id].get());
        METHOD_RETURN;
    }

    QUERY_UNRECOVERABLE_ERROR
        (thread_to_domain.find(id) != thread_to_domain.end()
         , "Could not find domain for task :: "<<id<<std::endl);

    
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
    
    VERBOSER(1000, "DTP observation now triggering action on task  :: "<<id);
    pthread_mutex_unlock(thread_mutex[id].get());
    
    METHOD_RETURN;
}

void  DTPCONTROL::post_action(Ice::Int id)
{
    VERBOSER(1000, "DTP posting actions on :: "<<id);
    
    while(thread_statuus[id]){
        pthread_mutex_lock(thread_mutex[id].get());

        METHOD_PREFIX;
        
        if(!thread_statuus[id]){
            break;
        }

        QUERY_UNRECOVERABLE_ERROR
            (thread_to_domain.find(id) != thread_to_domain.end()
             , "Could not find domain for task :: "<<id<<std::endl);
        
        Planning::Parsing::Problem_Identifier pi(thread_to_domain[id], thread_to_problem[id]);
        auto action = Planning::Parsing::problems[pi]->get__prescribed_action();

        
        auto name = std::tr1::get<0>(action.contents());
        std::ostringstream oss;
        oss<<name;
        auto action_name = oss.str();
        
        auto _arguments = std::tr1::get<1>(action.contents());
        std::vector<std::string> action_arguments;

        VERBOSER(1000, "DTP posting an action :: "<<action_name<<std::endl
                 <<"Arguments are :: "<<_arguments);
        for(auto argument = _arguments.begin()
                ; argument != _arguments.end()
                ; argument++){
            std::ostringstream oss;
            oss<<*argument;
            auto _argument = oss.str();
            
            action_arguments.push_back(_argument);
            VERBOSER(1000, "Arg :: "<<_argument);
        }
        
        /*Moritz, how do I go about this???*/
        autogen::Planner::PDDLAction* _pddlaction = new autogen::Planner::PDDLAction();
        autogen::Planner::PDDLActionPtr  pddlaction(_pddlaction);
//         pddlaction->name = action_as_string;
        pddlaction->name = action_name;
        pddlaction->arguments = action_arguments;
        pyServer->deliverAction(id, pddlaction);


        LOOP_CLOSE;
    }

    VERBOSER(1000, "DTP killing task  :: "<<id);
    
    pthread_mutex_unlock(thread_mutex[id].get()); 
    pthread_mutex_destroy(thread_mutex[id].get());
    pthread_attr_destroy(thread_attributes[id].get());
    thread_attributes.erase(id);
    threads.erase(id);
    thread_mutex.erase(id);
    thread_to_domain.erase(id);
    thread_to_problem.erase(id);
    
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
    VERBOSER(1000, "CAST runComponent called for DTP.");
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


void DTPCONTROL::newTask(Ice::Int id,
                         const std::string& problemFile,
                         const std::string& domainFile, const Ice::Current&)
{
    METHOD_PREFIX;
    
    VERBOSER(1000, "for task ::"<<id
             <<" got problem at :: "<<problemFile
             <<" and domain at :: "<<domainFile);


    VERBOSER(1000, "Parsing domain."<<std::endl);
    Planning::Parsing::parse_domain(domainFile);
    
    VERBOSER(1000, "Parsing problem."<<std::endl);
    Planning::Parsing::parse_problem(problemFile);

    VERBOSER(1000, "Attempting thread assignments."<<std::endl);
    thread_to_domain[id] = Planning::Parsing::domain_Stack->get__domain_Name();
    thread_to_problem[id] = Planning::Parsing::problem_Stack->get__problem_Name();
    
    VERBOSER(1000, "Spawning the thread that posts actions to Moritz's system."<<std::endl);
    /*Planning is complete, now start \member{post_action} in a thread.*/
    spawn__post_action__thread(id);

    METHOD_RETURN;
}


void DTPCONTROL::cancelTask(Ice::Int id, const Ice::Current& ice_current)
{
    VERBOSER(1000, "DTP cancel called for task :: "<<id<<std::endl);
    deliverObservation(id,
                       autogen::Planner::ObservationSeq(),
                       ice_current);
    
}

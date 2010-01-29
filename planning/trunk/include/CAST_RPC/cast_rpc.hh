/* Copyright (C) 2009 Charles Gretton (charles.gretton@gmail.com)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Dear CogX team member :: Please email (charles.gretton@gmail.com)
 * if you make a change to this and commit that change to SVN. In that
 * email, can you please attach the source files you changed as they
 * appeared before you changed them (i.e., fresh out of SVN), and the
 * diff files (*). Alternatively, you could not commit your changes,
 * but rather post me a patch (**) which I will test and then commit
 * if it does not cause any problems under testing.
 *
 * (*) see http://www.gnu.org/software/diffutils/diffutils.html --
 * GNU-09/2009
 *
 * (**) see http://savannah.gnu.org/projects/patch -- GNU-09/2009
 *
 *
 * ABOUT CAST_RPC :: 
 *
 * WARNING ** This library is written in c++-0x, and thus contains
 * WARNING ** instances of the "explicit" keyword. 
 *
 * NOTE ** Below, when we refer to CAST and \module{cast::*} we are
 * NOTE ** talking about the The CoSy Architecture Schema Toolkit versions 2.0
 * NOTE ** and above. These use ICE ("highly efficient middleware" -- it is an
 * NOTE ** alternative to CORBA, see \url{http://www.zeroc.com/iceVsCorba.html} )
 * NOTE ** rather than CORBA (Common Object Request Broker Architecture -- see
 * NOTE ** \url{http://en.wikipedia.org/wiki/Common_Object_Request_Broker_Architecture}
 * NOTE ** ). CAST is: "a software toolkit to support the developments of
 * NOTE ** intelligent systems based on a space of possible architecture
 * NOTE ** designs.". Basically, a very cool library written by:
 * 
 *     * Nick Hawes (first author -- n.a.hawes@cs.bham.ac.uk)
 *     * Michael Zillich
 *     * Patric Jensfelt
 *     * Henrik Jacobsson
 *     * Gregor Berginc
 * 
 *
 * Executively speaking, \namespace{CAST_RPC} gives a wrapper of
 * useful sugar for "subarchitecture" interactions via classical --
 * i.e., circa August 2009 -- CAST. It has been tested and modified to
 * work with all releases of CAST since then. It allows a CAST
 * component to interact with another cast component in a C-like
 * procedural way, and at the same time arbitrary components can snoop
 * (i.e., listen in on) the calls. It also supports a kind of dynamic
 * component instantiation.
 *
 * Now you say: "I have no idea of the meaning for what you just
 * wrote..."
 *
 * - You are a CAST component. Yes, really, you are. I.e., you appear
 *   as an argument to \keyword{add_cast_component} in a
 *   \filename{CMakeLists.txt} file. Moreover, after execution of
 *   \command{make -C BUILD} you appear somewhere with a ".so" suffix.
 *   As a CAST component you want to post some information to a
 *   different component in a procedural fashion. Consider, if you
 *   will, the following C code:
 *
 *   void foo(){ do_something();}
 *
 *   void bar(){foo(); do_something_else(); }
 *
 *   During execution, when \function{bar()} is invoked it blocks for
 *   the invocation of \function{foo()}, and then when
 *   \function{foo()} completes execution and returns control to
 *   \function{bar()}, \function{bar()} continues executing and
 *   \function{do_something_else()} is invoked. This is a lovely
 *   story, that even my 5 yo cousin can understand. Now suppose we
 *   are in CAST so that we want the above procedural semantics but
 *   have that:
 *   
 *   COMPONENT 1 (Angel): void foo(){ do_something();}
 *   
 *   COMPONENT 2 (Buffy): void bar(){foo(); do_something_else(); }
 *
 *   I.e., the call is made in C(omponent)2 and the implementation is
 *   in C1. Moreover, you might want arbitrary components to snoop
 *   (i.e., listen in on), the conversation between C1 and C2. To
 *   achieve this using CAST_RPC, the code looks as follows:
 *
 *   COMPONENT 1 (you): void foo(foo__SLICE_classnamePtr&){ do_something();}
 *   
 *   COMPONENT 2 (you): void bar(){call<foo__SLICE_classname>(); do_something_else(); }
 *
 *   NOTE: This example is bogus because you can't put an underscore
 *   in a slice classname. But you get the idea.
 *     
 * 
 * Now you say: "What about that `dynamic component instantiation`
 * stuff you mentioned..." -- Am I projecting or what?!
 *
 * -  You are a CAST component. Yes, really, you are. I.e., you appear
 *    as an argument to \keyword{add_cast_component} in a
 *    \filename{CMakeLists.txt} file. Moreover, after execution of
 *    \command{make -C BUILD} you appear somewhere with a ".so"
 *    suffix. ALSO, you are mentioned in a file with a .cast suffix that
 *    is likely to appear in a directory called "config". In that file
 *    you are given a name. For example, lets say your CPP name is X-CPP,
 *    and you appear in a \filename{CMakeLists.txt} as:
 *
 *       add_cast_component(X-CPP X-CPP.cc)
 *
 *    Then, in the file with the .cast suffix you are given a new name
 *    X-CAST-COMPONENT-NAME, and appear in a line along the lines of:
 *
 *       CPP MG X-CAST-COMPONENT-NAME X-CPP [SOME ARGUMENTS MAY OCCUR HERE]
 *
 *    ASIDE: Talk to Nick Hawes if you want to know what the "MG"
 *    stands for. I can never remember. Last time I checked they made
 *    nice motor vehicles. But that was back in the days when James
 *    Bond had hair on his chest.
 *
 *    You also occur in the .cast file below a line having the form:
 *
 *       SUBARCHITECTURE STRING-NAME-OF-SUBARCHITECTURE
 *
 *    If a cast component wants to post an object to you, they would have
 *    the following code in their C++ source implementation:
 *
 *    
 *              addToWorkingMemory( id, 
 *                                  "STRING-NAME-OF-SUBARCHITECTURE",
 *                                  ice_handle);
 *
 *    Problem is, you might want to react differently to different posted
 *    objects as the application execution progresses. For example, as a
 *    component you might be a factory that builds planning systems
 *    depending on the requirements in of a planning problem
 *    description. Once a planner is built, the factory would inform the
 *    client that requested planning of the identifier of the planning
 *    system just built. That system would listen for, and react to
 *    commands sent to its particular designation. Okay, so rather than
 *    making you write all that code each time you have such a factory
 *    like scenario, where a component builds dynamic components at
 *    runtime, we have encapsulated all this functionality in
 *    \class{procedure_implementation} and \class{procedure_call}.
 *
 * 
 *
 * Non-executively speaking and in more detail.. What is CAST_RPC?!
 * The author believes.. CAST (see \module{cast::*}) is
 * lovely.. Wow!\footnote{CMAKE is also lovely, however that is
 * another story for another day.}  Unfortunately the authors cannot
 * get their head around all of it, and it seems to lack some
 * functionality that the author required for their work. So, they
 * have got their head around a (perhaps) tiny piece of it that they
 * believe is all that is required for the automated planning parts of
 * CogX, and extended those aspects of CAST to support working-memory
 * interactions that have a C-like procedural semantics and dynamic
 * components.
 *
 * What does the author guarantee?.. that CAST_RPC appears to work
 * without falling over itself, under "normal" circumstances. Please
 * take note of the following technical detail.
 *
 * TECHNICAL NOTE:
 *
 *    Every procedure call is executed in a separate
 *    pthread (here 'p' stands for posix). Because no-one knows how to
 *    make threaded systems reliable, on many systems CAST_RPC will
 *    become unstable where there are a lot of procedure calls (i.e.,
 *    threads) activated. You say:"okay, give it to me straight doc,
 *    what's the endgame.. how long've I got?". If you are silly, and try
 *    to make deep recursive calls, the system will fall on itself (like
 *    a Klein bottle, but a little more dramatic); probably in an
 *    unpleasant way. So, if you are trying to compute the 15th Fibonacci
 *    number (i.e, 610), and doing this both recursively and three times
 *    in parallel across components and subarchitectures, then the system
 *    will fall on its face. Sometimes you will have a graceful crash
 *    where the output is:
 *    
 *          **    UNRECOVERABLE ERROR -- Can't implement a CAST_RPC procedure
 *                   call. Tried :: 5 times and still nothing...Basically,
 *                   we can't start a thread in which to execute the
 *                   procedure call. Better luck next time chief!
 *    
 *    On any other occasion, the system may simply hang, and
 *    then glib might get upset. If you run things in valgrind the
 *    crashes are, in my experience, always graceful.
 *                 
 *
 * Our main contribution is a template-based interface for
 * procedural-like client and server interactions. A class that
 * implements/serves a functionality will inherit from
 * \class{procedure_implementation}. A class that calls/is-a-client of
 * some functionality inherits from \class{procedure_call}. Of course,
 * a class can both call and implement procedures.
 *
 * A caller posts an object to working memory, and then blocks until
 * that specific object is modified by a call to
 * \class{cast::ManagedComponent\member{overwriteWorkingMemory}} made
 * by the \class{procedure_implementation}. A caller and callee should
 * never interact with CAST working memory, because that is bound to
 * cause problems. Rather, these interactions should all occur via
 * \class{procedure_implementation} and \class{procedure_call}.
 * Continuing with the "caller" story, a \class{procedure_call} then
 * reads data from the modified object and derives from that a return
 * value, and finally the caller deletes the object from working
 * memory. The implementation listens for objects of a specific
 * \slice{type} with a specific \CAST_RPC{designation}.
 *
 * Finally, one thing we cannot do here (in C++) is stop CAST users
 * shooting themselves in the foot when making CMakeLists.txt files,
 * and .cast files. Essentially, we cannot ensure that you have
 * specified a valid (i.e., implemented, etc.) subarchitecture
 * name. Moreover, we have popped a few symbols into the global
 * namespace, and therefore there may be a clash or two there if you
 * do the same thing -- i.e., \module{utilities.hh} overloads
 * \function{template<typename T> ostream operator<<(ostream&, const
 * vector<T>&)}, so if you have also implemented this, problems are
 * going to arise at runtime (a.k.a. crashtime).. unless CAST is
 * clever and able to resolve to a correct implementation -- in which
 * case there are going to be horrible horrible bugs that take ages to
 * find...
 */

#ifndef CAST_RPC_HH
#define CAST_RPC_HH

#include "utilities.hh"
#include "debug.hh"
#include "stl__tuple_hash.hh"
#include "cast__pthread_demangle.hh"

#include <cast/architecture.hpp>

// #include <pthread.h> // This shouldn't be necessary if you have the
// -std=c++0x switch (include -lpthread for safety) activated during
// GCC compilation.


namespace CAST_RPC
{

    /* A CAST object address is given by two items. One contains the
     * object's Id and the other contains the subarchitecture with
     * which it is associated.*/
    typedef decltype(cast::cdl::WorkingMemoryChange().address) Address;
    typedef decltype(cast::cdl::WorkingMemoryChange().address.id) Id;
    typedef decltype(cast::cdl::WorkingMemoryChange().address.id) Subarchitecture;
    typedef std::string Designator;
    typedef std::vector<Designator> Designators;

    /* The typing here is really weak. Because the \type{Id} and the
     * \type{Subarchitecture} have the same underlying type, they can
     * be mixed without the programmer knowing about it until runtime.
     * But there is nothing sensible we can do at this
     * stage. Something along the lines of what I commented below is
     * the start of a useful direction. At the moment, all checking of
     * address, subarchitecture, and ID validity is going to have to
     * be at crashtime. */
//     typedef decltype(cast::cdl::WorkingMemoryChange().address.id) _Id;
//     typedef decltype(cast::cdl::WorkingMemoryChange().address.id) _Subarchitecture;    
    
//     class Id : public _Id
//     {
//     public:
//         Id(const std::string&) = delete;
//         Id(std::string&&) = delete;
//         Id(const Id& id)
//             :_Id(id){};
//     };
    
//     class Subarchitecture : public _Subarchitecture
//     {
//     public:
//         Id(const std::string&) = delete;
//         Id(std::string&&) = delete;
//         Id(const Subarchitecture& subarchitecture)
//             :_Subarchitecture(subarchitecture){};
//     };

    /* An \class{Address} identifies a \module{cast::*} managed
     * object. It can be achieved in many many ways. The following
     * definitions provide a factory for getting an \class{Address}.*/
    class _recover_address
    {
    public:
        template<typename ...T>
        std::shared_ptr<Address> operator()(const T&... t) const ;
    };
    
    template<>
    std::shared_ptr<Address> _recover_address::operator()<Address>
    (const Address& address) const
    {
        std::shared_ptr<Address> result(new Address());
        result->id = address.id;
        result->subarchitecture = address.subarchitecture;
        return result;
    }
    
    
    template<>/* FIX ::
              *
              *
              * NOTE:: (Wed Sep 30 09:49:15 BST 2009) I wrote that FIX
              * in a hurry it seems, because I am not sure what I am
              * supposed to be fixing. The creating on a new
              * \class{Address} perhaps?!
              */
    std::shared_ptr<Address> _recover_address::operator()<cast::cdl::WorkingMemoryChange>
    (const cast::cdl::WorkingMemoryChange& wmc) const
    {
        std::shared_ptr<Address> result(new Address());
        result->id = wmc.address.id;
        result->subarchitecture = wmc.address.subarchitecture;
        return result;
    }
    
    
    template<>
    std::shared_ptr<Address> _recover_address::operator()<std::tr1::tuple<const Id&, const Subarchitecture&> >
        (const std::tr1::tuple<const Id&, const Subarchitecture&>& address) const
    {
        std::shared_ptr<Address> result(new Address());
        result->id = std::tr1::get<0>(address);
        result->subarchitecture =  std::tr1::get<1>(address);
        
        return result;
    }
    
    /* Sometimes it is better when C++ is self-documenting... see
     * implementation of \class{_recover_address}
     * \function{operator()} directly below this type definition.*/
    typedef const std::string& (cast::ManagedComponent::*ManagedComponent_member_function_pointer)(void) const;
    
    template<>
    std::shared_ptr<Address> _recover_address::operator()<std::tr1::tuple<const Id&
                                                           , const cast::ManagedComponent&
                                                           , const ManagedComponent_member_function_pointer&> >
        (const std::tr1::tuple<const Id&
         , const cast::ManagedComponent&
         , const ManagedComponent_member_function_pointer&>& address) const
    {
        std::shared_ptr<Address> result(new Address());
        
        result->id = std::tr1::get<0>(address);
        
        const cast::ManagedComponent& managedComponent = std::tr1::get<1>(address);
        
        const ManagedComponent_member_function_pointer& managedComponent_member_function_pointer
            = std::tr1::get<2>(address);
        
        result->subarchitecture
            = (managedComponent.*managedComponent_member_function_pointer)();
        
        return result;
    }    
    
    template<typename ...T>
    std::shared_ptr<Address> _recover_address::operator()
    (const T&... t) const
    {
        return (*this)(std::tr1::tuple<const T&...>(t...));
    }



    /* If you start a thread that will invoke
     * \member{__receive_call()} of \class{_receive_call}, then this
     * is what you should use.
     *
     * WHY?! CAST blocks when it invokes a member function based on
     * "change filter". So for example, if you ask CAST to call
     * \function{X::foo()} when an overwrite happens to \object{o},
     * then if two overwrites occur, the invocations of
     * \function{X::foo()} are serialised.*/
    template<typename THING, typename FUNCTION, typename ICE_FUNCTION_CLASS, class PARENT>
    void* _receive_call__pthread_METHOD_CALLBACK____receive_call(void* _argument);
    
    
    /* The following (\class{_receive_call}) is a *utility* class
     * associated with \class{procedure_implementation}.
     *
     * When a \namespace{CAST_RPC} \class{procedure_call} is made,
     * it's implementation will be invoked by
     * \member{operator()(...)}. A single "implementation" (i.e.,
     * \class{procedure_implementation}) should only invoke one
     * procedure at a time. This functionality uses the
     * \argument{mutex} to enforce this.
     */
    template<typename THING/*Probably a cast::ManagedComponent is also a \class{procedure_implementation}*/,
             typename FUNCTION/*Member function from \type{THING}*/,
             typename ICE_FUNCTION_CLASS/*SLICE classname*/,
             class PARENT = NA>
    class _receive_call : public PARENT 
    {
    public:
        friend void* _receive_call__pthread_METHOD_CALLBACK____receive_call<THING, FUNCTION, ICE_FUNCTION_CLASS, PARENT>(void*);
        
    private:
        
        void __receive_call(const cast::cdl::WorkingMemoryChange& _wmc)
        {
            
            VERBOSER(601, "Pending call.");
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_lock(mutex.get()),
                                              "Unable to lock mutex.");

            
            VERBOSER(601, "Making call :: "
                     <<mutex.get()<<std::endl
                     <<_wmc.address.id
                     <<" "<<_wmc.address.subarchitecture
                     <<std::endl);
            
            VERBOSER(200, "Locked _receive_call mutex:: "
                     <<mutex.get()<<std::endl);
            
            VERBOSER(201, " Implementing procedure call for --  :: "
                     <<_wmc.address.id
                     <<" "<<_wmc.address.subarchitecture
                     <<std::endl);

            //if(managed_Component->intended_audience_test(std::string("")));

            
            auto argument
                = dynamic_cast<cast::ManagedComponent*>(managed_Component)
                ->getMemoryEntry<ICE_FUNCTION_CLASS>(_wmc.address);


            /* Make sure we are supposed to implement this procedure call.*/
            auto argument_designation = argument->optionalMemberDesignatorIsAnArgument;
            if(!managed_Component->intended_audience_test(argument_designation)){
                VERBOSER(200, "Rejecting a procedure call...");
                
                QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(mutex.get()),
                                              "Unable to unlock mutex.");
                return;
            };

            /* If we are supposed to accept this procedure call, but
             * no target designation was specified, then we revert to
             * the default designations (see
             * \member{managed_Component->get_designators()}). */
            if(argument->optionalMemberDesignatorIsAnArgument.empty()){
                WARNING("No designation specified, so defaulting to :: "
                        <<managed_Component->get_designators()<<"   "<<std::endl);
                
                argument->optionalMemberDesignatorIsAnArgument
                    = managed_Component->get_designators();
            }
            
            VERBOSER(501, " PENDING -- Overwriting working memory :: "
                     <<_wmc.address.id
                     <<" "<<_wmc.address.subarchitecture
                     <<std::endl);
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(mutex.get()),
                                      "Unable to unlock mutex.");
                
            (managed_Component->*function)(argument);
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_lock(mutex.get()),
                                      "Unable to unlock mutex.");
            
            VERBOSER(601, "  START -- Overwriting working memory :: "
                     <<_wmc.address.id
                     <<" "<<_wmc.address.subarchitecture
                     <<std::endl);
            
            managed_Component
                ->overwriteWorkingMemory(_wmc.address.id,
                                         _wmc.address.subarchitecture,
                                         argument);

            
            VERBOSER(601, "  DONE -- Overwriting working memory :: "
                     <<_wmc.address.id
                     <<" "<<_wmc.address.subarchitecture
                     <<std::endl);
            
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(mutex.get()),
                                      "Unable to unlock mutex.");
            
            VERBOSER(601, "Unlocked _receive_call mutex :: "
                     <<mutex.get()<<std::endl);
        }
    public:
        _receive_call(THING* managed_Component,
                      FUNCTION function,
                      std::shared_ptr<pthread_mutex_t> mutex):
            managed_Component(managed_Component),
            function(function),
            mutex(mutex)
        {};

        ~_receive_call(){

            
        };


        /* (see the private \member{__receive_call()}). This spawns a
         * thread that calls \member{__receive_call()}, and then
         * returns immediately. Thus, we overcome the serial semantics
         * that \module{cast::*} has for calling event handlers.*/
        void operator()(const cast::cdl::WorkingMemoryChange& _wmc)
        {

            
            
            VERBOSER(499, "Trying to receive a call...");
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_lock(mutex.get()),
                                              "Unable to lock mutex.");
            
            /* \module{cast::*} will make the argument _wmc invalid
             * sometime after this thread of execution completes
             * execution of this member function. Because of that, I
             * have to copy the argument details and operate on the
             * copy.
             *
             * Nick Hawes (Sep 29): \argument{_wmc} is only an event
             * struct -- i.e. a small amount of information about the
             * change that has occurred. It does not hold the slice
             * object data or an encapsulated connection to working
             * memory. It just obeys all the usual C rules about
             * memory management. In this case the WorkingMemoryChange
             * object passed in as _wmc will eventually go out of
             * scope in the calling class and then your pointer will
             * point to garbage. Hope that helps.
             */
            cast::cdl::WorkingMemoryChange wmc = _wmc;//new cast::cdl::WorkingMemoryChange(__wmc);
            
            
            VERBOSER(499, "Entered call reception...");
            
//             /* Only exercised on first invocation.*/
//             if(my_Threads.empty())number_of_elements_in__my_threads =  0;
            
            auto thread = std::shared_ptr<pthread_t>(new pthread_t());
            
            auto attributes = std::shared_ptr<pthread_attr_t>(new pthread_attr_t());
            
//             auto thread_mutex = std::shared_ptr<pthread_mutex_t>(give_me_a_new__pthread_mutex_t());
            
//             if(my_Threads.size() <= number_of_elements_in__my_threads){
//                 assert(my_Threads.size() + 1 > number_of_elements_in__my_threads);
                
//                 VERBOSER(499, "Added entry to implementation thread mutexes...");
//                 my_Threads.push_back(My_Thread_Details());
//             }
            
//             VERBOSER(499, "I have a thread with some attributes and a mutex...");
//             my_Threads[number_of_elements_in__my_threads++] = My_Thread_Details(thread, attributes, thread_mutex);
            
            VERBOSER(499, "Now initialising the thread attributes...");
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_init(attributes.get()),
                                      "Failed to initialise thread attributes...");
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setdetachstate(attributes.get(), PTHREAD_CREATE_DETACHED),
                                      //PTHREAD_CREATE_JOINABLE
                                      "Making a thread that we don't have to join...");   
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setscope(attributes.get(), PTHREAD_SCOPE_SYSTEM),//PTHREAD_SCOPE_PROCESS),
                                      //PTHREAD_SCOPE_SYSTEM
                                      "Give the thread a process scope...");   

            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_setinheritsched(attributes.get(), PTHREAD_INHERIT_SCHED),
                                      //PTHREAD_SCOP_SYSTEM
                                      "Make the thread execution schedule, that of its parent...");
    
            VERBOSER(499, "Preparing to pass-on the thread mutex...");

            
            typedef _receive_call<THING/*implementation*/,
                FUNCTION/*member function*/,
                ICE_FUNCTION_CLASS,
                PARENT> Receive_Call;
            typedef std::tr1::tuple<Receive_Call*
                , cast::cdl::WorkingMemoryChange
                //, std::shared_ptr<pthread_mutex_t>
                > Argument;

            /* ALLOCATING -- 40598 (i.e., look for DELETING -- 40598 ) */
            Argument* _tuple = new Argument(this, wmc);//, thread_mutex);
            
//             auto _tuple =
//                 new std::tr1::tuple
//                 <decltype(this), decltype(wmc), decltype(thread_mutex)>
//                 (this, wmc, thread_mutex);

//             VERBOSER(499, "Locking the thread mutex...");
//             /*Lock the mutex associated with a new thread we are about to spawn.*/
//             QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_lock(thread_mutex.get()), "Unable to lock mutex.");
            
//             join_threads_in__my_Threads__that_are_completed();
            
            /*Attempt to spawn a new thread.*/
            auto result = 0;
            auto number_of_attempted_thread_invocations = 0;
            
            VERBOSER(499, "Starting spinning to create a thread...");
            while(0 != (result = pthread_create(thread.get(),
                                             attributes.get(),
                                             _receive_call__pthread_METHOD_CALLBACK____receive_call
                                             <THING, FUNCTION, ICE_FUNCTION_CLASS, PARENT>,
                                             _tuple))){

                VERBOSER(299, "Spinning to create a thread...");
            
//                 UNRECOVERABLE_ERROR("Unable to spawn a thread.");

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
                        //usleep(10 * number_of_attempted_thread_invocations);
                        break;
                }
                
//                 join_threads_in__my_Threads__that_are_completed();
                
                number_of_attempted_thread_invocations++;
                VERBOSER(601, number_of_attempted_thread_invocations<<" attempted thread invocations.");

                QUERY_UNRECOVERABLE_ERROR(number_of_attempted_thread_invocations > 4/*FIX : Magic number*/,
                                          "Can't implement a CAST_RPC procedure call. Tried :: "
                                          <<number_of_attempted_thread_invocations
                                          <<" times and still nothing..."
                                          <<"Basically, we can't start a thread in which to "
                                          <<"execute the procedure call. Better luck next time chief!");

            }
            VERBOSER(499, "Finished spinning to create a thread...");
            
            /* We do not wait to join the thread just spawned, as that
             * is indeed the point of spawning it, to get around the
             * \module{cast::*} problem of blocking on a call to an
             * event handler -- i.e., A member function that is
             * invoked by \module{cast::*} when an
             * ADD/DELETE/OVERWRITE signal occurs on a SLICE object
             * under its management.*/       
            VERBOSER(499, "Releasing mutex...");
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(mutex.get()),
                                      "Unable to unlock mutex.");
        }
    private:
//         typedef std::tr1::tuple<std::shared_ptr<pthread_t>
//                                 , std::shared_ptr<pthread_attr_t>
//                                 , std::shared_ptr<pthread_mutex_t> > My_Thread_Details;
        
//         /* To get around the CAST serialisation constraint -- i.e.,
//          * that a \class{ManagedComponent} can only receive one event
//          * from a change filter at a time -- we employ threads. This
//          * is the list of threads we thusly employ. Yes, thusly might
//          * be a real world in the English language, rather than, for
//          * example, an integral world, or some such. Needles to say
//          * needless to say, but including it here helps me, the
//          * author, a whole bunch when I am grepping for this
//          * functionality. So I would really appreciate it if you, a
//          * person, or class of persons who is not the author, not to
//          * use it in your own comments. Incidentally, while writing
//          * this I believed "thusly" meant: "in the way indicated".*/
//         std::vector<My_Thread_Details> my_Threads;
        
//         /* (see \member{my_Threads}). Standard trick, we don't resize
//          * the vector, but rather keep a prefix of valid entries, and
//          * the following item stores the length of that prefix.*/
//         uint number_of_elements_in__my_threads;
        
        
//         inline void join_threads_in__my_Threads__that_are_completed()
//         {
//             QUERY_UNRECOVERABLE_ERROR(0 == pthread_mutex_trylock(mutex.get()),
//                                       "Procedure call mutex should have been locked.");
            
//             VERBOSER(499, "Joining threads that we spawned...");

// //             return;/*FIX :: Remove this return, and get the
// //                     * functionality in this function happening...*/
            
//             for(uint my_Thread__index = 0
//                     ; my_Thread__index < number_of_elements_in__my_threads
//                     ; my_Thread__index++){
                    
//                 assert(my_Threads.size() > my_Thread__index);
                
//                 auto my_Thread = my_Threads[my_Thread__index];

//                 auto result = 0;
                
//                 VERBOSER(499, "Waiting for a thread that we spawned earlier...");
//                 /*Has a thread that I spawned finished executing.*/
//                 if(0 == (result = pthread_mutex_trylock(std::tr1::get<2>(my_Thread).get()))){
//                     VERBOSER(499, "Joining a thread that we spawned earlier...");
                        
                        
//                     VERBOSER(499, "Joining a thread that we spawned earlier...");
//                     QUERY_UNRECOVERABLE_ERROR(0 != pthread_join(*std::tr1::get<0>(my_Thread).get(), 0),
//                                               "Unable to join a thread.\n");
                        
//                     VERBOSER(499, "Attribute destruction for a thread that we spawned earlier...");
//                     QUERY_UNRECOVERABLE_ERROR(0 != pthread_attr_destroy(std::tr1::get<1>(my_Thread).get()),
//                                               "Unable to destroy some thread attributes.\n");
                        
//                     VERBOSER(499, "Unlocking thread related mutex...");
//                     QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(std::tr1::get<2>(my_Thread).get()),
//                                               "Can't lock mutex that we just locked. Strange...");
                    
//                     VERBOSER(499, "Destroying thread related mutex...");
//                     QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_destroy(std::tr1::get<2>(my_Thread).get()),
//                                               "Unable to destroy mutex.");
                        
//                     VERBOSER(499, "Done cleanup...");
                    
//                     assert(my_Threads.size() >= number_of_elements_in__my_threads);

//                     assert(number_of_elements_in__my_threads > 0);
//                     assert(number_of_elements_in__my_threads > 1);
//                     assert(my_Thread__index <= (number_of_elements_in__my_threads - 1));

//                     if((number_of_elements_in__my_threads - 1) == my_Thread__index){
//                         --number_of_elements_in__my_threads;                        
//                     } else {
//                         assert(my_Thread__index < (number_of_elements_in__my_threads - 1));
//                         /* Move the last element in the array to take
//                          * up the position of the just-now redundant
//                          * element.*/
//                         my_Threads[my_Thread__index]
//                             = std::move(my_Threads[--number_of_elements_in__my_threads]);
//                     }
                    
//                 } else {
//                     switch(result){
//                         case EBUSY:
//                             VERBOSER(499, "Still waiting for busy mutex...");
//                             break;
//                         case EINVAL:
//                             UNRECOVERABLE_ERROR("Locking on mutex that is not an initialised.");
//                             break;
//                         case EFAULT:
//                             UNRECOVERABLE_ERROR("Locking on mutex that is an invalid pointer.");
//                             break;
//                         default:
//                             UNRECOVERABLE_ERROR("Attempted to lock on a mutex and got a nonsense result."
//                                                 <<" Something bad has happened, so we are aborting "
//                                                 <<"-- sorry Ludwig van Beethoven.");
//                             break;
//                     }
//                 }
//             }
//         }
            
    private:
        /*CAST-based managed component that implements the procedure call.*/
        THING* managed_Component;
        FUNCTION function;
        std::shared_ptr<pthread_mutex_t> mutex;
    };

    template<typename THING, typename FUNCTION, typename ICE_FUNCTION_CLASS, class PARENT>
    void* _receive_call__pthread_METHOD_CALLBACK____receive_call(void* _argument)
    {
        VERBOSER(299, "Executing receive call...");
        
        typedef _receive_call<THING, FUNCTION, ICE_FUNCTION_CLASS, PARENT> Receive_Call;
        typedef std::tr1::tuple<Receive_Call*
            , cast::cdl::WorkingMemoryChange/*const cast::cdl::WorkingMemoryChange& (is UNSAFE)*/
//             , std::shared_ptr<pthread_mutex_t>
//             , std::shared_ptr<pthread_t>
//             , std::shared_ptr<pthread_attr_t>
            > Argument;
        
        VERBOSER(299, "Recovering argument...");
        auto argument = static_cast<Argument* >(_argument);
        VERBOSER(299, "Recovered argument...");
        VERBOSER(299, "Recovering caller...");
        auto receive_Call = std::tr1::get<0>(*argument);//->first;
        VERBOSER(299, "Recovering working memory change...");
        auto workingMemoryChange = std::tr1::get<1>(*argument);//->second;
//         VERBOSER(299, "Recovering thread mutex...");
//         auto mutex = std::tr1::get<2>(*argument);
        
        VERBOSER(299, "Making invocation...");
        receive_Call->__receive_call(workingMemoryChange);

//         VERBOSER(299, "Unlocking mutex...");
//         QUERY_UNRECOVERABLE_ERROR((0 != pthread_mutex_unlock(mutex.get())),
//                                           "Failed to unlock mutex!!");

        /* DELETING -- 40598 */
        delete argument;
        
        /*Threads execution has completed.*/
        pthread_exit(_argument);
        //return 0;
    }
    
    /* A \module{cast::*}-based procedure call is either "made" (i.e.,
     * make a call) "implemented" (i.e., we implement the call)
     * locally or globally.
     *
     * NOTE : I do not know the distinction between \module{cast::*}
     * "local" v.s. "global". So I wrote the code below to support
     * both options.. defaulting to "local".
     *
     * \begin{conversation -- Aug 18 2009}
     *
     * Charles Gretton: Is there a rule-of-thumb where to use local
     * v.s. global?
     *
     * Nick Hawes: People just do what they fancy really. Local saves
     * cycles, but then components always have to be in the same
     * subarchitecture.
     *
     * \end{conversation}
     *
     * Right.. okay.. so I remain clueless ;)
     *
     */
    enum class Locality
    {
        Global,
        Local
    };
    

    /* Under the rpc-scheme, you either call or implement a
     * "procedure". Procedure implementation must be invoked during
     * execution of the \parent{cast::ManagedComponent} 
     * \method{start()}.
     *
     * Example of usage: Suppose you have a SLICE
     * \class{helloworld::Announcement}. Moreover, suppose this is a
     * rpc-scheme class, and thus conceptually encapsulates a c-like
     * procedure call. To implement that call using a, let's call it
     * \function{HelloReader::makeAnnouncement()}, the following code
     * would appear in \child{your} \method{start()}:
     * 
     * implement<helloworld::Announcement>(&HelloReader::makeAnnouncement);
     *
     */
    template<typename CHILD
             , Locality locality = Locality::Local>
    class procedure_implementation : public virtual cast::ManagedComponent
    {
    public:
        /* A type for objects that can be deleted.*/
        class can_be_deleted { public:  virtual ~can_be_deleted() {}; };
    private:
        /* Each procedure call is invoked by a wrapper (see
         * \class{_receive_call}) that can be deleted.*/
        std::vector<std::shared_ptr<can_be_deleted> > to_be_deleted;

        /* We can only implement one call at a time.*/
        std::shared_ptr<pthread_mutex_t> mutex;

        /* If a procedure call specified designators -- i.e., SLICE
         * class \SLICE_member{optionalMemberDesignatorIsAnArgument}
         * is non-empty -- then an \class{procedure_implementation}
         * only implements the procedure call where the elements of
         * \SLICE_member{optionalMemberDesignatorIsAnArgument}
         * intersects with those of \member{designators}.*/
        Designators designators;
    public:

        /*(see \member{designators})*/
        const Designators& get_designators() const
        {
           return designators; 
        }
        
        
        explicit procedure_implementation(const Designator& designator = "")
            :mutex(give_me_a_new__pthread_mutex_t())
        {
            
            VERBOSER(401, "Got lvalue designator :: "<<designator);
            
            if("" == designator){
                designators  = decltype(designators)();
            } else {

                Designators _designators;
                _designators.push_back(designator);
                
                designators  = std::move(_designators);//{designator};
            }
            
            
            VERBOSER(401, "LVALUE -- Created new procedure implementation with mutex :: "<<mutex.get()
                     <<" and designators :: "<<get_designators());
        }
        
        explicit procedure_implementation(Designator&& designator = Designator(""))
            :mutex(give_me_a_new__pthread_mutex_t())
        {
            
            VERBOSER(401, "Got rvalue designator :: "<<designator);
            
//             if("" == designator)
//                 designators  = decltype(designators)();
//             else
//                 designators  = {std::move(designator)};

            
            if("" == designator){
                designators  = decltype(designators)();
            } else {

                Designators _designators;
                _designators.push_back(std::move(designator));
                
                designators  = std::move(_designators);//{designator};
            }
            
            
            VERBOSER(401, "RVALUE -- Created new procedure implementation with mutex :: "<<mutex.get()
                     <<" and designators :: "<<get_designators());
            
        }

        ~procedure_implementation()
        {
            CAST__QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_destroy(mutex.get()),
                                              "Unable to destroy mutex.");
        }

        /* Are we the intended \argument{audience} of a procedure
         * call? -- I.e. were we designated to receive a calls sent to
         * \argument{audience}?*/
        bool intended_audience_test(const Designator& audience) const
        {
             QUERY_UNRECOVERABLE_ERROR(0 == pthread_mutex_trylock(mutex.get()),
                                       "Procedure call mutex should have been locked.");
            
            /* If we have not been specified to accept particular
             * designators, then we accept all invocations.*/
            if(designators.empty())return true;

            /* If the \argument{audience} is non-specific, then we
             * also accept the invocation.*/
            if("" == audience)return true;

            /* If at some point we have been told to accept any invocations...*/
            if(designators.end() != std::find(designators.begin(), designators.end(), "")) return true; 
            
            auto designators_iterator = std::find(designators.begin(), designators.end(), audience);

            /* Otherwise, we only accept the invocation if we have
             * been specified to do so.*/
            return designators_iterator != designators.end();
        }
        
        /* If this has specified \member{designators}, then where the
         * audiences intersect with \member{designators}. */
        bool intended_audience_test(const decltype(designators)& audiences) const
        {
             QUERY_UNRECOVERABLE_ERROR(0 == pthread_mutex_trylock(mutex.get()),
                                       "Procedure call mutex should have been locked.");
             
            if(designators.empty())return true;
            if(audiences.empty())return true;
            if(designators.end() != std::find(designators.begin(), designators.end(), "")) return true; 
            if(audiences.end() != std::find(audiences.begin(), audiences.end(), "")) return true; 
            
            decltype(designators) result;
            std::set_intersection( audiences.begin(), audiences.end(),
                                   designators.begin(), designators.end(),
                                   inserter(result,result.begin()) );
            
            return !result.empty();
        }        

        /* Make this implement calls with a \argument{designator}
         * designation.*/
        void add_designator(const Designator& designator)
        {
             QUERY_UNRECOVERABLE_ERROR(0 == pthread_mutex_trylock(mutex.get()),
                                       "Procedure call mutex should have been locked.");
             
            if("" == designator) return;
            
            designators.push_back(designator);
        }
        
        /* (see \member{add_designator()})*/
        void add_designator(Designator&& designator)
        {
             QUERY_UNRECOVERABLE_ERROR(0 == pthread_mutex_trylock(mutex.get()),
                                       "Procedure call mutex should have been locked.");
             
            if("" == designator) return;
            
            designators.push_back(std::move(designator));
        }
        
        
        /* Declare that we implement the function of
         * \type{ICE_FUNCTION_CLASS} using member
         * \function{function}. This _must only_ be called by a
         * \class{cast::ManagedComponent} \method{start()}. It is
         * _not_ thread safe... .*/
        template<typename ICE_FUNCTION_CLASS, typename FUNCTION_TYPE>
        void implement(FUNCTION_TYPE function)
        {

            typedef _receive_call<CHILD,   /*Class that implements procedure calls for decltype(*this).*/
                FUNCTION_TYPE,             /*Child's member function that is to be called.*/
                ICE_FUNCTION_CLASS,        /*SLICE class, that we are listening for.*/
                can_be_deleted             /*A procedure call can be deleted.*/
                > __receive_call;
            
            to_be_deleted
                .push_back(std::shared_ptr<can_be_deleted>
                           (new __receive_call(dynamic_cast<CHILD*>(this),
                                               function,
                                               mutex)
                            )
                           );
            
            
            
            switch(locality){
                case Locality::Global:
                    
                {
                    CAST__VERBOSER(12, "Not expecting global components...");
                    
                    CAST__VERBOSER(4, "Procedure with mutex :: "<<mutex.get()
                             <<" is looking for _GLOBAL_ adds of type :: "
                             <<DEMANGLE_TYPE(ICE_FUNCTION_CLASS));
                    
                    auto type_filter =  cast::createGlobalTypeFilter<ICE_FUNCTION_CLASS>(cast::cdl::ADD);

                    
#define procedure_implementation__implement__IMPLEMENTATION______addChangeFilter                \
                    addChangeFilter(type_filter,                                                \
                                    new cast::MemberFunctionChangeReceiver<__receive_call>      \
                                    (dynamic_cast<__receive_call*>(to_be_deleted.back().get()), \
                                     &__receive_call::operator()));                             \
                    
                    
                    procedure_implementation__implement__IMPLEMENTATION______addChangeFilter;
                }
                
                    
                break;
                case Locality::Local:
                    
                {

                    
                    CAST__VERBOSER(4, "Procedure with mutex :: "<<mutex.get()
                             <<" is looking for _LOCAL_ adds of type :: "
                             <<DEMANGLE_TYPE(ICE_FUNCTION_CLASS));
                    
                    auto type_filter =  cast::createLocalTypeFilter<ICE_FUNCTION_CLASS>(cast::cdl::ADD);
                
                    procedure_implementation__implement__IMPLEMENTATION______addChangeFilter;  
                }
                
                    
                break;
                default:
                    CAST__UNRECOVERABLE_ERROR("Component has unknown locality.");
                    break;
            }
        }
    };
    
    
    template<typename ADDRESS_RECOVERY = _recover_address>
    class procedure_call : public virtual cast::ManagedComponent
    {
    public:
        int i ;
        procedure_call()
            :procedure_call____MUTEX(give_me_a_new__pthread_mutex_t())
        {

#define LOCK_ACCESS_TO_MEMBER_DATA                                      \
            {                                                           \
                CAST__QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_lock(procedure_call____MUTEX.get()), \
                                                "Unable to lock procedure_call____MUTEX."); \
            }                                                           \
            
#define UNLOCK_ACCESS_TO_MEMBER_DATA                                    \
            {                                                           \
                CAST__QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(procedure_call____MUTEX.get()), \
                                                "Unable to unlock procedure_call____MUTEX."); \
            }                                                           \



// #define LOCK_ACCESS_TO_MEMBER_DATA                                      \
//             {                                                           \
//                 sleep(1);std::cerr<<'@'<<__PRETTY_FUNCTION__<<std::endl; \
//                 CAST__QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_lock(procedure_call____MUTEX.get()), \
//                                                 "Unable to lock procedure_call____MUTEX."); \
//                 i++;                                                    \
//                 sleep(1);std::cerr<<i<<__PRETTY_FUNCTION__<<std::endl;                                           \
//             }                                                           \
            
// #define UNLOCK_ACCESS_TO_MEMBER_DATA                                    \
//             {                                                           \
//                 sleep(1);std::cerr<<'#'<<__PRETTY_FUNCTION__<<std::endl;       \
//                 int j = i;                                              \
//                 CAST__QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(procedure_call____MUTEX.get()), \
//                                                 "Unable to unlock procedure_call____MUTEX."); \
//                 sleep(1);std::cerr<<j<<__PRETTY_FUNCTION__<<std::endl;                                           \
//             }                                                           \
            
            
// #define LOCK_ACCESS_TO_MEMBER_DATA  ;   
            
// #define UNLOCK_ACCESS_TO_MEMBER_DATA ;
            
        }
        
        
        ~procedure_call()
        {
            CAST__QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_destroy(procedure_call____MUTEX.get()),
                                              "Unable to destroy mutex.");
            
            for(auto more_mutexes = mutexes.begin()
                    ; more_mutexes != mutexes.end()
                    ; more_mutexes++){
                for(auto even_more_mutexes = more_mutexes->second.begin()
                        ; even_more_mutexes != more_mutexes->second.begin()
                        ; even_more_mutexes++){
                    CAST__VERBOSER(1, "Attempting the destruction of a mutex.");
                    CAST__QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_destroy(even_more_mutexes->second.get()),
                                              "Unable to destroy mutex.");
                }
            }

            
        }
        
        typedef procedure_call<ADDRESS_RECOVERY> THIS__TYPE;
        typedef void (THIS__TYPE::*THIS__FUNCTION__WMC__TO__VOID)
            (const cast::cdl::WorkingMemoryChange&);        

        
#define procedure_call____IMPLEMENT_FROM__NOT_AN_ADDRESS(FUNCTION_NAME, FORWARD_ANNOTATION, BACK_ANNOTATION) \
        template<typename ...T> \
        FORWARD_ANNOTATION FUNCTION_NAME(const T&... t) BACK_ANNOTATION \
        { \
            return FUNCTION_NAME(*recover_address(t...)); \
        } \

        
        /* A \function{call} is a wrapper to a procedure call using
         * the distributed stuff of \module{cast::*}. Here, the first
         * argument specifies the subarchitecture to which the
         * procedure call shall be made. */
        template<typename ICE_FUNCTION_CLASS, typename ...ARGS>
        IceInternal::Handle<ICE_FUNCTION_CLASS> call_to_subarchitecture
        (const Subarchitecture& subarchitecture,
         ARGS&&... t)
        {   
            ICE_FUNCTION_CLASS* functional = new ICE_FUNCTION_CLASS(std::forward<ARGS>(t)...);
            
            /* CHECK :: Below, I am using IceInternal (resp. IceUtil)
             * smart pointers because I have to. But I am not sure of
             * the legitimacy of the code...*/
#define procedure_call____IMPLEMENTATION__call                          \
            typedef typename IceInternal::Handle<ICE_FUNCTION_CLASS> Ice_Handle_Type; \
            /*Let cast manage the deletion of the object at \local{functional}.*/ \
            Ice_Handle_Type ice_Handle(functional);                     \
            return _call_to_subarchitecture<Ice_Handle_Type> \
                (subarchitecture, ice_Handle);               \
            
            
            procedure_call____IMPLEMENTATION__call;
        }

        /* As above, only without the move semantics. */
        template<typename ICE_FUNCTION_CLASS, typename ...ARGS>
        IceInternal::Handle<ICE_FUNCTION_CLASS> call_to_subarchitecture
        (const Subarchitecture& subarchitecture,
         const ARGS&... t)
        {
            ICE_FUNCTION_CLASS* functional = new ICE_FUNCTION_CLASS(t...);
            
            procedure_call____IMPLEMENTATION__call;
        }

        /* (see \function{call_to_subarchitecture}) -- Here, the
         * subarchitecture is not specified, and we default to the
         * result of a
         * \function{cast::ManagedComponent::getSubarchitectureID()}
         * call via \parent{cast::ManagedComponent}.*/
        template<typename ICE_FUNCTION_CLASS, typename ...ARGS>
        IceInternal::Handle<ICE_FUNCTION_CLASS> call(ARGS&&... t)
        {
            CAST__VERBOSER(17, "");
            
            Subarchitecture subarchitecture = getSubarchitectureID();

            return call_to_subarchitecture<ICE_FUNCTION_CLASS, ARGS...>(subarchitecture, std::forward<ARGS>(t)...);
        }
        
        /* (see \function{call_to_subarchitecture})*/
        template<typename ICE_FUNCTION_CLASS, typename ...ARGS>
        IceInternal::Handle<ICE_FUNCTION_CLASS> call(const ARGS&... t)
        {
            CAST__VERBOSER(17, "");
            
            Subarchitecture subarchitecture = getSubarchitectureID();

            return call_to_subarchitecture<ICE_FUNCTION_CLASS, ARGS...>(subarchitecture, t...);
        }
        
        
    
        std::shared_ptr<pthread_mutex_t> get_mutex(const Address& a_creation)
        {
            assert(is_my_creation(a_creation));
            return mutexes[a_creation.subarchitecture][a_creation.id];
        }


        procedure_call____IMPLEMENT_FROM__NOT_AN_ADDRESS(get_mutex, std::shared_ptr<pthread_mutex_t>, /**/);
        
        /* (see \member{get_mutex} -- i.e., non-const) */
        const std::shared_ptr<pthread_mutex_t> get_mutex(const Address& a_creation) const
        { 
            return const_cast<decltype(*this)>(this)->get_mutex();
        }

        procedure_call____IMPLEMENT_FROM__NOT_AN_ADDRESS(get_mutex, const std::shared_ptr<pthread_mutex_t>, const);
        

        void release_wrapper_for_cast(const cast::cdl::WorkingMemoryChange& in)
        {
            CAST__VERBOSER(201, "CAST-based call to RELEASE :: "
                           <<in.address.id<<" "<<in.address.subarchitecture<<std::endl);
            
            VERBOSER(501, "CAST-based call to RELEASE :: "
                     <<in.address.id<<" "<<in.address.subarchitecture<<std::endl);

//             exit(0);
            
            if(!release(in)){
                UNRECOVERABLE_ERROR("Failure during posix mutex release.");
            }
        }
  
        bool release(const Address& address) 
        {
            
            CAST__VERBOSER(15, "TRY :: entry release:: "<<address.id<<" "
                           <<" Subarchitecture :: "<<address.subarchitecture<<std::endl);
            LOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(15, "SUCCESS :: entry release Id:: "<<address.id<<" "
                           <<" Subarchitecture :: "<<address.subarchitecture<<std::endl);
            
            bool result = false;
            
            if(is_my_creation(address)){
            
                std::shared_ptr<pthread_mutex_t> mutex = get_mutex(address);

                
                CAST__VERBOSER(1, "Trying to unlock mutex..."<<std::endl);

                CAST__VERBOSER(15, "Unlocking :: "<<address.id<<" "<<address.subarchitecture<<" "<<mutex.get()<<std::endl);
                CAST__QUERY_UNRECOVERABLE_ERROR((0 != pthread_mutex_unlock(mutex.get())),
                                          "Failed to unlock mutex!!");
                
                CAST__VERBOSER(15, "Done unlocking :: "<<address.id<<" "<<address.subarchitecture<<" "<<mutex.get()<<std::endl);

                result = true;
            } else {
                CAST__UNRECOVERABLE_ERROR("Asked to release :: "
                                          <<address.id<<" "<<address.subarchitecture<<std::endl
                                          <<"that is either: (1) supposed to be dead..."<<std::endl
                                          <<"or otherwise (2) has yet to be created."<<std::endl);
            }
            


            
            CAST__VERBOSER(9, "TRY:: exiting release Id:: "<<address.id<<" "
                     <<" Subarchitecture :: "<<address.subarchitecture<<std::endl);
            UNLOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(9, "DONE :: exiting release Id:: "<<address.id<<" "
                           <<" Subarchitecture :: "<<address.subarchitecture<<std::endl);
            
            return result;
        }
        procedure_call____IMPLEMENT_FROM__NOT_AN_ADDRESS(release, bool, /**/);

        
        void kill_creation(const Address& my_creation)
        {
            /* ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK */
            /* ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK */
            /* ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK */
            /* ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK */
            /* ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK ** LOCK */
            CAST__VERBOSER(9, "TRY GET ::  "<<my_creation.id<<" "<<my_creation.subarchitecture<<std::endl);
            LOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(9, "DONE GET ::  "<<my_creation.id<<" "<<my_creation.subarchitecture<<std::endl);
            
            auto more_mutexes = mutexes.find(my_creation.subarchitecture);
            
            if(more_mutexes != mutexes.end()){

                CAST__VERBOSER(2, "Deleting a mutex from Subarchitecture :: "
                         << my_creation.subarchitecture<<std::endl);

                auto even_more_mutexes = more_mutexes->second.find(my_creation.id);
                
                if(even_more_mutexes != more_mutexes->second.end()){

                    

                    CAST__VERBOSER(15, "Destroying :: "<<even_more_mutexes->second.get()<<std::endl);
                    int i = pthread_mutex_destroy(even_more_mutexes->second.get());
                    
                    CAST__QUERY_UNRECOVERABLE_ERROR(i,
                                              "Unable to destroy mutex :: "<<even_more_mutexes->second.get()<<std::endl
                                              <<i<<std::endl
                                              <<EBUSY<<std::endl
                                              <<EDEADLK<<std::endl
                                              <<EINVAL<<std::endl
                                              <<EPERM<<std::endl);

                    auto object_Identifier
                        = Object_Identifier(my_creation.id,
                                            my_creation.subarchitecture);
                    
                    assert(objects_managed_by_cast.find(object_Identifier) !=
                           objects_managed_by_cast.end());

                    CAST__VERBOSER(4, "Getting the change receiver that I created for :: "
                                   <<my_creation.id<<" " 
                                   <<my_creation.subarchitecture<<std::endl);
                    
                    auto change_receiver = objects_managed_by_cast[object_Identifier];

                    CAST__VERBOSER(4, "Removing filter on :: "
                                   <<my_creation.id<<" " 
                                   <<my_creation.subarchitecture<<std::endl);
                    removeChangeFilter(change_receiver, cast::cdl::DELETERECEIVER);
                    
                    CAST__VERBOSER(4, "Deleting working memory related to :: "
                                   <<my_creation.id<<" " 
                                   <<my_creation.subarchitecture<<std::endl);
                    deleteFromWorkingMemory(my_creation.id,
                                            my_creation.subarchitecture);
                    
                    CAST__VERBOSER(4, "We are no longer managing the object at :: "
                                   <<my_creation.id<<" " 
                                   <<my_creation.subarchitecture<<std::endl);
                    objects_managed_by_cast.erase(object_Identifier);
                    
                    CAST__VERBOSER(4, "We are no longer managing the mutex at :: "
                                   <<my_creation.id<<" " 
                                   <<my_creation.subarchitecture<<std::endl);
                    more_mutexes->second.erase(my_creation.id);
                }
                
                if(more_mutexes->second.empty()){
                    mutexes.erase(my_creation.subarchitecture);
                }
            }
            
            /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
            /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
            /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
            /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
            /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
            CAST__VERBOSER(9, "TRY RELEASE ::  "<<my_creation.id<<" "<<my_creation.subarchitecture<<std::endl);
            UNLOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(9, "DONE RELEASE ::  "<<my_creation.id<<" "<<my_creation.subarchitecture<<std::endl);
        }
        
        procedure_call____IMPLEMENT_FROM__NOT_AN_ADDRESS(kill_creation, void, /**/);
        
        bool is_my_creation(const Address& a_creation) const
        {
            auto subarchitecture_mutexes = mutexes.find(a_creation.subarchitecture);
            if(subarchitecture_mutexes != mutexes.end()){
                auto relevant_mutexes = subarchitecture_mutexes->second.find(a_creation.id);

                if(relevant_mutexes != subarchitecture_mutexes->second.end()){
                    return true;
                }
            }
        
            return false;
        }
        
        procedure_call____IMPLEMENT_FROM__NOT_AN_ADDRESS(is_my_creation, bool, const);

    private:

        
        /* Claim to have created \argument{my_creation}.*/
        void register_creation(const Address& my_creation)
        {
            CAST__VERBOSER(2, "Registering creation... Id :: "<<my_creation.id
                     <<" "<<" Subarchitecture :: "<<my_creation.subarchitecture);
            
            if(mutexes.find(my_creation.subarchitecture) == mutexes.end()){
                CAST__VERBOSER(2, "As yet, no creations registered for Subarchitecture :: "
                         <<my_creation.subarchitecture);
                
                mutexes[my_creation.subarchitecture] =  map__subarchitecture_id__to__posix_mutex();
                CAST__VERBOSER(2, "Done registration...");
            }

            CAST__VERBOSER(2, "Attempting initialisation for creation... Id :: "<<my_creation.id
                     <<" "<<" Subarchitecture :: "<<my_creation.subarchitecture);
            

            std::shared_ptr<pthread_mutex_t> mutex(give_me_a_new__pthread_mutex_t());

            CAST__VERBOSER(15, "Initialising :: "<<mutex.get()<<" "<<my_creation.id
                           <<" "<<" Subarchitecture :: "<<my_creation.subarchitecture<<std::endl);

            // ** CHECK: \function{give_me_a_new__pthread_mutex_t} gives
            // ** and initialises the mutex.
//             QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_init(mutex.get(), NULL),
//                                       "Failed to initialise the mutex...");
            
            CAST__VERBOSER(15, "Done initialising :: "<<mutex.get()<<" "<<my_creation.id
                           <<" "<<" Subarchitecture :: "<<my_creation.subarchitecture<<std::endl);
            
//             if(0 != pthread_mutex_init(mutex.get(), NULL))
//             {UNRECOVERABLE_ERROR("Failed to initialise the mutex...");}
            
            CAST__VERBOSER(2, "Done initialising mutex...");
            
            
            CAST__VERBOSER(1, "Registration of mutex for creation... Id :: "<<my_creation.id
                     <<" "<<" Subarchitecture :: "<<my_creation.subarchitecture);
            mutexes[my_creation.subarchitecture][my_creation.id] = mutex;
            
            CAST__VERBOSER(2, "Done...");
        }
    
        procedure_call____IMPLEMENT_FROM__NOT_AN_ADDRESS(register_creation, void, /**/);
    
        
    protected:
        ADDRESS_RECOVERY recover_address;
        
    private:
        
        template<typename ICE_HANDLE_TYPE>
        ICE_HANDLE_TYPE _call_to_subarchitecture
        (const Subarchitecture& subarchitecture,
         ICE_HANDLE_TYPE& ice_handle)
        {
            
            CAST__VERBOSER(200, "TRY GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
            LOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(200, "SUCCESS GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
            
            VERBOSER(501, "Executing a call to subarchitecture :: "<<subarchitecture<<std::endl);
            
            CAST__VERBOSER(13, "Number of subarchitectures is :: "
                           <<mutexes.size()<<std::endl);
            if(mutexes.find(subarchitecture) != mutexes.end())
                CAST__VERBOSER(13, "Number of mutexes in this subarchitecture is ::"
                               <<mutexes[subarchitecture].size());
            
            typedef typename ICE_HANDLE_TYPE::element_type ICE_TYPE;
            
            auto id = newDataID();
//             auto subarchitecture = getSubarchitectureID();

            /* Ensure that there is a posix mutex element for \local{id, subarchitecture}.*/
            register_creation(id, subarchitecture);

            assert(is_my_creation(id, subarchitecture));
            CAST__VERBOSER(15, "MAKING A CALL TO LOCK :: "<<id<<":"<<subarchitecture<<std::endl;);
            
            CAST__VERBOSER(200, "TRY RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
            UNLOCK_ACCESS_TO_MEMBER_DATA; 
            CAST__VERBOSER(200, "DONE RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
                
            CAST__QUERY_UNRECOVERABLE_ERROR(!lock(id, subarchitecture),
                                            "During first posix mutex lock :: "<<id<<" "<<subarchitecture);
            
            
            CAST__VERBOSER(200, "TRY GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
            LOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(200, "SUCCESS GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
            
            VERBOSER(501, "LOCKED--1 :i: "<<id<<" :s: "<<subarchitecture<<" :m: "<<get_mutex(id, subarchitecture).get()<<std::endl;);
            
            /* Listen for a change to the written data... */
            THIS__FUNCTION__WMC__TO__VOID p_to_release
                = &procedure_call<ADDRESS_RECOVERY>
                ::release_wrapper_for_cast;
            
//             auto type_filter =  cast::createGlobalTypeFilter<ICE_TYPE>(cast::cdl::OVERWRITE);


            
#define procedure_call____IMPLEMENTATION___call____addChangeFilter
            {                                                           \
                auto filter                                             \
                    = cast::                                            \
                    createAddressFilter(id,                             \
                                        subarchitecture,                \
                                        cast::cdl::OVERWRITE);  \ 
                                                                    auto object_Identifier = \
                                                                    Object_Identifier(id, subarchitecture); \
                assert(objects_managed_by_cast.find(object_Identifier) == \
                       objects_managed_by_cast.end());                  \
                auto memberFunctionChangeReceiver =                     \
                    new cast::MemberFunctionChangeReceiver<procedure_call<ADDRESS_RECOVERY>> \
                    (this,                                              \
                     p_to_release);                                     \
                objects_managed_by_cast[object_Identifier]              \
                    = memberFunctionChangeReceiver;                     \
                addChangeFilter(filter,                                 \
                                memberFunctionChangeReceiver);          \
            }                                                           \
            
            
            CAST__VERBOSER(601, "GLOBAL :: Listening to changes on :: "<<id<<" "<<subarchitecture<<std::endl);
            
            procedure_call____IMPLEMENTATION___call____addChangeFilter;
                    
            
//             switch(locality){
//                 case Locality::Global:
                    
//                 {
//                     CAST__VERBOSER(12, "Procedure calls do not distinguish between local and global scope."<<std::endl
//                                    <<"You are using global.");
                    
//                     //UNRECOVERABLE_ERROR("Not expecting global...");
//                     //auto type_filter = cast::createGlobalTypeFilter<ICE_TYPE>(cast::cdl::OVERWRITE);

                
//                 }
                
                    
//                      break;
//                 case Locality::Local:
                    
//                 {
//                     CAST__VERBOSER(12, "Procedure calls do not distinguish between local and global scope."<<std::endl
//                                   <<"You are using local.");
// // //                     auto filter =  cast::createLocalTypeFilter<ICE_TYPE>(cast::cdl::OVERWRITE);
// //                     auto filter
// //                         = cast::createAddressFilter(id,
// //                                                     subarchitecture,
// //                                                     cast::cdl::OVERWRITE);
                    
//                         CAST__VERBOSER(601, "LOCAL :: Listening to changes on :: "<<id<<" "<<subarchitecture<<std::endl);
                        
//                     procedure_call____IMPLEMENTATION___call____addChangeFilter;  
//                 }
                
                    
//                      break;
//                 default:
//                     UNRECOVERABLE_ERROR("Component has unknown locality.");
//                     break;
//             }
            

            
            CAST__VERBOSER(200, "Added a change filter -- i.e., post the procedure call...");
            addToWorkingMemory( id, 
                                subarchitecture,
                                ice_handle);
            
            
            
            CAST__VERBOSER(13, "Added to working memory...");
            
            
            {  
                assert(is_my_creation(id, subarchitecture));
                CAST__VERBOSER(15, "MAKING A CALL TO LOCK(&& -- 2).....");



                assert(mutexes.find(subarchitecture) != mutexes.end());
                assert(mutexes.find(subarchitecture)->second.find(id)
                       != mutexes.find(subarchitecture)->second.end());
                CAST__VERBOSER(200, "Waiting for client response :: "<<mutexes[id][subarchitecture].get());
                

//                 overwriteWorkingMemory(id,
//                                        subarchitecture,
//                                        ice_handle);

                
                VERBOSER(501, "LOCKING--2 :i: "<<id
                         <<" :s: "<<subarchitecture
                         <<" :m: "<<get_mutex(id, subarchitecture).get()
                         <<std::endl;);
                
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                CAST__VERBOSER(200, "TRY RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
                UNLOCK_ACCESS_TO_MEMBER_DATA;/* HERE #*/
                CAST__VERBOSER(200, "DONE RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
                
                 /* Wait for a release.*/
                
                CAST__QUERY_UNRECOVERABLE_ERROR(
                    !lock(id, subarchitecture),
                    "During second posix mutex lock :: "<<id<<" "<<subarchitecture);
                
                CAST__VERBOSER(200, "TRY GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
                LOCK_ACCESS_TO_MEMBER_DATA;
                CAST__VERBOSER(200, "SUCCESS GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
                
                CAST__VERBOSER(200, "Got release from calling thread, and have now relocked...");
                
                assert(is_my_creation(id, subarchitecture));
            
                CAST__VERBOSER(15, "MAKING A CALL TO RELEASE(&& -- caller).....");
                
                CAST__VERBOSER(200, "TRY RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
                UNLOCK_ACCESS_TO_MEMBER_DATA;
                CAST__VERBOSER(200, "DONE RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
                
                CAST__QUERY_UNRECOVERABLE_ERROR(
                    !release(id, subarchitecture),
                    "During posix mutex release :: "<<id<<" "<<subarchitecture);
                
                CAST__VERBOSER(200, "TRY GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
                LOCK_ACCESS_TO_MEMBER_DATA;
                CAST__VERBOSER(200, "SUCCESS GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
                
                CAST__VERBOSER(1, "Received client response...");
            }
            
            CAST__VERBOSER(1, "Killing creation...");
            
            assert(is_my_creation(id, subarchitecture));

            auto result = getMemoryEntry<ICE_TYPE>(id, subarchitecture);
            
            /* Release all resources associated with \local{id, subarchitecture}.*/
            CAST__VERBOSER(200, "TRY RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
            UNLOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(200, "DONE RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
            kill_creation(id, subarchitecture);
            CAST__VERBOSER(200, "TRY GET ::  "<<procedure_call____MUTEX.get()<<std::endl);
            LOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(200, "SUCCESS GET ::  "<<procedure_call____MUTEX.get()<<std::endl);

            CAST__VERBOSER(1, "Completed procedure call...");
            
            CAST__VERBOSER(200, "TRY RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
            UNLOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(200, "DONE RELEASE ::  "<<procedure_call____MUTEX.get()<<std::endl);
            
            return result;
            
        }
        

        bool lock(const Address& address)
        {
            LOCK_ACCESS_TO_MEMBER_DATA;
            
            if(!is_my_creation(address)){
                CAST__WARNING("Asked to lock Id:: "<<address.id<<std::endl
                              <<" Subarchitecture :: "<<address.subarchitecture<<std::endl
                              <<". These had not been registered with a mutex on call to lock."<<std::endl
                              <<"Making registration now..."<<std::endl);
                register_creation(address);
            }
            
            UNLOCK_ACCESS_TO_MEMBER_DATA;
            return _lock(address);
        }

        procedure_call____IMPLEMENT_FROM__NOT_AN_ADDRESS(lock, bool, /**/);
        

        
        bool _lock(const Address& address)
        {
            LOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(1, "Locking on  Id ::"<<address.id<<" Subarchitecture :: "<<address.subarchitecture);
            
            std::shared_ptr<pthread_mutex_t> mutex = get_mutex(address);
            
            CAST__VERBOSER(15, "Locking :: "<<address.id<<" "<<address.subarchitecture<<" "<<mutex.get()<<std::endl);

            //int lock_result = pthread_mutex_lock(mutex.get());

            int lock_result = 0;
            
            UNLOCK_ACCESS_TO_MEMBER_DATA;
            CAST__QUERY_UNRECOVERABLE_ERROR(0 != (lock_result = pthread_mutex_lock(mutex.get())),
                                            "Unable to lock :: "<<mutex.get()<<" "<<lock_result<<std::endl);
            LOCK_ACCESS_TO_MEMBER_DATA;
            
            CAST__VERBOSER(15, "Done locking :: "<<address.id<<" "<<address.subarchitecture<<" "<<mutex.get()<<std::endl);
            
            UNLOCK_ACCESS_TO_MEMBER_DATA;
            
            return (0 == lock_result);//static_cast<bool>(lock_result);//(0 == pthread_mutex_lock(mutex.get()));
        }

    private:
        /* Any \class{procedure_call} member functions that access
         * local member data lock access to that data during reads and
         * writes. */
        std::shared_ptr<pthread_mutex_t> procedure_call____MUTEX;

        /* C++ type to manage the set of mutexes associated with
         * procedure calls. These are accessed first through a
         * \type{CAST_RPC::Subarchitecture} , and then through a
         * \type{CAST_RPC::Id} -- for the latter see
         * \class{map__subarchitecture__to____map__subarchitecture_id__to__posix_mutex}.*/
        typedef std::tr1::
        unordered_map<Id
                      , std::shared_ptr<pthread_mutex_t>> map__subarchitecture_id__to__posix_mutex;
        
        /* C++ type to manage the set of mutexes associated with
         * procedure calls (see related
         * \type{map__subarchitecture_id__to__posix_mutex} ).*/
        typedef std::tr1::
        unordered_map<Subarchitecture
                      , map__subarchitecture_id__to__posix_mutex>
        map__subarchitecture__to____map__subarchitecture_id__to__posix_mutex;

        /* Each \class{procedure_call} and \module{cast::*} object
         * that is managed by this has associated with it a
         * \standard{posix} mutex -- see \library{pthreads.h}. This
         * set of mutexes occurs in the following data
         * structure. Access is first through the subarchitecture to
         * which the procedure call was made, followed by the id of
         * the SLICE-based object that encapsulates that call.*/ 
        map__subarchitecture__to____map__subarchitecture_id__to__posix_mutex mutexes;

        /* Tuple gives data that identifies an object that is managed
         * by \module{cast::*}.*/
        typedef std::tr1::tuple<Id, Subarchitecture> Object_Identifier;

        /* When an object is passed to \module{cast::*} for
         * management, the caller usually has access to a reference to
         * the object in the form of an \class{Object_Identifier}
         * instance. When a caller wants to listen to changes made to
         * that object, a reference to that request comes in the form
         * of a pointer to a
         * \class{cast::WorkingMemoryChangeReceiver}. To stop
         * listening to an object, it is not sufficient to have only
         * the \class{Object_Identifier} information, but rather we
         * also need the
         * \class{cast::WorkingMemoryChangeReceiver}. The following
         * map associates an \objects{Object_Identifier} with its
         * corresponding \class{cast::WorkingMemoryChangeReceiver}.*/
        typedef std::tr1::
        unordered_map<Object_Identifier
                      , cast::WorkingMemoryChangeReceiver*
                      , boost_combine_based_tuple_hasher<Id, Subarchitecture> >
        map__Object_Identifier__to__WorkingMemoryChangeReceiver;

        map__Object_Identifier__to__WorkingMemoryChangeReceiver objects_managed_by_cast;
    };    
}



#endif
/*
 *
 * \begin{quote}
 *
 * Yeah, it breaks down like this:
 * it's legal to buy it, it's legal to
 * own it and, if you're the
 * proprietor of a hash bar, it's
 * legal to sell it.  It's legal to
 * carry it, which doesn't really
 * matter 'cause -- get a load of this
 * -- if the cops stop you, it's
 * illegal for this to search you.
 * Searching you is a right that the
 * cops in Amsterdam don't have.
 *
 * \end{quote}
 *
 * Quentin Tarantino and Roger Roberts Avery, 1993 draft screenplay
 * for the movie Pulp Fiction.
 *
 *
 */

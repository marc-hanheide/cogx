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
 * if you make a change to this and commit that change to SVN.
 *
 * ABOUT :: 
 *
 * Executively speaking, \namespace{CAST_SCAT} gives a wrapper of
 * useful syntactic sugar for "subarchitecture" interactions via
 * classical -- i.e., circa August 2009 -- CAST.
 * 
 * In more detail.. What is CAST_SCAT?! The author believes.. CAST
 * (see \module{cast::*}) is lovely.. Wow!  But they can't get their
 * head around all of it. So, they have got their head around a
 * (perhaps) tiny piece of it that they believe is all that is
 * required for the automated planning parts of CogX. What does the
 * author guarantee?.. that CAST_SCAT appears to work without falling
 * over itself.
 */

#ifndef CAST_SCAT_HH
#define CAST_SCAT_HH

#include "utilities.hh"
#include "debug.hh"
#include "stl__tuple_hash.hh"
#include "cast__pthread_demangle.hh"

#include <cast/architecture.hpp>

// #include <pthread.h> // This shouldn't be necessary if you have the
// -std=c++0x switch (include -lpthread for safety) activated during
// GCC compilation.

namespace CAST_SCAT
{

    /* A CAST object address is given by two items. One contains the
     * object's Id and the other contains the subarchitecture with
     * which it is associated.*/
    typedef decltype(cast::cdl::WorkingMemoryChange().address) Address;
    typedef decltype(cast::cdl::WorkingMemoryChange().address.id) Id;
    typedef decltype(cast::cdl::WorkingMemoryChange().address.id) Subarchitecture;
    typedef std::string Designator;
    typedef std::vector<Designator> Designators;

    /* The typing here is really really weak. Because the Id and the
     * Subarchitecture have the same underlying type, they can be
     * mixed without the programmer knowing about it until runtime.
     * But there is nothing sensible we can do at this
     * stage. Something along the lines of what I commented below is
     * the start of a useful direction. At the moment, all checking of
     * address, subarchitecture, and ID validity is going to have to
     * be at runtime (a.k.a. crashtime). */
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
    
    
    template<>/* FIX :: */
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



    
    /* The following (\class{_receive_call}) is a *utility* class
     * associated with \class{procedure_implementation}.
     *
     * When a \namespace{CAST_SCAT} \class{procedure_call} is made,
     * it's implementation will be invoked by
     * \member{operator()(...)}. A single "implementation" (i.e.,
     * \class{procedure_implementation}) can only invoke one procedure
     * at a time. This is enforced by the \argument{mutex}.
     *
     */
    template<typename THING, typename FUNCTION, typename ICE_FUNCTION_CLASS, class PARENT = NA>
    class _receive_call : public PARENT
    {
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
        
        void operator()(const cast::cdl::WorkingMemoryChange& _wmc)
        {
            VERBOSER(15, "Locking _receive_call mutex:: "
                     <<mutex.get()<<std::endl);
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_lock(mutex.get()),
                                              "Unable to lock mutex.");

            
            VERBOSER(15, " Procedure call for --  :: "
                     <<_wmc.address.id
                     <<" "<<_wmc.address.subarchitecture
                     <<std::endl);

            //if(managed_Component->intended_audience_test(std::string("")));
            
            auto argument
                = dynamic_cast<cast::ManagedComponent*>(managed_Component)
                ->getMemoryEntry<ICE_FUNCTION_CLASS>(_wmc.address);

//             auto nonsense = argument->optionalMemberDesignatorIsAnArgument
            
//             /* If \local{managed_Component} is not the intended
//              * audience of the working memory object, then the
//              * implementation is not invoked.*/
//             if(!managed_Component->intended_audience_test(argument->optionalMemberDesignatorIsAnArgument)){
//                 QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(mutex.get()),
//                                               "Unable to unlock mutex.");
//                 return;
//             }
            

            (managed_Component->*function)(argument);

            
            VERBOSER(15, " Overwriting working memory :: "
                     <<_wmc.address.id
                     <<" "<<_wmc.address.subarchitecture
                     <<std::endl);
            
            managed_Component
                ->overwriteWorkingMemory(_wmc.address.id,
                                         _wmc.address.subarchitecture,
                                         argument);

            VERBOSER(15, "Unlocking _receive_call mutex :: "
                     <<mutex.get()<<std::endl);
            
            
            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_unlock(mutex.get()),
                                      "Unable to unlock mutex.");
        }
    private:
        THING* managed_Component;
        FUNCTION function;
        std::shared_ptr<pthread_mutex_t> mutex;
    };
    
    /* A \module{cast::*}-based procedure call is either "made" (i.e.,
     * make a call) "implemented" (i.e., we implement the call)
     * locally or globally (see the relevant template parameters of
     * \class{procedure_implementation} and \class{procedure_call}
     * respectively).
     *
     * NOTE : I do not know the distinction between \module{cast::*}
     * "local" v.s. "global". So I wrote the code below to support
     * both options.. defaulting to "global".
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
     */
    enum class Locality
    {
        Global,
        Local
    };
    

    /* Under the scat-scheme, you either call or implement a
     * "procedure". Procedure implementation must be invoked during
     * execution of the \parent{cast::ManagedComponent} 
     * \method{start()}.
     *
     * Example of usage: Suppose you have a SLICE
     * \class{helloworld::Announcement}. Moreover, suppose this is a
     * scat-scheme class, and thus conceptually encapsulates a c-like
     * procedure call. To implement that call using a, let's call it
     * \function{HelloReader::makeAnnouncement()}, the following code
     * would appear in \child{your} \method{start()}:
     * 
     * implement<helloworld::Announcement>(&HelloReader::makeAnnouncement2);
     *
     */
    template<typename CHILD
             , Locality locality = Locality::Global>
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
        
        explicit procedure_implementation(const Designator& designator = "")
            :mutex(give_me_a_new__pthread_mutex_t())
        {
            if("" == designator)
                designators  = decltype(designators)();
            else
                designators  = {designator};
            
            CAST__VERBOSER(4, "Created new procedure implementation with mutex :: "<<mutex.get());
        }
        
        explicit procedure_implementation(Designator&& designator = "")
            :mutex(give_me_a_new__pthread_mutex_t())
        {
            if("" == designator)
                designators  = decltype(designators)();
            else
                designators  = {designator};
            
            CAST__VERBOSER(4, "Created new procedure implementation with mutex :: "<<mutex.get());
            
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
            if(designators.empty())return true;
            if(audiences.empty())return true;
            if(designators.end() != std::find(designators.begin(), designators.end(), "")) return true; 
            if(audiences.end() != std::find(audiences.begin(), audiences.end(), "")) return true; 
            
            decltype(audiences) result;
            std::set_intersection( audiences.begin(), audiences.end(),
                                   designators.begin(), designators.end(),
                                   inserter(result,result.begin()) );
            
            return !result.empty();
        }        

        /* Make this implement calls with a \argument{designator}
         * designation.*/
        void add_designator(const Designator& designator)
        {
            if("" == designator) return;
            
            designators.push_back(designator);
        }
        
        /* (see \member{add_designator()})*/
        void add_designator(Designator&& designator)
        {
            if("" == designator) return;
            
            designators.push_back(std::move(designator));
        }
        
        
        /* This _must only_ be called by a
         * \class{cast::ManagedComponent} \method{start()}. It is
         * _not_ thread safe... .*/
        template<typename ICE_FUNCTION_CLASS, typename FUNCTION_TYPE>
        void implement(FUNCTION_TYPE function)
        {

            typedef _receive_call<CHILD, FUNCTION_TYPE, ICE_FUNCTION_CLASS, can_be_deleted> __receive_call;
            
            to_be_deleted
                .push_back(std::shared_ptr<can_be_deleted>
                           (new __receive_call(dynamic_cast<CHILD*>(this),
                                               function,
                                               mutex)
                            )
                           );



            switch(locality){
                case Locality::Global:
                    
                {   UNRECOVERABLE_ERROR("Not expecting global...");
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
    
    
    template<typename ADDRESS_RECOVERY = _recover_address
             , Locality locality = Locality::Global>
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
        
        typedef procedure_call<ADDRESS_RECOVERY, locality> THIS__TYPE;
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
            ICE_FUNCTION_CLASS* functional = new ICE_FUNCTION_CLASS(std::forward<ARGS...>(t...));
            
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
            CAST__VERBOSER(15, "CAST-based call to RELEASE :: "
                           <<in.address.id<<" "<<in.address.subarchitecture<<std::endl);
            
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

            QUERY_UNRECOVERABLE_ERROR(0 != pthread_mutex_init(mutex.get(), NULL),
                                      "Failed to initialise the mutex...");
            
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
            CAST__VERBOSER(15, "TRY GET ::  "<<std::endl);
            LOCK_ACCESS_TO_MEMBER_DATA;
            CAST__VERBOSER(15, "SUCCESS GET ::  "<<std::endl);
            
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
            CAST__VERBOSER(15, "MAKING A CALL TO LOCK(&& -- 1).....");
            
            UNLOCK_ACCESS_TO_MEMBER_DATA; 
            CAST__QUERY_UNRECOVERABLE_ERROR(!lock(id, subarchitecture),
                                            "During first posix mutex lock :: "<<id<<" "<<subarchitecture);
            LOCK_ACCESS_TO_MEMBER_DATA;
            
            /* Listen for a change to the written data... */
            THIS__FUNCTION__WMC__TO__VOID p_to_release
                = &procedure_call<ADDRESS_RECOVERY, locality>
                ::release_wrapper_for_cast;
            
//             auto type_filter =  cast::createGlobalTypeFilter<ICE_TYPE>(cast::cdl::OVERWRITE);


            switch(locality){
                case Locality::Global:
                    
                {
                   UNRECOVERABLE_ERROR("Not expecting global...");
                    //auto type_filter = cast::createGlobalTypeFilter<ICE_TYPE>(cast::cdl::OVERWRITE);

                
#define procedure_call____IMPLEMENTATION___call____addChangeFilter      \
                    {                                                   \
                        auto filter                                     \
                            = cast::                                    \
                            createAddressFilter(id,                     \
                                                subarchitecture,        \
                                                cast::cdl::OVERWRITE);  \ 
                        auto object_Identifier =                        \
                            Object_Identifier(id, subarchitecture);     \
                        assert(objects_managed_by_cast.find(object_Identifier) == \
                               objects_managed_by_cast.end());          \
                        auto memberFunctionChangeReceiver =             \
                            new cast::MemberFunctionChangeReceiver<procedure_call<ADDRESS_RECOVERY, locality>> \
                            (this,                                      \
                             p_to_release);                             \
                        objects_managed_by_cast[object_Identifier]      \
                            = memberFunctionChangeReceiver;             \
                        addChangeFilter(filter,                         \
                                        memberFunctionChangeReceiver);  \
                    }                                                   \
                    
                    
                    procedure_call____IMPLEMENTATION___call____addChangeFilter;
                    
                }
                
                    
                     break;
                case Locality::Local:
                    
                {
// //                     auto filter =  cast::createLocalTypeFilter<ICE_TYPE>(cast::cdl::OVERWRITE);
//                     auto filter
//                         = cast::createAddressFilter(id,
//                                                     subarchitecture,
//                                                     cast::cdl::OVERWRITE);
                    
                    procedure_call____IMPLEMENTATION___call____addChangeFilter;  
                }
                
                    
                     break;
                default:
                    UNRECOVERABLE_ERROR("Component has unknown locality.");
                    break;
            }

            
            CAST__VERBOSER(13, "Added a change filter...");
            addToWorkingMemory( id, 
                                subarchitecture,
                                ice_handle);
            
            
            
            CAST__VERBOSER(13, "Added to working memory...");
            
            
            {  
                assert(is_my_creation(id, subarchitecture));
                CAST__VERBOSER(15, "MAKING A CALL TO LOCK(&& -- 2).....");



                
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                /* ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK ** UN--LOCK */
                CAST__VERBOSER(13, "TRY RELEASE ::  "<<std::endl);
                UNLOCK_ACCESS_TO_MEMBER_DATA;/* HERE #*/
                CAST__VERBOSER(13, "DONE RELEASE ::  "<<std::endl);
                
                 /* Wait for a release.*/
                CAST__VERBOSER(13, "Waiting for client response...");
                
                CAST__QUERY_UNRECOVERABLE_ERROR(
                    !lock(id, subarchitecture),
                    "During second posix mutex lock :: "<<id<<" "<<subarchitecture);
                
                LOCK_ACCESS_TO_MEMBER_DATA;
                
                CAST__VERBOSER(13, "Got release from calling thread, and have now relocked...");
                
                assert(is_my_creation(id, subarchitecture));
            
                CAST__VERBOSER(15, "MAKING A CALL TO RELEASE(&& -- caller).....");
                
                UNLOCK_ACCESS_TO_MEMBER_DATA;
                
                CAST__QUERY_UNRECOVERABLE_ERROR(
                    !release(id, subarchitecture),
                    "During posix mutex release :: "<<id<<" "<<subarchitecture);
                
                LOCK_ACCESS_TO_MEMBER_DATA;
                
                CAST__VERBOSER(1, "Received client response...");
            }
            
            CAST__VERBOSER(1, "Killing creation...");
            
            assert(is_my_creation(id, subarchitecture));

            auto result = getMemoryEntry<ICE_TYPE>(id, subarchitecture);
            
            /* Release all resources associated with \local{id, subarchitecture}.*/
            UNLOCK_ACCESS_TO_MEMBER_DATA;
            kill_creation(id, subarchitecture);
            LOCK_ACCESS_TO_MEMBER_DATA;

            CAST__VERBOSER(1, "Completed procedure call...");
            
            UNLOCK_ACCESS_TO_MEMBER_DATA;
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
         * \type{CAST_SCAT::Subarchitecture} , and then through a
         * \type{CAST_SCAT::Id} -- for the latter see
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

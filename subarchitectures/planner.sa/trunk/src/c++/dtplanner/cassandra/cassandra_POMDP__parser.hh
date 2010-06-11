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
 */

#ifndef CASSANDRA_POMDP__PARSER_HH
#define CASSANDRA_POMDP__PARSER_HH


#include "utilities.hh"

#include <pegtl.hh>

// #ifdef DEBUG_LEVEL
// #undef DEBUG_LEVEL
// #endif
// #define DEBUG_LEVEL 10

namespace POMDP
{
    namespace Parsing
    {   
        using namespace pegtl;
        using namespace pegtl::ascii;

        class Indexed_Strings;
        class Problem_Data;
        
        std::ostream& operator<<(std::ostream&, const POMDP::Parsing::Indexed_Strings&);
        class Indexed_Strings
        {
        public:
            friend std::ostream& operator<<(std::ostream&, const POMDP::Parsing::Indexed_Strings&);
    
            Indexed_Strings(const Indexed_Strings&);
            Indexed_Strings();
            
            typedef std::vector<std::string> Named_Numbers;
            typedef std::map<std::string, uint> Numbered_Names;

            typedef Named_Numbers::iterator iterator;
            typedef Named_Numbers::const_iterator const_iterator;
            
            inline uint size() const  {return named_Numbers.size();}
            
            bool valid(const std::string&) const;
            bool valid(uint) const;

            void push_back(const std::string&);
            
            uint get(const std::string&) const;
            const std::string& get(uint) const;

            const Named_Numbers& get() const;

            std::string operator[](uint) const;
            uint operator[](const std::string&) const;
            
            const_iterator begin() const {return named_Numbers.begin();}
            iterator begin()  {return named_Numbers.begin();}
            const_iterator end() const {return named_Numbers.end();}
            iterator end()  {return named_Numbers.end();}
        private:
            Named_Numbers named_Numbers;
            
            Numbered_Names numbered_Names;
            
            const std::string empty_string;// = "";
        };
        
        std::ostream& operator<<(std::ostream&, const POMDP::Parsing::Problem_Data&);
        
        class Problem_Data
        {
        private:
            
            /* Last sequence of numbers parsed*/
            Indexed_Strings actions;
            Indexed_Strings states;
            Indexed_Strings observations;
            
        public:
            /* Do we minimise cost, or maximise rewards? */
            enum Value_Type {Reward, Cost};
            
            Problem_Data()
                :already__called__initialise__reward_model(false),
                 already__called__initialise__transition_model(false),
                 already__called__initialise__observation_model(false)
            {}
            
            
            friend std::ostream& operator<<(std::ostream&, const POMDP::Parsing::Problem_Data&);

            /******************************************************************************************************
             ******************************************************************************************************
             * INTERFACE TO :: PLANNER INTERFACE IMPLEMENTATION
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            
            uint get__states_count() const;
            uint get__actions_count() const;
            uint get__observations_count() const;
            
            const decltype(states)& get__states() const;
            const decltype(actions)& get__actions() const;
            const decltype(observations)& get__observations() const;
            
            const decltype(states[0])& get__state(int) const;
            const decltype(actions[0])& get__action(int) const;
            const decltype(observations[0])& get__observation(int) const;
            
            double get__reward(const std::string& executed_action__name,
                               const std::string& starting_state__name,
                               const std::string& successor_state__name,
                               const std::string& observation__name) const;
            
            double get__transition_probability(const std::string& executed_action__name,
                                               const std::string& starting_state__name,
                                               const std::string& successor_state__name) const;
            
            double get__observation_probability(const std::string& executed_action__name,
                                                const std::string& successor_state__name,
                                                const std::string& observation__name) const;

            double get__reward(int executed_action__index,
                               int starting_state__index,
                               int successor_state__index,
                               int observation__index) const;
            
            double get__transition_probability(int executed_action__index,
                                               int starting_state__index,
                                               int successor_state__index) const;
            
            double get__observation_probability(int executed_action__index,
                                                int successor_state__index,
                                                int observation__index) const;

            double get__discount_factor() const;

            
            
            /******************************************************************************************************
             ******************************************************************************************************
             * SANITY OF MODELS
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/

            
            /* Is the \member{transition_model} sane? -- i.e., For
             * each state and action, the probability of transitions
             * to successors adds to 1.*/
            bool sanity__transition_model() const;
            bool sanity__transition_model__has_non_zero_entries() const;
            bool sanity__observation_model() const;
            bool sanity__observation_model__has_non_zero_entries() const;
            
            /******************************************************************************************************
             ******************************************************************************************************
             * ENDING ENDING ENDING INTERFACE TO ::  PLANNER INTERFACE IMPLEMENTATION
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            
            /* Floating point.*/
            void add__number(const std::string&);

            /* Positive integer.*/
            void add__natural_number(const std::string&);

            /******************************************************************************************************
             ******************************************************************************************************
             * INTERFACE TO :: DESCRIPTION OF REWARDS
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            
            void add__rewards____action__start_state__successor_state__observation(const std::string&);
            void add__rewards____action__start_state__successor_state(const std::string&);
            void add__rewards____action__start_state(const std::string&);

            
            /******************************************************************************************************
             ******************************************************************************************************
             * INTERFACE TO :: DESCRIPTION OF TRANSITIONS
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            

            void add__transitions____action__start_state__successor_state(const std::string& );
            void add__transitions____action__start_state(const std::string& );
            void add__transitions____action(const std::string& );

            /*Keyword descriptions of matrices for transition relations.*/
            void add__transitions____action__UNIFORM(const std::string&);
            void add__transitions____action__IDENTITY(const std::string&);
            void  add__transitions____action__start_state__UNIFORM(const std::string&);
            void  add__transitions____action__start_state__IDENTITY(const std::string&);
            void add__transitions____action__start_state__successor_state__UNIFORM(const std::string& );
            void add__transitions____action__start_state__successor_state__IDENTITY(const std::string& );


            
            /******************************************************************************************************
             ******************************************************************************************************
             * INTERFACE TO :: DESCRIPTION OF OBSERVATIONS
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            

            void add__observations____action__successor_state__observation(const std::string& );
            void add__observations____action__successor_state(const std::string& );
            void add__observations____action(const std::string& );
                

            void add__observations____action__successor_state__observation__UNIFORM(const std::string& );
            void add__observations____action__successor_state__UNIFORM(const std::string& );
            void add__observations____action__UNIFORM(const std::string& );
                

            void add__observations____action__successor_state__observation__IDENTITY(const std::string& );
            void add__observations____action__successor_state__IDENTITY(const std::string& );
            void add__observations____action__IDENTITY(const std::string& );
                

            /******************************************************************************************************
             ******************************************************************************************************
             * OTHER INTERFACE FEATURES (declaration and reporting of actions, states, observations, etc...)
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            void add__actions_given_number(const std::string&);
            void add__states_given_number(const std::string&);
            void add__observations_given_number(const std::string&);

            void add__start_state_distribution(const std::string&);
            
            void add__action(const std::string&);
            void add__state(const std::string&);
            void add__observation(const std::string&);
            
            void report__action(const std::string&);
            void report__starting_state(const std::string&);
            void report__successor_state(const std::string&);
            void report__observation(const std::string&);
            
            void report__action_as_number(const std::string&);
            void report__starting_state_as_number(const std::string&);
            void report__successor_state_as_number(const std::string&);
            void report__observation_as_number(const std::string&);
            
            void report__discount_factor(const std::string&);
            void report__value_Type_is_Reward(const std::string&);
            void report__value_Type_is_Cost(const std::string&);
            
            //             void add__rewards____action__start_state__successor_state__observation(const std::string&);
            //             void add__rewards____action__start_state__successor_state(const std::string&);
            //             void add__rewards____action__start_state(const std::string&);

            
        private:
            
            /******************************************************************************************************
             ******************************************************************************************************
             * INTERFACE TO :: DESCRIPTION OF REWARDS
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            
            void add__rewards____action__start_state__successor_state__observation_();
            void add__rewards____action__start_state__successor_state___observation();
            void add__rewards____action__start_state___successor_state__observation();
            void add__rewards____action___start_state__successor_state__observation();
            void add__rewards____action__start_state__successor_state_NUMBERS();
            void add__rewards____action__start_state__successor_state_();
            void add__rewards____action__start_state___successor_state();
            void add__rewards____action___start_state__successor_state();
            void add__rewards____action__start_state_NUMBERS();
            void add__rewards____action__start_state_();
            void add__rewards____action___start_state();

            /******************************************************************************************************
             ******************************************************************************************************
             * INTERFACE TO :: DESCRIPTION OF TRANSITIONS
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            

            void add__transitions____action__start_state__successor_state_();
            void add__transitions____action__start_state___successor_state();
            void add__transitions____action___start_state__successor_state();
            void add__transitions____action__start_state_NUMBERS();
            void add__transitions____action__start_state_();
            void add__transitions____action___start_state();
            void add__transitions____action_NUMBERS();
            void add__transitions____action_();

            /******************************************************************************************************
             ******************************************************************************************************
             * INTERFACE TO :: DESCRIPTION OF OBSERVATIONS
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/
            
            void add__observations____action__successor_state__observation_();
            void add__observations____action__successor_state___observation();
            void add__observations____action___successor_state__observation();
            void add__observations____action__successor_state_NUMBERS();
            void add__observations____action__successor_state_();
            void add__observations____action___successor_state();
            void add__observations____action_NUMBERS();
            void add__observations____action_();

            /******************************************************************************************************
             ******************************************************************************************************
             * OTHER DETAILS -- OTHER DETAILS -- OTHER DETAILS -- OTHER DETAILS -- OTHER DETAILS -- OTHER DETAILS -- 
             ******************************************************************************************************
             ******************************************************************************************************
             ******************************************************************************************************/

            /*Is this a "reward" or "cost" POMDP?*/
            Value_Type value_Type;
            
            /* ONCE :: This member functions actions on
             * \member{reward_model} IFF it is invoked for the first
             * time. That it has already been called is tracked by
             * \member{already__called__initialise__reward_model}.*/
            void initialise__reward_model();
            bool already__called__initialise__reward_model;
            
            /* ONCE :: This member functions actions on
             * \member{transition_model} IFF it is invoked for the
             * first time. That it has already been called is tracked
             * by
             * \member{already__called__initialise__transition_model}.*/
            void initialise__transition_model();
            bool already__called__initialise__transition_model;
            
            /* ONCE :: This member functions actions on
             * \member{observation_model} IFF it is invoked for the
             * first time. That it has already been called is tracked
             * by
             * \member{already__called__initialise__observation_model}.*/
            void initialise__observation_model();
            bool already__called__initialise__observation_model;
            
            /* Value of the last parsed string symbol of the type indicated by the variable name*/
            std::string action;
            std::string start_state;
            std::string successor_state;
            std::string observation;
            
            /* Last number parsed*/
            double number;

            /* Last natural number (positive integer) parsed*/
            uint natural_number;
            
            /* Discount factor reported by \member{report__discount_factor}.*/
            double discount_factor;

            /* Last sequence of numbers parsed*/
            std::vector<double> numbers;
            /*Shared index into elements of \member{numbers}.*/
            uint numbers_index;// = 0;

            
            /* action -> state (starting) -> state (successor) -> observation -> reward*/
            std::map<
                std::string,
                std::map<
                std::string,
                std::map<
                std::string,
                std::map<
                std::string,
                double>>>> reward_model;

            /* action -> state (starting) -> state (successor) -> probability */
            std::map<
                std::string,
                std::map<
                std::string,
                std::map<
                std::string,
                double>>> transition_model;
            
            /* action -> state (you end up in "successor") -> observation -> probability */
            std::map<
                std::string,
                std::map<
                std::string,
                std::map<
                std::string,
                double>>> observation_model;

            /* Each element in this vector determines the probability
             * that we are in a particular state at "time-zero".*/
            std::vector<double> start_state_distribution;
        };
        
        std::ostream& operator<<(std::ostream& o
                                 , const POMDP::Parsing::Problem_Data::Value_Type& value_Type);
        
        struct DEBUG__Action : action_base< DEBUG__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                UNRECOVERABLE_ERROR("DEBUG RULE APPLIED.");
            }
        };
        
        struct Number__Action : action_base< Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Natural_Number__Action : action_base< Natural_Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__natural_number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Actions_Given_Number__Action : action_base< Add_Actions_Given_Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__actions_given_number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_States_Given_Number__Action : action_base< Add_States_Given_Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__states_given_number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Observations_Given_Number__Action : action_base< Add_Observations_Given_Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__observations_given_number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_State__Action : action_base< Add_State__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__state(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Action__Action : action_base< Add_Action__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__action(str);
                VERBOSER(1, str);
            }
        };
        
        
        struct Add_Observation__Action : action_base< Add_Observation__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__observation(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Starting_State__Action : action_base< Report_Starting_State__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__starting_state(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Successor_State__Action : action_base< Report_Successor_State__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__successor_state(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Action__Action : action_base< Report_Action__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__action(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Observation__Action : action_base< Report_Observation__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__observation(str);
                VERBOSER(1, str);
            }
        };


        struct Report_Starting_State_As_Number__Action : action_base< Report_Starting_State_As_Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__starting_state_as_number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Successor_State_As_Number__Action : action_base< Report_Successor_State_As_Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__successor_state_as_number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Action_As_Number__Action : action_base< Report_Action_As_Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__action_as_number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Observation_As_Number__Action : action_base< Report_Observation_As_Number__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__observation_as_number(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Discount_Factor__Action : action_base< Report_Discount_Factor__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__discount_factor(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Value_Type_Cost__Action : action_base< Report_Value_Type_Cost__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__value_Type_is_Cost(str);
                VERBOSER(1, str);
            }
        };
        
        struct Report_Value_Type_Reward__Action : action_base< Report_Value_Type_Reward__Action>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->report__value_Type_is_Reward(str);
                VERBOSER(1, str);
            }
        };
        
        /******************************************************************************************************
         ******************************************************************************************************
         * INTERFACE TO :: DESCRIPTION OF REWARDS
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/
            
        struct Add_Rewards__Action____action__start_state__successor_state__observation
            : action_base< Add_Rewards__Action____action__start_state__successor_state__observation>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                VERBOSER(1, " **************** Application of :: Add_Rewards__Action____action__start_state__successor_state__observation");
                
                s->add__rewards____action__start_state__successor_state__observation(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Rewards__Action____action__start_state__successor_state
            : action_base< Add_Rewards__Action____action__start_state__successor_state>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__rewards____action__start_state__successor_state(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Rewards__Action____action__start_state
            : action_base< Add_Rewards__Action____action__start_state>
        { 
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__rewards____action__start_state(str);
                VERBOSER(1, str);
            }
        };


//         struct Add_Rewards__Action____action__start_state__successor_state__observation__IDENTITY
//             : action_base< Add_Rewards__Action____action__start_state__successor_state__observation__IDENTITY>
//         { 
//             template<typename stack_type>
//             static void apply(const std::string& str, stack_type& s)
//             {
//                 VERBOSER(1, " **************** Application of :: Add_Rewards__Action____action__start_state__successor_state__observation");
                
//                 s->add__rewards____action__start_state__successor_state__observation__IDENTITY(str);
//                 VERBOSER(1, str);
//             }
//         };
        
//         struct Add_Rewards__Action____action__start_state__successor_state__IDENTITY
//             : action_base< Add_Rewards__Action____action__start_state__successor_state__IDENTITY>
//         { 
//             template<typename stack_type>
//             static void apply(const std::string& str, stack_type& s)
//             {
//                 s->add__rewards____action__start_state__successor_state__IDENTITY(str);
//                 VERBOSER(1, str);
//             }
//         };
        
//         struct Add_Rewards__Action____action__start_state__IDENTITY
//             : action_base< Add_Rewards__Action____action__start_state__IDENTITY>
//         { 
//             template<typename stack_type>
//             static void apply(const std::string& str, stack_type& s)
//             {
//                 s->add__rewards____action__start_state__IDENTITY(str);
//                 VERBOSER(1, str);
//             }
//         };
        

//         struct Add_Rewards__Action____action__start_state__successor_state__observation__UNIFORM
//             : action_base< Add_Rewards__Action____action__start_state__successor_state__observation__UNIFORM>
//         { 
//             template<typename stack_type>
//             static void apply(const std::string& str, stack_type& s)
//             {
//                 VERBOSER(1, " **************** Application of :: Add_Rewards__Action____action__start_state__successor_state__observation");
                
//                 s->add__rewards____action__start_state__successor_state__observation__UNIFORM(str);
//                 VERBOSER(1, str);
//             }
//         };
        
//         struct Add_Rewards__Action____action__start_state__successor_state__UNIFORM
//             : action_base< Add_Rewards__Action____action__start_state__successor_state__UNIFORM>
//         { 
//             template<typename stack_type>
//             static void apply(const std::string& str, stack_type& s)
//             {
//                 s->add__rewards____action__start_state__successor_state__UNIFORM(str);
//                 VERBOSER(1, str);
//             }
//         };
        
//         struct Add_Rewards__Action____action__start_state__UNIFORM
//             : action_base< Add_Rewards__Action____action__start_state__UNIFORM>
//         { 
//             template<typename stack_type>
//             static void apply(const std::string& str, stack_type& s)
//             {
//                 s->add__rewards____action__start_state__UNIFORM(str);
//                 VERBOSER(1, str);
//             }
//         };
        

        
        /******************************************************************************************************
         ******************************************************************************************************
         * INTERFACE TO :: DESCRIPTION OF TRANSITIONS
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/   
        
        struct Add_Transitions__Action____action__start_state__successor_state
            : action_base<Add_Transitions__Action____action__start_state__successor_state>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action__start_state__successor_state(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Transitions__Action____action__start_state
            : action_base<Add_Transitions__Action____action__start_state>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action__start_state(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Transitions__Action____action
            : action_base<Add_Transitions__Action____action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action(str);
                VERBOSER(1, str);
            }
        };



        struct Add_Transitions__Action____action__start_state__successor_state__UNIFORM
            : action_base<Add_Transitions__Action____action__start_state__successor_state__UNIFORM>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action__start_state__successor_state__UNIFORM(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Transitions__Action____action__start_state__UNIFORM
            : action_base<Add_Transitions__Action____action__start_state__UNIFORM>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action__start_state__UNIFORM(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Transitions__Action____action__UNIFORM
            : action_base<Add_Transitions__Action____action__UNIFORM>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action__UNIFORM(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Transitions__Action____action__start_state__successor_state__IDENTITY
            : action_base<Add_Transitions__Action____action__start_state__successor_state__IDENTITY>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action__start_state__successor_state__IDENTITY(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Transitions__Action____action__start_state__IDENTITY
            : action_base<Add_Transitions__Action____action__start_state__IDENTITY>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action__start_state__IDENTITY(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Transitions__Action____action__IDENTITY
            : action_base<Add_Transitions__Action____action__IDENTITY>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__transitions____action__IDENTITY(str);
                VERBOSER(1, str);
            }
        };
        
        /******************************************************************************************************
         ******************************************************************************************************
         * INTERFACE TO :: DESCRIPTION OF OBSERVATIONS
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/
        
        struct Add_Observations__Action____action__successor_state__observation
            : action_base<Add_Observations__Action____action__successor_state__observation>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
//                 UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                
                s->add__observations____action__successor_state__observation(str);
                VERBOSER(1, str);
            }
        };

        struct Add_Observations__Action____action__successor_state
            : action_base<Add_Observations__Action____action__successor_state>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                //UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                s->add__observations____action__successor_state(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Observations__Action____action
            : action_base<Add_Observations__Action____action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                //UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                s->add__observations____action(str);
                VERBOSER(1, str);
            }
        };




        struct Add_Observations__Action____action__successor_state__observation__UNIFORM
            : action_base<Add_Observations__Action____action__successor_state__observation__UNIFORM>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                //UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                s->add__observations____action__successor_state__observation__UNIFORM(str);
                VERBOSER(1, str);
            }
        };

        struct Add_Observations__Action____action__successor_state__UNIFORM
            : action_base<Add_Observations__Action____action__successor_state__UNIFORM>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                //UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                s->add__observations____action__successor_state__UNIFORM(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Observations__Action____action__UNIFORM
            : action_base<Add_Observations__Action____action__UNIFORM>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                //UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                s->add__observations____action__UNIFORM(str);
                VERBOSER(1, str);
            }
        };


        struct Add_Observations__Action____action__successor_state__observation__IDENTITY
            : action_base<Add_Observations__Action____action__successor_state__observation__IDENTITY>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                //UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                s->add__observations____action__successor_state__observation__IDENTITY(str);
                VERBOSER(1, str);
            }
        };

        struct Add_Observations__Action____action__successor_state__IDENTITY
            : action_base<Add_Observations__Action____action__successor_state__IDENTITY>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                //UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                s->add__observations____action__successor_state__IDENTITY(str);
                VERBOSER(1, str);
            }
        };
        
        struct Add_Observations__Action____action__IDENTITY
            : action_base<Add_Observations__Action____action__IDENTITY>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                //UNRECOVERABLE_ERROR("ADDING OBSERVATIONS...");
                s->add__observations____action__IDENTITY(str);
                VERBOSER(1, str);
            }
        };
        
        /******************************************************************************************************
         ******************************************************************************************************
         * ________ENDING________ENDING________ INTERFACE TO :: DESCRIPTION OF OBSERVATIONS
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/
        

        
        

        
        
        struct Add_Start_State_Distribution__Action
            : action_base<Add_Start_State_Distribution__Action>
        {
            template<typename stack_type>
            static void apply(const std::string& str, stack_type& s)
            {
                s->add__start_state_distribution(str);
                VERBOSER(1, str);
            }
        };

        
        struct s_discount : string< d, i, s, c, o, u, n, t > {};
        struct s_start : string< s, t, a, r, t > {};
        struct s_values : string<v, a, l, u, e, s> {};
        struct s_reward : string<r, e, w, a, r, d> {};/*type of "value"*/
        struct s_cost : string<c,o,s,t> {};/*type of "value"*/
        
        struct s_states : string<s, t, a, t, e, s> {};
        struct s_actions : string<a,c,t,i,o,n,s> {};
        struct s_observations : string<o,b,s,e,r,v,a,t,i,o,n,s> {};


        
        struct s_identity : string<i,d,e,n,t,i,t,y> {};
        struct s_uniform : string<u,n,i,f,o,r,m> {};

        struct s_include : string<i,n,c,l,u,d,e> {};
        
        /* "first-state"*/
        struct s_first_state : seq<string<f,i,r,s,t>,one<'-'>,string<s,t,a,t,e> > {};

        struct keyword : sor<s_uniform
                             , s_identity
                             , s_observations
                             , s_actions
                             , s_states
                             , s_cost
                             , s_reward
                             , s_values
                             , s_start
                             , s_discount
                             , seq<one<'T'>, pad<one<':'>, space> > 
                             , seq<one<'R'>, pad<one<':'>, space> > 
                             , seq<one<'O'>, pad<one<':'>, space> >  > {};

        /* TODO -- Make sure the user can comment... */
        struct comment : seq<one<'#'>, until<eol>> {};
        
        struct simple_space : one<' '>{};
        
        struct basic_alphanumeric : seq< alpha
                                         , star < sor< alpha
                                                       , digit > > >{};

        struct basic_name : seq<basic_alphanumeric
                                , star< sor<basic_alphanumeric
                                            , one<'-'> > > > {};
        
        struct digits : plus<digit> {};
        
        struct n_double : ifapply< seq<opt< one< '+', '-' > >, digits, opt<one<'.'>, digits> >/*_seq*/,
                                   Number__Action>/*_ifapply*/  {};
        
        struct natural_number : ifapply <digits, Natural_Number__Action> {};
        
        struct state_name : sor<one<'*'>, basic_name> {}; // basic_name --> basic_alphanumeric
        struct action_name : sor<one<'*'>, basic_name> {}; // basic_name --> basic_alphanumeric
        struct observation_name : sor<one<'*'>, basic_name> {}; // basic_name --> basic_alphanumeric
        
        struct action_declaration :
            pad<sor< ifapply<natural_number, Add_Actions_Given_Number__Action>
                    , ifapply<action_name, Add_Action__Action> >
                    , simple_space > {};
        
        struct state_declaration :
            pad<sor< ifapply<natural_number, Add_States_Given_Number__Action>
                     , ifapply<state_name, Add_State__Action>>
                , simple_space >  {};
        
        struct observation_declaration :
            pad<sor< ifapply<natural_number, Add_Observations_Given_Number__Action>
                     , ifapply<observation_name, Add_Observation__Action> >
                , simple_space > {};
        
        struct actions_declaration : seq<s_actions, pad<one<':'>, space>, plus<action_declaration > >{};
        struct states_declaration : seq<s_states, pad<one<':'>, space>, plus<state_declaration > >{};
        struct observations_declaration : seq<s_observations, pad<one<':'>, space>, plus<observation_declaration > >{};
        

        /******************************************************************************************************
         ******************************************************************************************************
         * States (starting and successor) can be reported as strings or numbers. 
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/

        template<typename Number_Reporting__Action, typename String_Reporting__Action>
        struct action_name_or_number_indication :
            sor<ifapply<natural_number, Number_Reporting__Action>
                , ifapply<action_name,  String_Reporting__Action> >{};
        
        template<typename Number_Reporting__Action, typename String_Reporting__Action>
        struct state_name_or_number_indication :
            sor<ifapply<natural_number, Number_Reporting__Action>
                , ifapply<state_name,  String_Reporting__Action> >{};
        
        template<typename Number_Reporting__Action, typename String_Reporting__Action>
        struct observation_name_or_number_indication :
            sor<ifapply<natural_number, Number_Reporting__Action>
                , ifapply<observation_name,  String_Reporting__Action> >{};
        
        /******************************************************************************************************
         ******************************************************************************************************
         * DESCRIPTION OF REWARDS
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/
        
        struct reward_prefix : seq<string<R>, pad<one<':'>, space> > {};

        struct reward_description__type1 :
            ifapply< plus<pad<n_double, space> >
                     , Add_Rewards__Action____action__start_state >{};
        
        struct reward_description__type2 :
            ifapply< plus<pad<n_double, space> >
                     , Add_Rewards__Action____action__start_state__successor_state >{};

        struct reward_description__type3 :
            ifapply<
            seq<pad<one<':'>, space>
                , observation_name_or_number_indication<Report_Observation_As_Number__Action
                                                        , Report_Observation__Action> /*Observation received*/
                , pad<n_double, space> >
            , Add_Rewards__Action____action__start_state__successor_state__observation > {};
        
        struct reward_description__type2_or_type3 :
            seq<pad<one<':'>, space>
                , state_name_or_number_indication< Report_Successor_State_As_Number__Action
                                                   , Report_Successor_State__Action> /*Ending state*/
                , sor<reward_description__type2, reward_description__type3> > {};
        
        struct reward_description :
            seq<reward_prefix
                , action_name_or_number_indication<Report_Action_As_Number__Action
                                                   , Report_Action__Action> /*Executed action*/
                , pad<one<':'>, space>
                , state_name_or_number_indication<Report_Starting_State_As_Number__Action
                                                  , Report_Starting_State__Action> /*Starting state*/
                , sor<reward_description__type1, reward_description__type2_or_type3> >{};

        
        /******************************************************************************************************
         ******************************************************************************************************
         * DESCRIPTION OF ACTIONS
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/

        struct action_description__type1 :
            sor< ifapply<pad<s_identity, space>,
                         Add_Transitions__Action____action__IDENTITY>
                 , ifapply<pad<s_uniform, space>,
                           Add_Transitions__Action____action__UNIFORM>
                 , ifapply<plus<pad<n_double, space> >,
                           Add_Transitions__Action____action>
            > {};
        
        struct action_description__type2 :
            sor< ifapply<pad<s_identity, space>,
                         Add_Transitions__Action____action__start_state__IDENTITY>
                 , ifapply<pad<s_uniform, space>,
                           Add_Transitions__Action____action__start_state__UNIFORM>
                 , ifapply<plus<pad<n_double, space> >,
                           Add_Transitions__Action____action__start_state>
            > {};

        struct action_description__type3 :
            seq<pad<one<':'>, space>
                , state_name_or_number_indication<Report_Successor_State_As_Number__Action
                                                  , Report_Successor_State__Action> /*Successor state*/
                , sor< ifapply<pad<s_identity, space>,
                               Add_Transitions__Action____action__start_state__successor_state__IDENTITY>
                       , ifapply<pad<s_uniform, space>,
                                 Add_Transitions__Action____action__start_state__successor_state__UNIFORM>
                       , ifapply<pad<n_double, space>,
                                 Add_Transitions__Action____action__start_state__successor_state> > 
            > {};
        
        struct action_description__type2_or_type3 :
            seq<pad<one<':'>, space>
                , state_name_or_number_indication<Report_Starting_State_As_Number__Action
                                                  , Report_Starting_State__Action> /*Starting state*/
                , sor<action_description__type2, action_description__type3> >{};
        
        struct action_prefix : seq<string<T>, pad<one<':'>, space> > {};
        
        struct action_description :
            seq<action_prefix
                , action_name_or_number_indication<Report_Action_As_Number__Action
                                                   , Report_Action__Action>//ifapply<action_name,Report_Action__Action>
                , sor<action_description__type1, action_description__type2_or_type3> > {};
        
        /******************************************************************************************************
         ******************************************************************************************************
         * DESCRIPTION OF OBSERVATIONS
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/

        struct observation_description__type1 :
            sor< ifapply<pad<s_uniform, space>,
                         Add_Observations__Action____action__UNIFORM>
                 , ifapply<pad<s_identity, space>,
                           Add_Observations__Action____action__IDENTITY>
                 , ifapply<plus<pad<n_double, space> >,
                           Add_Observations__Action____action>
            > {};

        struct observation_description__type2 :
            sor< ifapply< pad<s_uniform, space>,
                          Add_Observations__Action____action__successor_state__UNIFORM>
                 , ifapply<pad<s_identity, space> ,
                           Add_Observations__Action____action__successor_state__IDENTITY>
                 , ifapply<plus<pad<n_double, space> >,
                           Add_Observations__Action____action__successor_state>
            > {};

        struct observation_description__type3 :
            seq<pad<one<':'>, space>
                , observation_name_or_number_indication<Report_Observation_As_Number__Action
                                                        , Report_Observation__Action>
                , sor< ifapply<pad<s_uniform, space>,
                               Add_Observations__Action____action__successor_state__observation__UNIFORM>
                       , ifapply<pad<s_identity, space>,
                                 Add_Observations__Action____action__successor_state__observation__IDENTITY>
                       , ifapply<pad<n_double, space>,
                                 Add_Observations__Action____action__successor_state__observation> > 
            > {};
        
        struct observation_description__type2_or_type3 :
            seq<pad<one<':'>, space>
                , state_name_or_number_indication<Report_Successor_State_As_Number__Action
                                                  , Report_Successor_State__Action> /*Starting state*/
                , sor<observation_description__type2, observation_description__type3> >{};
        
        struct observation_prefix : seq<string<O>, pad<one<':'>, space> > {};
        
        struct observation_description :
            seq<observation_prefix
                , action_name_or_number_indication<Report_Action_As_Number__Action
                                                   , Report_Action__Action>
                , sor<observation_description__type1, observation_description__type2_or_type3> > {};
        
        
        
        /******************************************************************************************************
         ******************************************************************************************************
         * DESCRIPTION OF STARTING-STATE
         ******************************************************************************************************
         ******************************************************************************************************
         ******************************************************************************************************/
        
        struct starting_state_distribution
            : ifapply<
            seq<s_start
                , pad<one<':'>, space>
                , plus< seq<n_double, star<simple_space> > > >
            , Add_Start_State_Distribution__Action > {};
            
        struct discount_factor :
            ifapply<
            seq<s_discount, pad<one<':'>, space>
                , n_double
                , star<simple_space> >
            , Report_Discount_Factor__Action/*ifapply -- END*/> {};

        struct value_type :
            sor<ifapply<s_reward, Report_Value_Type_Reward__Action>
                , ifapply<s_cost, Report_Value_Type_Cost__Action>
            > {};
        
        struct values_declaration :
            seq<s_values, pad<one<':'>, space>, value_type, star<simple_space> > {};

        struct actions_states_observations_declaration
            : sor<actions_declaration //one<'\n'> >
                  , states_declaration //one<'\n'> >
                  , observations_declaration >{};//one<'\n'> > >{};

        struct problem_prefix_element :
            sor<actions_states_observations_declaration
                , discount_factor
                , values_declaration
                , starting_state_distribution
                , comment
                >
        {};
        
        struct problem :
            seq<plus< seq<star<space>, problem_prefix_element, until<eol> > >
                , star<space>
                , star< sor<reward_description
                            , action_description
                            , observation_description
                            , comment> > >{};
        
//         struct problem : seq<star<space>, discount_factor, values_declaration, states_declaration, actions_declaration, observations_declaration>{};
        //star<space>, star<problem_prefix_element> > {};
        //                   , starting_state_distribution >{};
        
//             seq< star<problem_prefix_element>
//                  , star< sor<reward_description
//                              , action_description
//                              , observation_description > > > {};
        
        /*Problem as parsed.*/
        extern std::tr1::shared_ptr<Problem_Data> problem_Data;

        void parse_Cassandra_POMDP_problem(
            const std::string& problem_file_name,
            std::tr1::shared_ptr<Problem_Data>& = problem_Data);

    }
}


#endif

/*
 * Anekdot : Is the Russian word for a political joke.
 *
 * -- Seth Benedict Graham, A Cultural Analysis of the Russo-Soviet
 *    Anekdot, 2003.
 */

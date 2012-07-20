/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
 *
 * Authorship of this source code was supported by EC FP7-IST grant
 * 215181-CogX.
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
 * CogX ::
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


#ifndef SOLVER_HH
#define SOLVER_HH

#include "solver_basics.hh"
#include "state_basics.hh"
#include "dtp_pddl_parsing_data_constants.hh"


#include "planning_state.hh"
#include "partially_observable_markov_decision_process_state.hh"


namespace Planning
{
    class Solver
    {
    public:
        virtual ~Solver();//{};
        void cleanup();
        
        /*Penalty for ending up in an illegal state.*/
        double get__sink_state_penalty() const;
        /*Penalty for ending up in an illegal state.*/
        void set__sink_state_penalty(double);
        
        /*Testing conditions on doubles.*/
        static Are_Doubles_Close are_Doubles_Close;//(1e-9);

        /* When the solver interacts by taking perceptions and
         * prescribing actions, these are the relevant
         * data-structures for the receipt of perceptions.*/
        typedef std::vector<std::string> Precept_Arguments;
        typedef std::pair<std::string, Precept_Arguments>  Precept;
        typedef std::vector<std::pair<std::string, Precept_Arguments> >  Percept_List;
        
        typedef std::vector<std::string> Proposition_Arguments;
        typedef std::pair<std::string, Proposition_Arguments>  Proposition;
        typedef std::vector<std::pair<std::string, Proposition_Arguments> >  Proposition_List;

        
        std::vector<double> report__probabilities_of_facts(POMDP_State* current_state,
                                                           const Percept_List& perceptions);
        
        POMDP_State* take_observation(POMDP_State* current_state,
                                      Observational_State* observation,
                                      uint action_index);
        
        POMDP_State* take_observation(POMDP_State*, const Percept_List&, uint action_index);
        Observational_State* find_observation(Observational_State* new_observation);
        POMDP_State* compute_successor(Observational_State* observation,
                                 uint action_index,
                                 POMDP_State* current_state);
        
//         void expand_belief_state(State* current_state);

        std::pair<Planning::Formula::Action_Proposition, uint>
        get_prescribed_action(State* current_state);
        std::pair<Planning::Formula::Action_Proposition, uint>
        get_prescribed_action(POMDP_State* current_state);
        

        virtual void report__new_belief_state(POMDP_State* );
        virtual POMDP_State* obtain__next_belief_state_for_expansion();
        virtual POMDP_State* peek__next_belief_state_for_expansion();
        
        
        
        /*Problem that is the target of the solution procedure.*/
        Solver(Planning::Parsing::Problem_Data&, double sink_state_penalty = 0.0);//-1e6);

        bool operator()(){return true;};
        
        void expand_belief_state( POMDP_State*);
        bool expand_belief_state_space();
        /* \argument{action_index} does not indicate a legal action at
         * \argument{successor_state}.*/
        void fill_illegals(POMDP_State::Normalisation_Factors& _normalisation_Factors
                   , POMDP_State::Action__to__Observation_to_Belief& _successor_belief_state
                   , uint action_index
                   , MDP_State* successor_state
                   , double probability);
        void add_entry(POMDP_State::Normalisation_Factors&
                       , POMDP_State::Action__to__Observation_to_Belief&
                       , uint 
                       , Planning::Observational_State*
                       , Planning::MDP_State* 
                       , double);
        void press__belief_transitions(POMDP_State*,
                                       const POMDP_State::Normalisation_Factors& _normalisation_Factors,
                                       const POMDP_State::Action__to__Observation_to_Belief&);

        /* \argument{Observational_State} is being generated, action
         * \argument{ID_TYPE} was executed, and led to
         * \argument{State}.
         *
         * ASSUME: \argument{Observational_State} is allocated and thus
         * non-null*/
        std::vector<Planning::Observational_State*>
        expand_observations(Observational_State* , ID_TYPE , State* );
        
        /* Supposing action \argument{ID_TYPE} was executed, and led
         * to \argument{State}, we add the possibile observations that
         * can be seen at \argument{State}.*/
        void expand_observations(const State_Transformation*, State* );
        
        /* Compute/generate all the successors and executions
         * probabilities for action \argument{State_Transformation}.*/
        std::vector<Planning::State*> expand(Planning::State* ,
                                             const State_Transformation*);
        
        /* Complete expansion of \argument{State} without regard to
         * \member{State::applicable_optional_transformations}.*/
        std::vector<Planning::State*> expand(Planning::State*);

        /* Compute/generate all the successors, corresponding actions,
         * and executions probabilities. */
        void expand_optional_transformations(Planning::State*);
        void expand_optional_transformation(Planning::State*, const State_Transformation*);

#ifdef LAO_STAR
        bool lao_star();
        void prioritise(Planning::POMDP_State* state,
                        Planning::Set_Of_POMDP_State_Pointers& locally_traversed);
#endif
        
        Planning::Set_Of_State_Pointers state_space;
        Planning::Set_Of_POMDP_State_Pointers belief_state__space;
        Planning::Set_Of_Observational_State_Pointers observation__space;
        
        /* MUST CALL THIS member BEFORE ANY SOLVING CAN OCCUR!
         *
         * - Initialisation of \member{domain_Data}.
         *
         * - Initialisation of \member{problem_Grounding}.
         *
         * - Merge object and constants data from the \member{problem}
         * and its associated problem.
         *
         */
        void preprocess();

        /* Is this solver in a sane state? */
        bool sanity() const;


        /* Delete and remove all entries from
         * \member{belief_state__space}. Then add the starting state
         * (belief state) to that set.*/
        void reset__pomdp_state_hash_table();
        
        /*Puts the starting belief state back on to the OPEN LISP.*/
        virtual void reinstate__starting_belief_state();

        /* Changes the starting belief state to correspond with
         * \argument{pomdp_state}.*/
        void instate__starting_belief_state(Planning::POMDP_State* pomdp_state);
    
        /*Sets the OPEN LIST to have no elements in it.*/
        virtual void empty__belief_states_for_expansion();

        /* Puts all the MDP states that occure in the initial belief
         * state into the OPEN LIST.*/
        void generate_markov_decision_process_starting_states();

        
        const std::map<Type, Constants>& get__extensions_of_types() const;
        CXX__PTR_ANNOTATION(Problem_Grounding) get__problem_Grounding();

        const Planning::POMDP_State* get__starting_belief_state()const;

        POMDP_State* solve__for_new_starting_state(Planning::POMDP_State* successor_state);
    protected:
        /*Initial POMDP state.*/
        Planning::POMDP_State* starting_belief_state;
    private:
        
        /* Compute the problem starting states and add them to
         * \member{expansion_stack} and \member{state_space}.*/
        void generate_starting_state();

        
        
        /* - Add \member{domain_Data::constants} and associated data to
         * \member{problem_Data}.
         *
         * - Configure \member{extensions_of_types} based on the
         *   merging of domain constants and problem objects.
         */
        void proprocess__Constants_Data();

        /* Takes all the constants from the definition of the
         * \member{problem_Data} associated with the
         * \member{problem_Data}, and makes them objects of the
         * \member{problem_Data}.*/
        void domain_constants__to__problem_objects();

        /* Configuration (i.e., initialisation) of
         * \member{extensions_of_types}.*/
        void configure__extensions_of_types();

    private:
        /*FIFO queue for simple state expansion.*/
        std::queue<Planning::POMDP_State*> expansion_queue;
        
        /* Functionality for obtaining a ground version of the problem
         * at hand.*/
        CXX__PTR_ANNOTATION(Problem_Grounding) problem_Grounding;
        
        /* PDDL types for \member{constants}*/
        Planning::Constants_Description constants_Description;

        /* Reverse map associated with
         * \member{constants_Description}. A type can be interpreted
         * in terms of the corresponding set of constants of that
         * type. Here we store that interpretation.*/
        std::map<Type, Constants> extensions_of_types;
        
        /*Problem targetted by this solver.*/
        Planning::Parsing::Problem_Data& problem_Data;
        
        /*(see \member{preprocess})*/
        bool preprocessed;
        
        /* Domain data associated with \member{problem} (see
         * \member{preprocess}).*/
        CXX__PTR_ANNOTATION(Planning::Parsing::Domain_Data) domain_Data;

        /*An observation of nothing.*/
        Observational_State* null_observation;

        /*Penalty for ending up in an illegal state.*/
        double sink_state_penalty;
    };
}

#endif


/* A digital wiring configuration comprises a switch control allowing
 * a user to select a function to control a corresponding electrical
 * device. A control unit couples electrical power to the electrical
 * devices through power outlets. The control unit allows an operator
 * to dynamically configure the switch controls to operate electrical
 * devices at specified power outlets. Upon selection of a function on
 * the switch control, the switch control transmits both a switch
 * state, indicative of the function selected, and a switch
 * identification that uniquely identifies that switch control. The
 * control unit receives the switch state and the switch
 * identification and generates a device identification uniquely
 * identifying the power outlet corresponding to the control
 * switch. The control unit transmits the device identification and
 * the switch state to the power outlets. The corresponding power
 * outlet is selected through the device identification and executes
 * the function in accordance with the switch state.
 *
 * -- Abstract from United States Patent 5,455,464, by James Gosling
 * (that Java guy), 1992.
 */

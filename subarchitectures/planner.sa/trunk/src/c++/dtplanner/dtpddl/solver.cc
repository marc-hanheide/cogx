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

#include "solver.hh"

#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"
#include "problem_grounding.hh"

#include "planning_state.hh"

#include "action__literal.hh"
#include "action__state_transformation.hh"
#include "action__probabilistic_state_transformation.hh"

#include "state_formula__literal.hh"
#include "state_formula__disjunctive_clause.hh"
#include "state_formula__conjunctive_normal_form_formula.hh"

/*Get rid of this if you stop doing policy iteration in the solver.*/
#include "policy_iteration_over_information_state_space__GMRES.hh"

using namespace Planning;
using namespace Planning::Parsing;


extern int max_expanded_states;

extern double beta;

Are_Doubles_Close Solver::are_Doubles_Close(1e-9);

void Solver::cleanup()
{
    for(auto state = state_space.begin()
            ; state != state_space.end()
            ; state++){
        delete *state;
    }
    for(auto state = belief_state__space.begin()
            ; state != belief_state__space.end()
            ; state++){
        delete *state;
    }
    for(auto state = observation__space.begin()
            ; state != observation__space.end()
            ; state++){
        delete *state;
    }
}


// size_t extern_runtime_thread_thingi;
Solver::~Solver()
{
    for(auto state = state_space.begin()
            ; state != state_space.end()
            ; state++){
        delete *state;
    }
    for(auto state = belief_state__space.begin()
            ; state != belief_state__space.end()
            ; state++){
        delete *state;
    }
    for(auto state = observation__space.begin()
            ; state != observation__space.end()
            ; state++){
        delete *state;
    }
}


std::pair<Planning::Formula::Action_Proposition, uint>
Solver::get_prescribed_action(POMDP_State* state)
{
    basic_type::Runtime_Thread runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>
        (dynamic_cast<const Planning::Problem_Grounding*>(problem_Grounding.get()));
// extern_runtime_thread_thingi = runtime_Thread;


    auto prescribed_action_index = state->get__prescribed_action();
    
    QUERY_UNRECOVERABLE_ERROR(!State_Transformation::
                              ith_exists(runtime_Thread, prescribed_action_index)
                              , "Could not find a ground symbol associated with index :: "
                              << prescribed_action_index);
    
    INTERACTIVE_VERBOSER(true, 10015,
                         "Got successor driver :: "<<prescribed_action_index<<" "
                         <<State_Transformation::
                         make_ith<State_Transformation>
                         (runtime_Thread,
                          prescribed_action_index).get__identifier()<<std::endl);
    
    auto symbol = State_Transformation::
        make_ith<State_Transformation>
        (runtime_Thread,
         prescribed_action_index);
    auto& identifier = symbol.get__identifier();
    auto id_value =  symbol.get__id();
    
    std::pair<Planning::Formula::Action_Proposition, uint> result(identifier, id_value);
    return result;
    
//     auto belief = state->get__belief_state();
//     uint index = random() % belief.size();
//     POMDP_State::Belief_Atom atom = belief[index];


//     assert(dynamic_cast<State*>(atom.first));
    
//     return get_prescribed_action(dynamic_cast<State*>(atom.first));//mdp_state);
}

/*Let's remove a few bugs at a time ;) */
std::pair<Planning::Formula::Action_Proposition, uint>
Solver::get_prescribed_action(State* current_state)
{
    basic_type::Runtime_Thread runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>
        (dynamic_cast<const Planning::Problem_Grounding*>(problem_Grounding.get()));

    INTERACTIVE_VERBOSER(true, 10015, "Getting prescribed action from :: "
                         <<*current_state<<std::endl);
    
    auto executable_action_indices = current_state->get__successor_Driver();
    auto _action_index = random() % executable_action_indices.size();
    auto action_index = executable_action_indices[_action_index];
    

#ifndef DNDEBUG
    if(!Formula::State_Proposition::
       ith_exists(runtime_Thread, action_index)){
        for(auto i =0; i < action_index; i++){
            if(!Formula::State_Proposition::
               ith_exists(runtime_Thread, i)){

                std::cerr<<State_Transformation::
                    make_ith<State_Transformation>
                    (runtime_Thread,
                     action_index).get__identifier();
            }
            
        }
    }
#endif
    
    
    QUERY_UNRECOVERABLE_ERROR(!State_Transformation::
                              ith_exists(runtime_Thread, action_index)
                              , "Could not find a ground symbol associated with index :: "
                              << action_index);
    
    INTERACTIVE_VERBOSER(true, 10015,
                         "Got successor driver :: "<<action_index<<" "
                         <<State_Transformation::
                         make_ith<State_Transformation>
                         (runtime_Thread,
                          action_index).get__identifier()
                         <<"For atom :: "<<*current_state<<std::endl);
    
    auto symbol = State_Transformation::
        make_ith<State_Transformation>
        (runtime_Thread,
         action_index);
    auto identifier = symbol.get__identifier();
    auto id_value =  symbol.get__id();
    std::pair<Planning::Formula::Action_Proposition, uint> result(identifier, id_value);
    
    return result;
}

Observational_State* Solver::find_observation(Observational_State* observation_state)
{
    auto index = observation__space.find(observation_state);

    if(index == observation__space.end()){
        return 0;
    } else {
        return *index;
    }
}

POMDP_State* Solver::compute_successor(Observational_State* observation,
                                 uint action_index,
                                 POMDP_State* current_state)
{
    auto result =  current_state->get__successor(action_index, observation);
    
    QUERY_UNRECOVERABLE_ERROR
        (!result
         , "Unknown successor state"<<std::endl);

    return result;
}



POMDP_State* Solver::take_observation(POMDP_State* current_state,
                                      Observational_State* observation,
                                      uint action_index)
{
    auto successor_state
        = compute_successor(observation, action_index, current_state);

    if(successor_state->unexpanded()){
        expand_belief_state(successor_state);
    }
    

    
    return successor_state;
}

std::vector<double>
Solver::report__probabilities_of_facts(POMDP_State* current_state,
         const Percept_List& propositions)
{
    std::vector<double> answer;
    
    Formula::List__State_Propositions planning_propositions;
    for(auto prop = propositions.begin()
            ; prop != propositions.end()
            ; prop++){
        std::string _predicate_name = (*prop).first;

        NEW_referenced_WRAPPED
            (domain_Data.get()
             , Planning::Predicate_Name
             , predicate_name
             , _predicate_name);
        
        Planning::Constant_Arguments constant_Arguments;
        for(auto _argument = (*prop).second.begin()
                ; _argument != (*prop).second.end()
                ; _argument++){

            std::string argument = *_argument;
            
            NEW_referenced_WRAPPED
                (&problem_Data//runtime_Thread
                 , Planning::Constant
                 , constant
                 , argument);
            constant_Arguments.push_back(constant);
        }
        
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (problem_Grounding.get()
             , Formula::State_Proposition
             , __proposition
             , predicate_name
             , constant_Arguments);
        auto proposition =  Formula::State_Proposition__Pointer(__proposition);
        planning_propositions.push_back(proposition);
    }

    auto& belief_state = current_state->get__belief_state();
    
    for(auto prop = planning_propositions.begin()
            ; prop != planning_propositions.end()
            ; prop++){
        auto& belief_state = current_state->get__belief_state();
        double prop_probability = 0.0;
        
        for(auto atom_info = belief_state.begin()
                ; atom_info != belief_state.end()
                ; atom_info++){
            auto probability = atom_info->second;
            auto state = atom_info->first;

            if(state->is_true((*prop)->get__id())){
                prop_probability += probability;
            }
        }

        answer.push_back(prop_probability);
    }
    
    return std::move(answer);
}



POMDP_State* Solver::take_observation(POMDP_State* current_state,
                                const Percept_List& perceptions,
                                uint action_index)
{
    VERBOSER(10050, "Got percepts from THE PLANNING"<<std::endl);
    
//     basic_type::Runtime_Thread runtime_Thread = reinterpret_cast<basic_type::Runtime_Thread>
//         (dynamic_cast<const Planning::Problem_Grounding*>(problem_Grounding.get()));

    Observational_State* new_observation
        = new Observational_State(problem_Grounding->get__perceptual_Propositions().size());
    
    for(auto obs = perceptions.begin()
            ; obs != perceptions.end()
            ; obs++){
        std::string _predicate_name = (*obs).first;

        NEW_referenced_WRAPPED
            (domain_Data.get()
             , Planning::Percept_Name
             , percept_name
             , _predicate_name);
        
        Planning::Constant_Arguments constant_Arguments;
        for(auto _argument = (*obs).second.begin()
                ; _argument != (*obs).second.end()
                ; _argument++){

            std::string argument = *_argument;
            
            NEW_referenced_WRAPPED
                (&problem_Data//runtime_Thread
                 , Planning::Constant
                 , constant
                 , argument);
            constant_Arguments.push_back(constant);
        }
        
        NEW_referenced_WRAPPED_deref_visitable_POINTER
            (problem_Grounding.get()
             , Formula::Perceptual_Proposition
             , __proposition
             , percept_name
             , constant_Arguments);
        auto proposition =  Formula::Perceptual_Proposition__Pointer(__proposition);
        
        auto& perceptual_Propositions
            = get__problem_Grounding()->get__perceptual_Propositions();
        if(perceptual_Propositions.find(proposition) != perceptual_Propositions.end()){
            VERBOSER(10050, "Known percept :: "<<*proposition<<std::endl);
            new_observation->flip_on(proposition->get__id());
        } else {
            WARNING("Unknown perceptual proposition :: "<<proposition);
        }
    }

    auto observation = find_observation(new_observation);
    delete new_observation;
    
    
    QUERY_UNRECOVERABLE_ERROR
        (!observation
         , "Unknown observation :: "<<*observation<<std::endl);

    return take_observation(current_state, observation, action_index);
}

double Solver::get__sink_state_penalty() const
{
    return sink_state_penalty;
}

void Solver::set__sink_state_penalty(double in)
{
    sink_state_penalty = in;
}

Solver::Solver(Planning::Parsing::Problem_Data& problem_Data, double sink_state_penalty)
    :problem_Data(problem_Data),
     preprocessed(false),
     starting_belief_state(0),
     null_observation(0),
     sink_state_penalty(sink_state_penalty)// ,
//      constants_Description(0),
//      constants(0)
{
}

void Solver::preprocess()
{
    if(preprocessed) return;
    
    domain_Data = problem_Data.get__domain_Data();

    VERBOSER(3101, "Got domain :: "<<*domain_Data<<std::endl);
    
    
    
    proprocess__Constants_Data();
    
    configure__extensions_of_types();
    
    problem_Grounding = CXX__PTR_ANNOTATION(Problem_Grounding)
        (new Problem_Grounding(problem_Data,
                               domain_Data,
                               constants_Description,
                               extensions_of_types));

    problem_Grounding->ground_actions();
    problem_Grounding->ground_derived_predicates();
    problem_Grounding->ground_derived_perceptions();
    problem_Grounding->ground_starting_states();
    problem_Grounding->ground_objective_function();

    assert(problem_Grounding->get__deterministic_actions().size());
    problem_Grounding->ground_observations();
    QUERY_WARNING(!problem_Grounding->get__observations().size(),
                  "Problem has no observation schemata.");
//     assert(problem_Grounding->get__observations().size());
    
    INTERACTIVE_VERBOSER(true, 10501, "Calling generation of starting state.")
                
    generate_starting_state();
    
    INTERACTIVE_VERBOSER(true, 10501, "Done generation of starting state, preprocessing completed.")
    
    
    preprocessed = true;
}

void Solver::empty__belief_states_for_expansion()
{
    INTERACTIVE_VERBOSER(true, 14000, "Emptying the expansion queue."<<std::endl);
    
    expansion_queue = std::queue<Planning::POMDP_State*>();


    assert(expansion_queue.size() == 0);
}

void Solver::reset__pomdp_state_hash_table()
{
    VERBOSER(14000, "Reseting the POMDP hash table."<<std::endl);
    
    /*Clean up some of the memory used in the first phase. */
    for(auto bstate = belief_state__space.begin()
            ; bstate != belief_state__space.end()
            ; bstate++){
        if(*bstate != starting_belief_state){
            delete *bstate;
        }
        
    }
    
    
    belief_state__space = Planning::Set_Of_POMDP_State_Pointers();
    belief_state__space.insert(starting_belief_state);
}



void Solver::reinstate__starting_belief_state()
{
    assert(expansion_queue.size() == 0);
    expansion_queue.push(starting_belief_state);
}

void Solver::instate__starting_belief_state(Planning::POMDP_State* pomdp_state)
{
    starting_belief_state = pomdp_state;
    //assert(expansion_queue.size() == 0);
    expansion_queue.push(starting_belief_state);
}

void Solver::generate_markov_decision_process_starting_states()
{
    assert(expansion_queue.size() == 0);
    auto belief_states = starting_belief_state->get__belief_state();
    for(auto belief_state = belief_states.begin()
            ; belief_state != belief_states.end()
            ; belief_state++){
        auto mdp_state = belief_state->first;

        auto state = new POMDP_State();
        state->add__belief_atom(mdp_state, 1.0);
        state->set__index(belief_state__space.size());
        state->initialise__prescribed_action_index();
        belief_state__space.insert(state);
        report__new_belief_state(state);
    }
}

#ifdef LAO_STAR

void Solver::prioritise(Planning::POMDP_State* state,
                        Planning::Set_Of_POMDP_State_Pointers& locally_traversed)
{
    if(locally_traversed.find(state) != locally_traversed.end()){
        return;
    }

    locally_traversed.insert(state);
    
    INTERACTIVE_VERBOSER(true, 14000, "Testing new state :: "<<*state<<std::endl
                         <<!state->get__expansion_attempted()<<std::endl);
    
    if(!state->get__expansion_attempted()){    
        INTERACTIVE_VERBOSER(true, 14000, "Reporting new state :: "<<*state<<std::endl);
        report__new_belief_state(state);
        return;
    }
    
    if(state->unexpanded()){
        return;
    }
    
    
    auto prescribed_action_index = state->get__prescribed_action();
//     auto atoms = state->get__successors();
    auto& atoms = state->get__successors(prescribed_action_index);

    for(auto atom = atoms.begin()
            ; atom != atoms.end()
            ; atom++){
        Solver::prioritise(*atom, locally_traversed);
    }
}

// void Solver::prioritise(Planning::POMDP_State* state,
//                         Planning::Set_Of_POMDP_State_Pointers& locally_traversed)
// {
//     if(locally_traversed.find(state) != locally_traversed.end()){
//         return;
//     }

//     locally_traversed.insert(state);
    
//     INTERACTIVE_VERBOSER(true, 14000, "Testing new state :: "<<*state<<std::endl
//                          <<!state->get__expansion_attempted()<<std::endl);
    
//     if(!state->get__expansion_attempted()){    
//         INTERACTIVE_VERBOSER(true, 14000, "Reporting new state :: "<<*state<<std::endl);
//         report__new_belief_state(state);
//         return;
//     }
    
//     if(state->unexpanded()){
//         return;
//     }
    
    
//     auto prescribed_action_index = state->get__prescribed_action();
//     auto& atoms = state->get__successors(prescribed_action_index);

//     for(auto atom = atoms.begin()
//             ; atom != atoms.end()
//             ; atom++){
//         Solver::prioritise(*atom, locally_traversed);
//     }
// }

// bool Solver::lao_star()
// {
//     if(this->belief_state__space.size() > max_expanded_states) {return false;}
    

//     std::vector<Planning::POMDP_State*> local__states_to_expand;
//     Planning::POMDP_State* some_state;
//     while(some_state = obtain__next_belief_state_for_expansion()){
//         if(some_state->get__expansion_attempted()) continue;

        
//         QUERY_UNRECOVERABLE_ERROR(some_state->get__expansion_attempted(),
//                                   "#1Expanding a state for the second time...");
        
//         local__states_to_expand.push_back(some_state);
//     }
//     if(local__states_to_expand.size() == 0) return false;

    
    
//     VERBOSER(15000, "Iterating LAO. Expanding a number of states :: "
//              <<local__states_to_expand.size()<<std::endl);

    
// //     Planning::Set_Of_POMDP_State_Pointers local_test_set;
//     for(auto state = local__states_to_expand.begin()
//             ; state != local__states_to_expand.end()
//             ; state++){
        

// //         if((*state)->get__expansion_attempted()){
// //             QUERY_UNRECOVERABLE_ERROR(*state != starting_belief_state,
// //                                       "Non starting state double expansion :: "
// //                                       <<(**state == *starting_belief_state));
// //             continue;
// //         }
        
// //         QUERY_UNRECOVERABLE_ERROR(local_test_set.find(*state) != local_test_set.end(),
// //                                   "Same state twice...");
        
//         QUERY_UNRECOVERABLE_ERROR((*state)->get__expansion_attempted(),
//                                   "#2Expanding a state for the second time...");
//         (*state)->set__expansion_attempted();

// //         local_test_set.insert(*state);
        
//         expand_belief_state(*state);
//     }
    
    
// //     if(!this->expand_belief_state_space()){
// //         return false;
// //     }

// //     /*Expand the whole fringe.*/
// //     while(this->expand_belief_state_space()){
// //     }
    
//     Planning::Policy_Iteration__GMRES policy_Iteration(this->belief_state__space,
//                                                        this->get__sink_state_penalty(),
//                                                        .95);

//     //policy_Iteration();
//     while(policy_Iteration()){};
    
//     VERBOSER(15000, "Iterating LAO with a number of belief state :: "<<this->belief_state__space.size()<<std::endl);
    
//     Planning::Set_Of_POMDP_State_Pointers locally_traversed;
//     this->empty__belief_states_for_expansion();
//     assert(!obtain__next_belief_state_for_expansion());
//     prioritise(starting_belief_state, locally_traversed);
    
//     VERBOSER(15000, "Iterating LAO locality :: "<<locally_traversed.size()<<std::endl);

//     return true;
// }

bool Solver::lao_star()
{
    if(this->belief_state__space.size() > max_expanded_states) {return false;}
    

    std::vector<Planning::POMDP_State*> local__states_to_expand;
    Planning::POMDP_State* some_state;
    while(some_state = obtain__next_belief_state_for_expansion()){
        ;//if(some_state->get__expansion_attempted()) continue;

        
        QUERY_UNRECOVERABLE_ERROR(some_state->get__expansion_attempted(),
                                  "#1Expanding a state for the second time...");
        
        local__states_to_expand.push_back(some_state);
    }
    
    if(local__states_to_expand.size() == 0) {return false;}
    

    
    
    VERBOSER(189000, "Iterating LAO. Expanding a number of states :: "
             <<local__states_to_expand.size()<<std::endl);

    
//     Planning::Set_Of_POMDP_State_Pointers local_test_set;
    for(auto state = local__states_to_expand.begin()
            ; state != local__states_to_expand.end()
            ; state++){
        

//         if((*state)->get__expansion_attempted()){
//             QUERY_UNRECOVERABLE_ERROR(*state != starting_belief_state,
//                                       "Non starting state double expansion :: "
//                                       <<(**state == *starting_belief_state));
//             continue;
//         }
        
//         QUERY_UNRECOVERABLE_ERROR(local_test_set.find(*state) != local_test_set.end(),
//                                   "Same state twice...");
        
        QUERY_UNRECOVERABLE_ERROR((*state)->get__expansion_attempted(),
                                  "#2Expanding a state for the second time...");
        (*state)->set__expansion_attempted();

//         local_test_set.insert(*state);
        
        expand_belief_state(*state);
    }
    
    
//     if(!this->expand_belief_state_space()){
//         return false;
//     }

//     /*Expand the whole fringe.*/
//     while(this->expand_belief_state_space()){
//     }
    
    Planning::Policy_Iteration__GMRES policy_Iteration(this->belief_state__space,
                                                       this->get__sink_state_penalty(),
                                                       beta);

    //    policy_Iteration();
    while(policy_Iteration()){};
    
    VERBOSER(15000, "Iterating LAO with a number of belief state :: "<<this->belief_state__space.size()<<std::endl);
    
    Planning::Set_Of_POMDP_State_Pointers locally_traversed;
    this->empty__belief_states_for_expansion();
    assert(!obtain__next_belief_state_for_expansion());
    prioritise(starting_belief_state, locally_traversed);
    
    VERBOSER(15000, "Iterating LAO locality :: "<<locally_traversed.size()<<std::endl);

    return true;
}

#endif


#ifdef LAO_STAR

/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/


POMDP_State* /*LAO_STAR*/Solver::solve__for_new_starting_state(Planning::POMDP_State* successor_state)
{
    
    auto new_starting_belief_state = new Planning::POMDP_State();
    auto belief = successor_state->get__belief_state();
    for(auto atom = belief.begin()
            ; atom != belief.end()
            ; atom++){
        double prob = atom->second;
        auto state = atom->first;

        
        INTERACTIVE_VERBOSER(true, 14000, "Creating new belief state with atom :: "
                             <<*state<<std::endl
                             );
        
        new_starting_belief_state->add__belief_atom(state, prob);
    }

    
    new_starting_belief_state->set__index(0);
    new_starting_belief_state->initialise__prescribed_action_index();
    
    this->instate__starting_belief_state(new_starting_belief_state);/*Alters the starting state.*/
    this->empty__belief_states_for_expansion();
    
    this->reset__pomdp_state_hash_table();
    this->reinstate__starting_belief_state();
    
    auto current_state = this->peek__next_belief_state_for_expansion();//expansion_queue.front();
    QUERY_UNRECOVERABLE_ERROR(!current_state, "No future state for expansion, presumably the expansion queue is empty.");
    
    INTERACTIVE_VERBOSER(true, 14000, "Current state is :: "
                         <<*current_state<<std::endl
                         );

    while(lao_star()){
    }

    INTERACTIVE_VERBOSER(true, 18900, "Current state is :: "
                         <<this->belief_state__space.size()<<std::endl
                         );
    
//     Planning::Policy_Iteration__GMRES policy_Iteration(this->belief_state__space,
//                                                        this->get__sink_state_penalty(),
//                                                        .95);

//     while(policy_Iteration()){};

    return current_state;
}

/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/
/*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*//*LAO_STAR*/


#else

/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/
/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/
/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/
/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/
/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/

POMDP_State* /*NON-LAO_STAR*/Solver::solve__for_new_starting_state(Planning::POMDP_State* successor_state)
{
    assert(successor_state);
    
    auto new_starting_belief_state = new Planning::POMDP_State();
    auto belief = successor_state->get__belief_state();
    for(auto atom = belief.begin()
            ; atom != belief.end()
            ; atom++){
        double prob = atom->second;
        auto state = atom->first;

        
        INTERACTIVE_VERBOSER(true, 14000, "Creating new belief state with atom :: "
                             <<*state<<std::endl
                             );
        
        new_starting_belief_state->add__belief_atom(state, prob);
    }

    
    new_starting_belief_state->set__index(0);
    new_starting_belief_state->initialise__prescribed_action_index();
    
    this->instate__starting_belief_state(new_starting_belief_state);/*Alters the starting state.*/
    
    this->empty__belief_states_for_expansion();

    
    this->reset__pomdp_state_hash_table();
    this->reinstate__starting_belief_state();
    
    auto current_state = this->peek__next_belief_state_for_expansion();//expansion_queue.front();
    QUERY_UNRECOVERABLE_ERROR(!current_state, "No future state for expansion, presumably the expansion queue is empty.");
    
    Planning::Policy_Iteration__GMRES policy_Iteration(this->belief_state__space,
                                                       this->get__sink_state_penalty(),
                                                       beta);
    
    INTERACTIVE_VERBOSER(true, 14000, "Current state is :: "
                         <<*current_state<<std::endl
                         );

    
    for(uint i = 0; i < 100000; i++){
        if(!this->expand_belief_state_space()){
            break;
            VERBOSER(14000, "No starting state!"<<std::endl);
        } else {
            VERBOSER(15000, "Expanding (so far we have "
                     <<this->belief_state__space.size()<<" beliefs)!"<<std::endl
                     <<"Expected reward is :: "
                     <<current_state->get__expected_value()<<std::endl);
        }

        if(this->belief_state__space.size() > max_expanded_states)break;
    }
    
    while(policy_Iteration()){
        
        VERBOSER(15000, "PI.. Expected reward is :: "
                 <<current_state->get__expected_value()<<std::endl);

    }

    return current_state;
}

/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/
/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/
/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/
/*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*//*NON-LAO_STAR*/
#endif

const Planning::POMDP_State* Solver::get__starting_belief_state()const
{
    return starting_belief_state;
}

void Solver::domain_constants__to__problem_objects()
{
    VERBOSER(3001, "Adding domain constants to the problem description.");
    
//     const Constants_Data& problem__Constants_Data = problem_Data;
    const Constants_Data& domain__Constants_Data = *domain_Data;
    auto domain__constants_Description = domain__Constants_Data.get__constants_Description();
    
    for(auto _constant = domain__constants_Description.begin()
            ; _constant != domain__constants_Description.end()
            ; _constant++){

        const Constant& constant = _constant->first;
        
        VERBOSER(3001, "Adding domain constant :: "<<constant<<std::endl
                 <<"As problem object. "<<std::endl);
        
        problem_Data
            .add__constant(constant.get__name());
        
        auto types = domain__Constants_Data.get__constantx_types(constant);

        QUERY_UNRECOVERABLE_ERROR
            (!types.size(),
             "No types were specified for domain constant :: "<<constant<<std::endl);
        
        for(auto type = types.begin()
                ; type != types.end()
                ; type++){

            QUERY_UNRECOVERABLE_ERROR(domain_Data->get__types_description().find(*type)
                                      == domain_Data->get__types_description().end(),
                                      "Thread :: "<<domain_Data->get__types_description().begin()->first.get__runtime_Thread()<<std::endl
                                      <<"Got query from thread :: "<<type->get__runtime_Thread()<<std::endl
                                      <<"For domain at :: "<<reinterpret_cast<basic_type::Runtime_Thread>(domain_Data.get())<<std::endl);
            
            assert(type->get__runtime_Thread() == reinterpret_cast<basic_type::Runtime_Thread>(domain_Data.get()));
            assert(domain_Data->get__types_description().find(*type) != domain_Data->get__types_description().end());
            
            problem_Data
                .add__type_of_constant(type->get__name());
        }
        
        problem_Data
            .add__constants();
    }

    constants_Description = problem_Data.get__constants_Description();
    
}

void Solver::configure__extensions_of_types()
{
    /* For each problem constant. */
    for(auto constant_Description = constants_Description.begin()
            ; constant_Description != constants_Description.end()
            ; constant_Description++){
        auto types = constant_Description->second;
        auto constant = constant_Description->first;


        QUERY_UNRECOVERABLE_ERROR(types.size() > 1
                                  , "Each object is supposed to be of exactly one type."<<std::endl
                                  <<"However :: "<<constant<<" was declared with type :: "<<types<<std::endl);
        QUERY_UNRECOVERABLE_ERROR(types.size() == 0
                                  , "Each object is supposed to be of exactly one type."<<std::endl
                                  <<"However :: "<<constant<<" was declared without a type."<<std::endl);
        
        for(auto type = types.begin()
                ; type != types.end()
                ; type++){

            auto types_description = domain_Data->get__types_description();


//             std::string for_debug;
//             {
//                 std::ostringstream oss;
//                 oss<<*domain_Data<<std::endl;
//                 for(auto thing = types_description.begin()
//                         ; thing != types_description.end()
//                         ; thing++){
//                     oss<<thing->first<<" "<<thing->second<<std::endl;
//                 }
//                 for_debug = oss.str();  
//             }
            
            QUERY_UNRECOVERABLE_ERROR(types_description.find(*type) == types_description.end(),
                                      "Unable to find :: "<<*type<<std::endl
                                      <<"In the domain type hierarchy :from : "<<*domain_Data<<std::endl);

            if(extensions_of_types.find(*type) == extensions_of_types.end()){
                extensions_of_types[*type] = Constants();
            }

            VERBOSER(3001, "Adding :: "<<constant<<" of type ::"<<*type<<std::endl);
            
            extensions_of_types[*type].insert(constant);
            
            for(auto super_type = types_description.find(*type)->second.begin()
                    ; super_type != types_description.find(*type)->second.end()
                    ; super_type++){
                extensions_of_types[*super_type].insert(constant);
            }
        }
    }
}

void Solver::proprocess__Constants_Data()
{
    domain_constants__to__problem_objects();
    
    
}



bool Solver::sanity() const
{
    if(!preprocessed) {
        WARNING("Tested sanity on :: "<<problem_Data.get__problem_Name()
                <<"before preprocessing."<<std::endl);
        return false;
    }

    return true;
}



CXX__PTR_ANNOTATION(Problem_Grounding)  Solver::get__problem_Grounding()
{
    return problem_Grounding;
}

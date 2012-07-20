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

#include "global.hh"
#include "Command_Line_Arguments.hh"
#include "dtp_pddl_parsing_interface.hh"
#include "dtp_pddl_parsing_data.hh"
#include "dtp_pddl_parsing_data_problem.hh"

#include "turnstyle.hh"

#include "solver.hh"
#include "simple_online_solver.hh"
#include "two_phase_solver.hh"
#include "flatten.hh"
// #include "policy_iteration_over_information_state_space.hh"
#include "policy_iteration_over_information_state_space__GMRES.hh"

using std::endl;

/* WARNING :: C++ zero initialisation is not safe.*/
Command_Line_Arguments command_Line_Arguments;

int min_nonobs_steps = 1;
int max_expanded_states = 500;
double beta = 0.95;
double beta_mdp = 0.95;

namespace GLOBAL__read_in__domain_description
{
    unsigned short int CALL_COUNT__read_in__domain_description = 0;
}

bool read_in__domain_description()
{
    using GLOBAL__read_in__domain_description::CALL_COUNT__read_in__domain_description;

    VERBOSER(19, "");
    std::string guard = "--domain";
    std::ostringstream oss;
    if(CALL_COUNT__read_in__domain_description){
        oss<<CALL_COUNT__read_in__domain_description;
        guard += oss.str();
        VERBOSER(19, "");
    } 
    
    CALL_COUNT__read_in__domain_description++;
    
    std::string domain_file_name = "domain.pddl";
    if(!(command_Line_Arguments.got_guard(guard))){

        VERBOSER(19, "");
        /*First call*/
        if(1 == CALL_COUNT__read_in__domain_description &&
           command_Line_Arguments.got_guard(guard+"0")){
                domain_file_name = command_Line_Arguments.get_string();
                VERBOSER(19, "");
        } else if (1 == CALL_COUNT__read_in__domain_description) {
            WARNING("Missing argument ::"<<endl
                    <<guard<<" filename"<<endl
                    <<"(or "<<guard<<"0 filename)"<<endl
                    <<"Using :: "<<domain_file_name<<endl);
            VERBOSER(19, "");
        } else {
            return false;
        }
    } else {
        domain_file_name = command_Line_Arguments.get_string();
        VERBOSER(1, "Using domain file :: "
                 <<domain_file_name<<" :: for domain definition."); 
        VERBOSER(19, "");
    }
    
    Planning::Parsing::parse_domain(domain_file_name);
    return true;
}

namespace GLOBAL__read_in__problem_description
{
    unsigned short int CALL_COUNT__read_in__problem_description = 0;
}

bool read_in__problem_description()
{
    using GLOBAL__read_in__problem_description::CALL_COUNT__read_in__problem_description;

    VERBOSER(501, "");
    std::string guard = "--problem";
    std::ostringstream oss;
    if(CALL_COUNT__read_in__problem_description){
        oss<<CALL_COUNT__read_in__problem_description;
        guard += oss.str();
        VERBOSER(501, "");
    } 
    
    CALL_COUNT__read_in__problem_description++;
    
    std::string problem_file_name = "problem.pddl";
    if(!(command_Line_Arguments.got_guard(guard))){

        VERBOSER(501, "");
        /*First call*/
        if(1 == CALL_COUNT__read_in__problem_description &&
           command_Line_Arguments.got_guard(guard+"0")){
                problem_file_name = command_Line_Arguments.get_string();
                VERBOSER(501, "");
        } else if (1 == CALL_COUNT__read_in__problem_description) {
            WARNING("Missing argument ::"<<endl
                    <<guard<<" filename"<<endl
                    <<"(or "<<guard<<"0 filename)"<<endl
                    <<"Using :: "<<problem_file_name<<endl);
            VERBOSER(501, "");
        } else {
            return false;
        }
    } else {
        problem_file_name = command_Line_Arguments.get_string();
        VERBOSER(1, "Using problem file :: "
                 <<problem_file_name<<" :: for problem definition."); 
        VERBOSER(501, "");
    }
    
    Planning::Parsing::parse_problem(problem_file_name);
    return true;
}

// template<typename T_Solver>
// Planning::POMDP_State* expand_for_new_state(Planning::POMDP_State* successor_state,
//                           T_Solver* solver)
// {
//     //successor_state = const_cast<Planning::POMDP_State*>(solver->get__starting_belief_state());
    
//     auto new_starting_belief_state = new Planning::POMDP_State();
//     auto belief = successor_state->get__belief_state();
//     for(auto atom = belief.begin()
//             ; atom != belief.end()
//             ; atom++){
//         double prob = atom->second;
//         auto state = atom->first;

        
//         INTERACTIVE_VERBOSER(true, 14000, "Creating new belief state with atom :: "
//                              <<*state<<std::endl
//                              );
        
//         new_starting_belief_state->add__belief_atom(state, prob);
//     }

    
//     new_starting_belief_state->set__index(0);
//     new_starting_belief_state->initialise__prescribed_action_index();
    
//     solver->empty__belief_states_for_expansion();
//     solver->instate__starting_belief_state(new_starting_belief_state);/*Alters the starting state.*/
//     solver->reset__pomdp_state_hash_table();
//     solver->reinstate__starting_belief_state();
    
//     auto current_state = solver->peek__next_belief_state_for_expansion();//expansion_queue.front();
    
//     Planning::Policy_Iteration__GMRES policy_Iteration(solver->belief_state__space,
//                                                        solver->get__sink_state_penalty());
    
//     INTERACTIVE_VERBOSER(true, 15000, "Current state is :: "
//                          <<*current_state<<std::endl
//                          );

    
//     for(uint i = 0; i < 100000; i++){
//         if(!solver->expand_belief_state_space()){
//             break;
//             VERBOSER(15000, "No starting state!"<<std::endl);
//         } else {
//             VERBOSER(15000, "Expanding (so far we have "
//                      <<solver->belief_state__space.size()<<" beliefs)!"<<std::endl
//                      <<"Expected reward is :: "
//                      <<current_state->get__expected_value()<<std::endl);
//             //                 policy_Iteration();
//             //                     if(!(i % 10))policy_Iteration();

//             //                     policy_Iteration.reset__converged();
//         }

//         if(solver->belief_state__space.size() > 2000)break;
//     }
    
//     int counter = 0;
//     while(policy_Iteration() && counter < 10){
        
//         VERBOSER(15000, "PI.. Expected reward is :: "
//                  <<current_state->get__expected_value()<<std::endl);

//         counter++;
//     }

//     return current_state;// new_starting_belief_state;
// }


/* As requested by Moritz Fri Oct 22 in the planning meeting with
 * Richard Dearden.*/

#ifndef CHARLES_TESTING
    
int main(int argc, char** argv)
{
    assert(command_Line_Arguments.size() == 0);
    command_Line_Arguments = Command_Line_Arguments(argc, argv);
    
    if(!command_Line_Arguments.got_guard("--steps")){
        WARNING("Using --steps "<<min_nonobs_steps<<std::endl);
    } else {
        min_nonobs_steps = command_Line_Arguments.get_int();
    }
    if(!command_Line_Arguments.got_guard("--max-iStates")){
        WARNING("Using --max-iStates "<<max_expanded_states<<std::endl);
    } else {
        max_expanded_states = command_Line_Arguments.get_int();
    }
    
    if(!command_Line_Arguments.got_guard("--beta")){
        WARNING("Using --beta "<<beta<<std::endl);
    } else {
       beta = command_Line_Arguments.get_double();
    }
    
    if(!command_Line_Arguments.got_guard("--beta-mdp")){
        WARNING("Using --beta-mdp "<<beta_mdp<<std::endl);
    } else {
       beta_mdp = command_Line_Arguments.get_double();
    }
    
    while(read_in__domain_description()){};
    
    while(read_in__problem_description()){};
    
    assert(Planning::Parsing::domains.size() > 0);

    auto problem = Planning::Parsing::problems.begin();

    assert(problem != Planning::Parsing::problems.end());
    
#ifdef LAO_STAR
    string solver_name = "LAO-star";
    INTERACTIVE_VERBOSER(true, 15000, solver_name<<std::endl);
    
#define CHANGE_PHASE 1
    auto solver = new Planning::Two_Phase_Solver(*problem->second);
#else       
    string solver_name = "Sondik 197";
    INTERACTIVE_VERBOSER(true, 15000, solver_name<<std::endl);

    auto solver = new Planning::Solver(*problem->second);
#endif
    
    solver->set__sink_state_penalty(-1.0);
    
    solver->preprocess();
    
#ifdef LAO_STAR
        
    solver->empty__belief_states_for_expansion();
    solver->generate_markov_decision_process_starting_states();
    
    Planning::Policy_Iteration__GMRES policy_Iteration__for_MDP_states
        (solver->belief_state__space, solver->get__sink_state_penalty(), beta_mdp);

    {
        
        auto current_state = solver->peek__next_belief_state_for_expansion();
                
        while(solver->expand_belief_state_space()){
        }
        while(policy_Iteration__for_MDP_states()){
        }
    
        for(auto mdp_state = solver->belief_state__space.begin()
                ; mdp_state != solver->belief_state__space.end()
                ; mdp_state++){

            if(*mdp_state == solver->get__starting_belief_state()){
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

    
#ifdef CHANGE_PHASE
        solver->change_phase();//HERE -- MONDAY
#endif
                
        solver->reinstate__starting_belief_state();
    }
#endif

    INTERACTIVE_VERBOSER(true, 15000, solver_name<<" Getting belief state "<<std::endl);
    auto current_state = solver->peek__next_belief_state_for_expansion();

    assert(current_state);
    
#ifdef LAO_STAR
    INTERACTIVE_VERBOSER(true, 15000, solver_name<<" One iteration of LAO* "<<std::endl);
    while(solver->lao_star()){};
    Planning::Policy_Iteration__GMRES policy_Iteration(solver->belief_state__space,
                                                       solver->get__sink_state_penalty(),
                                                       beta_mdp);
    
    while(policy_Iteration()){};
#else
    INTERACTIVE_VERBOSER(true, 15000, solver_name<<" One iteration "<<std::endl);
    current_state = solver->solve__for_new_starting_state(current_state);
#endif


    bool post_an_action = true;
    
    int step_count = 0;
    
    while(true){

        std::pair<Planning::Formula::Action_Proposition, uint> _action
            = solver->get_prescribed_action(current_state);
        
        auto action = _action.first;
        auto action_index = _action.second;
            
        if(post_an_action){
            std::cout<<action<<std::endl;
        } else {
            post_an_action = true;
        }
        
        

    
        INTERACTIVE_VERBOSER(true, 15000, solver_name
                             <<" getting percepts "<<std::endl);

        bool parsing_observation;
        Planning::Solver::Proposition_List propositions;
        Planning::Solver::Percept_List percepts;    
        char ch = ' ';
        string line_in;
        
        
        ch = getchar();

        if(ch == 'P'){
                VERBOSER(18000, solver_name
                                     <<" Parsing a propositional string "<<std::endl);
            parsing_observation = false;
        }
        if(ch == 'O'){
                VERBOSER(18000, solver_name
                                     <<" Parsing an observation string "<<std::endl);
            parsing_observation = true;
        }
        
        do{
            ch = getchar();

            VERBOSER(18000, solver_name
                                 <<" :--> "<<ch<<std::endl);
            
            if(parsing_observation){
            
                if(ch == '(') {
                    VERBOSER(18000, solver_name
                                         <<" Got an opening bracket "<<std::endl);
                    ch = ' ';
                }

                if(ch == ')'){

                    VERBOSER(18000, solver_name
                                         <<" Got a closing bracket "<<std::endl);
            
                    Planning::Solver::Precept percept;

                    istringstream iss(line_in);

                    std::string str;
                    uint count = 0;
                    while(!iss.eof() && iss.good()){
                        iss>>str;
                        if(str == "") continue;
                    
                        std::cerr<<str<<std::endl;
                        if(count == 0){
                            percept.first = str;
                            percept.second = Planning::Solver::Precept_Arguments();
                        } else {
                            percept.second.push_back(str);
                        }
                
                        count++;
                        continue;
                    }
            
                    percepts.push_back(percept);
            
                    line_in = "";
            
                    continue;
                }
                
            } else {
                
            
                if(ch == '(') {
                    ch = ' ';
                }
        

                if(ch == ')'){

            
                    Planning::Solver::Proposition proposition;

                    istringstream iss(line_in);

                    std::string str;
                    uint count = 0;
                    while(!iss.eof() && iss.good()){
                        iss>>str;
                        if(str == "") continue;
                    
                        std::cerr<<str<<std::endl;
                        if(count == 0){
                            proposition.first = str;
                            proposition.second = Planning::Solver::Proposition_Arguments();
                        } else {
                            proposition.second.push_back(str);
                        }
                
                        count++;
                        continue;
                    }
            
                    propositions.push_back(proposition);
            
                    line_in = "";
            
                    continue;
                }        

        
            }
            
            line_in += ch;
                
            VERBOSER(18000, solver_name
                     <<" :--> "<<line_in<<std::endl);
            
        }while(ch != '\n');    

        if(parsing_observation){
            /*LOOP EXIT*/
            if(percepts.size() == 0) break;
    
            Planning::POMDP_State* successor_state
                = solver->take_observation(current_state,
                                           percepts,
                                           action_index);

    
            VERBOSER(18000, solver_name
                                 <<" generated the successor "
                                 <<*successor_state<<std::endl);
    
            auto& available_observations = current_state
                ->get__possible_observations_given_action(action_index);
    
            current_state = successor_state;
            
            if(1 < available_observations.size()){/*Should we replan, have our beliefs changed?*/
                step_count++;
                if(!(step_count % min_nonobs_steps)){
                    current_state = solver->solve__for_new_starting_state(successor_state);
                }
            }
        } else {
            post_an_action = false;

            std::vector<double> probabilities  =
                solver->report__probabilities_of_facts(current_state,
                                                       propositions);
            for(auto prob = probabilities.begin()
                    ; prob != probabilities.end()
                    ; prob++){
                std::cout<<*prob<<std::endl;
            }
        }
        
    }
    
    VERBOSER(18000, solver_name
                         <<" session terminated "<<std::endl);
    return 0;
}

#else


/*Charles' testing MAIN.*/
int main_OLD(int argc, char** argv)
{    
    assert(command_Line_Arguments.size() == 0);
    command_Line_Arguments = Command_Line_Arguments(argc, argv);

    if(!command_Line_Arguments.got_guard("--steps")){
        WARNING("Using --steps "<<min_nonobs_steps<<std::endl);
    } else {
        min_nonobs_steps = command_Line_Arguments.get_int();
    }
    
    if(!command_Line_Arguments.got_guard("--max-iStates")){
        WARNING("Using --max-iStates "<<max_expanded_states<<std::endl);
    } else {
        max_expanded_states = command_Line_Arguments.get_int();
    }
    
    if(!command_Line_Arguments.got_guard("--beta")){
        WARNING("Using --beta "<<beta<<std::endl);
    } else {
       beta = command_Line_Arguments.get_double();
    }
    
    if(!command_Line_Arguments.got_guard("--beta-mdp")){
        WARNING("Using --beta-mdp "<<beta_mdp<<std::endl);
    } else {
       beta_mdp = command_Line_Arguments.get_double();
    }
    
    int seed = 2010;
    srandom(seed);
    srand(seed);

    while(read_in__domain_description()){};
    while(read_in__problem_description()){};
    
    assert(Planning::Parsing::domains.size() > 0);

    INTERACTIVE_VERBOSER(true, 20000, "Done parsing problem and domain description");
    
    auto problem = Planning::Parsing::problems.begin();
        
    auto solver = new Planning::Two_Phase_Solver(*problem->second);
            
    solver->set__sink_state_penalty(-1.0); //-1234.0);//-1e6);
            
    INTERACTIVE_VERBOSER(true, 20000, "Made a solver, starting preprocessing.");
            
    solver->preprocess();

    
    solver->empty__belief_states_for_expansion();
    solver->generate_markov_decision_process_starting_states();

    Planning::Policy_Iteration__GMRES policy_Iteration__for_MDP_states
        (solver->belief_state__space, solver->get__sink_state_penalty(), .65);

    {
        auto current_state = solver->peek__next_belief_state_for_expansion();
        
        INTERACTIVE_VERBOSER(true, 20000, "MDP state expansion :: "<<*current_state<<std::endl);
        
        
        while(solver->expand_belief_state_space()){
            VERBOSER(20000, "MDP state expansion :: "<<solver->belief_state__space.size()<<std::endl);
        }
        
//         INTERACTIVE_VERBOSER(true, 20000, "FINISHED MDP state expansion :: "<<*current_state<<std::endl);
//         solver->reset__pomdp_state_hash_table();
        
        INTERACTIVE_VERBOSER(true, 20000, "RESET POMDP hash table :: "<<*current_state<<std::endl);
        solver->change_phase();
        
        INTERACTIVE_VERBOSER(true, 20000, "Reinstate starting belief state :: "<<*current_state<<std::endl);
        
        solver->reinstate__starting_belief_state();
        
        INTERACTIVE_VERBOSER(true, 20000, "FINISHED MDP state expansion :: "<<*current_state<<std::endl);
    }
    
                


    auto current_state = solver->peek__next_belief_state_for_expansion();
#ifdef LAO_STAR
    while(solver->lao_star()){};
    //             Planning::Policy_Iteration__GMRES policy_Iteration(solver->belief_state__space,
    //                                                                solver->get__sink_state_penalty(),
    //                                                                .95);
            
    //             while(policy_Iteration()){};

#else
    current_state = solver->solve__for_new_starting_state(current_state);
#endif


    int step_count = 0;
    bool needs_to_replan = false;
    for(auto i = 0; i < 20; i++){

                
        INTERACTIVE_VERBOSER(true, 19000, "Current state is :: "
                             <<*current_state<<std::endl
                             <<"First element is :: "
                             <<*dynamic_cast<const Planning::State*>(current_state->get__belief_state().back().first)<<std::endl);
                
        std::pair<Planning::Formula::Action_Proposition, uint> _action
            = solver->get_prescribed_action(current_state);
            
        INTERACTIVE_VERBOSER(true, 19000, "Prescribed action :: "<<_action.first<<" "<<_action.second<<std::endl);
            
        auto observations = current_state->get__possible_observations_given_action(_action.second);

        if(1 == observations.size()){
            needs_to_replan = needs_to_replan|false;
        } else {
            needs_to_replan = true;
        }
                
        auto random_index = 0;
        for(auto observation = observations.begin()
                ; observation != observations.end()
                ; observation++){
            INTERACTIVE_VERBOSER(true, 19000, "Observation :: "<<*observations[random_index]<<" "<<_action.second<<std::endl);
            
            {char ch; std::cin>>ch; if(ch == 'y')break;}
            random_index++;
        }
                
                
        //auto random_index = random() % observations.size();
        auto observation = observations[random_index];
            
        Planning::POMDP_State* successor_state
            = solver->take_observation(current_state,
                                       observation,
                                       _action.second);
            
        current_state = successor_state;

        /*TWO DAYS TO GO -- Do not replan unless there was sensing.*/
        if(observations.size() > 1){//;°needs_to_replan){
                    
            step_count++;

            if(!(step_count % min_nonobs_steps)){
                current_state = solver->solve__for_new_starting_state(successor_state);
            }
        }
                

                
        INTERACTIVE_VERBOSER(true, 19000, "Current belief state is :: "<<*current_state<<std::endl);
    }

        
    delete solver;
    
    INTERACTIVE_VERBOSER(true, 10004, "Passed test2 :: "<<std::endl);
    
    return 0;
}

/*
  
./pcogx --domain /data/private/grettonc/CogX/SVN/cogx/code/systems/bleeding-edge/subarchitectures/planner.sa/domains/icaps11/dora/problem01-domain.dtpddl --problem /data/private/grettonc/CogX/SVN/cogx/code/systems/bleeding-edge/subarchitectures/planner.sa/domains/icaps11/dora/problem01-problem-smaller.dtpddl  --max-iStates 1000 --steps 3 flatten

*/

int flatten()
{
    
    while(read_in__domain_description()){};
    
    while(read_in__problem_description()){};
    

    auto problem = Planning::Parsing::problems.begin();

    
    auto solver = new Planning::Flatten(*problem->second);
    
    solver->preprocess();

    
    solver->empty__belief_states_for_expansion();
    solver->generate_markov_decision_process_starting_states();

    
    auto current_state = solver->peek__next_belief_state_for_expansion();
    
    while(solver->expand_belief_state_space()){    
        VERBOSER(19000, "MDP state expansion :: "<<solver->belief_state__space.size()<<std::endl);
    }
    
    
    solver->print_flat_problem();
    
    
    return 0;
}

/*Charles' testing MAIN.*/
int main(int argc, char** argv)
{    
//     /*Some preliminary testing.*/
//     Turnstyle::test__turnstyle_hh();//turnstyle.hh
    
    assert(command_Line_Arguments.size() == 0);
    command_Line_Arguments = Command_Line_Arguments(argc, argv);

    
    if(!command_Line_Arguments.got_guard("--steps")){
        WARNING("Using --steps "<<min_nonobs_steps<<std::endl);
    } else {
        min_nonobs_steps = command_Line_Arguments.get_int();
    }
    
    
    if(!command_Line_Arguments.got_guard("--max-iStates")){
        WARNING("Using --max-iStates "<<max_expanded_states<<std::endl);
    } else {
        max_expanded_states = command_Line_Arguments.get_int();
    }
    
    if(!command_Line_Arguments.got_guard("--beta")){
        WARNING("Using --beta "<<beta<<std::endl);
    } else {
       beta = command_Line_Arguments.get_double();
    }
    
    if(!command_Line_Arguments.got_guard("--beta-mdp")){
        WARNING("Using --beta-mdp "<<beta_mdp<<std::endl);
    } else {
       beta_mdp = command_Line_Arguments.get_double();
    }
    
    
    int seed = 2010;
    srandom(seed);
    srand(seed);

    if(command_Line_Arguments.is_argument("flatten")){
        return flatten();
    }

    
    while(read_in__domain_description()){};
    
    while(read_in__problem_description()){};
    
    assert(Planning::Parsing::domains.size() > 0);

//     for(auto domain = Planning::Parsing::domains.begin()
//             ; domain != Planning::Parsing::domains.end()
//             ; domain++){
//         std::cout<<*domain->second<<std::endl;
//     }
    
    /*Testing exposure Wed Aug 25 15:41:33 BST 2010 ---*/
    
    for(auto problem = Planning::Parsing::problems.begin()
            ; problem != Planning::Parsing::problems.end()
            ; problem++){
            
        std::cout<<*problem->second->get__domain_Data()<<std::endl;
        std::cout<<*problem->second<<std::endl;
        
        for(int i =0 ; i < 100; i++){
            std::cout<<problem->second->get__prescribed_action()<<std::endl;
        }
    }
    
    INTERACTIVE_VERBOSER(true, 12000, "Passed test 1 :: "<<std::endl);

        
    uint count_test2 = 0;
        
        for(auto problem = Planning::Parsing::problems.begin()
                ; problem != Planning::Parsing::problems.end()
                ; problem++){
//             auto solver = new Planning::Simple_Online_Solver(*problem->second);//Planning::Solver*
//             auto solver = new Planning::Simple_Online_Solver(*problem->second);//Planning::Solver*
#ifdef LAO_STAR
#define CHANGE_PHASE 1
            auto solver = new Planning::Two_Phase_Solver(*problem->second);
            
//             auto solver = new Planning::Solver(*problem->second);
#else       
            auto solver = new Planning::Solver(*problem->second);
#endif
            
            solver->set__sink_state_penalty(-1.0); //-1234.0);//-1e6);
            
            INTERACTIVE_VERBOSER(true, 12000, "Made a solver, starting preprocessing.")
            
            solver->preprocess();

#ifdef LAO_STAR
            if(true){/*START -- TRYING THE MDP HEURISTIC.*/

                
                solver->empty__belief_states_for_expansion();
                solver->generate_markov_decision_process_starting_states();

                Planning::Policy_Iteration__GMRES policy_Iteration__for_MDP_states
                    (solver->belief_state__space, solver->get__sink_state_penalty(), .65);

                auto current_state = solver->peek__next_belief_state_for_expansion();
                
                INTERACTIVE_VERBOSER(true, 18000, "MDP state expansion :: "<<*current_state<<std::endl);
                

                while(solver->expand_belief_state_space()){
                    
                    VERBOSER(19000, "MDP state expansion :: "<<solver->belief_state__space.size()<<std::endl);
                }

                
                while(policy_Iteration__for_MDP_states()){
                    VERBOSER(18000, "Policy iteration."<<std::endl);
                }
                
                for(auto mdp_state = solver->belief_state__space.begin()
                        ; mdp_state != solver->belief_state__space.end()
                        ; mdp_state++){

                    if(*mdp_state == solver->get__starting_belief_state()){
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

                solver->reset__pomdp_state_hash_table();

#ifdef CHANGE_PHASE
                solver->change_phase();//HERE -- MONDAY
#endif
                
                solver->reinstate__starting_belief_state();
            }/*STOP -- TRYING THE MDP HEURISTIC.*/
#endif
            
#ifdef CHANGE_PHASE
            INTERACTIVE_VERBOSER(true, 19000, "Done MDP state expansion :: "<<std::endl);
#endif
                


            auto current_state = solver->peek__next_belief_state_for_expansion();
#ifdef LAO_STAR
            while(solver->lao_star()){};
//             Planning::Policy_Iteration__GMRES policy_Iteration(solver->belief_state__space,
//                                                                solver->get__sink_state_penalty(),
//                                                                .95);
            
//             while(policy_Iteration()){};

#else
            current_state = solver->solve__for_new_starting_state(current_state);
#endif


            int step_count = 0;
            bool needs_to_replan = false;
            for(auto i = 0; i < 20; i++){

                
                INTERACTIVE_VERBOSER(true, 19000, "Current state is :: "
                                     <<*current_state<<std::endl
                                     <<"First element is :: "
                                     <<*dynamic_cast<const Planning::State*>(current_state->get__belief_state().back().first)<<std::endl);
                
                std::pair<Planning::Formula::Action_Proposition, uint> _action
                    = solver->get_prescribed_action(current_state);
            
                INTERACTIVE_VERBOSER(true, 19000, "Prescribed action :: "<<_action.first<<" "<<_action.second<<std::endl);
            
                auto observations = current_state->get__possible_observations_given_action(_action.second);

                if(1 == observations.size()){
                    needs_to_replan = needs_to_replan|false;
                } else {
                    needs_to_replan = true;
                }
                
                auto random_index = 0;
                for(auto observation = observations.begin()
                        ; observation != observations.end()
                        ; observation++){
                    INTERACTIVE_VERBOSER(true, 19000, "Observation :: "<<*observations[random_index]<<" "<<_action.second<<std::endl);
            
                    {char ch; std::cin>>ch; if(ch == 'y')break;}
                    random_index++;
                }
                
                
                //auto random_index = random() % observations.size();
                auto observation = observations[random_index];
            
                Planning::POMDP_State* successor_state
                    = solver->take_observation(current_state,
                                               observation,
                                               _action.second);
            
                current_state = successor_state;

                /*TWO DAYS TO GO -- Do not replan unless there was sensing.*/
                if(observations.size() > 1){//;°needs_to_replan){
                    
                    step_count++;

                    if(!(step_count % min_nonobs_steps)){
                        current_state = solver->solve__for_new_starting_state(successor_state);
                    }
                }
                

                
                INTERACTIVE_VERBOSER(true, 19000, "Current belief state is :: "<<*current_state<<std::endl);
            }

        
            delete solver;
        }
    
    INTERACTIVE_VERBOSER(true, 10004, "Passed test2 :: "<<std::endl);
    
    return 0;
}

#endif

/* 
 *   With the organic tactility, refreshing scent and minimalist
 *   aesthetics of bamboo lend the ASUS Bamboo Series notebook an
 *   arresting aura of spirituality, warmth and old world charm that
 *   synthetic materials and cold, impersonal metals will struggle to
 *   replicate. With every touch, users will be able to feel the
 *   difference - the bamboo gives an instant sense of familiarity, just
 *   like the sensation one would get from running one's fingertips
 *   across furniture. The air of individuality of each piece can be
 *   further enhanced by several treatments that yield different colours,
 *   or by laser etching distinctive designs onto the ASUS Bamboo Series
 *   notebook's bamboo-clad cover. ASUS has achieved international renown
 *   for its research into, and inspired use of, biodegradable materials
 *   such as leather in its products, but its decision to embrace bamboo
 *   is nothing short of ingenious.
 * 
 *   Through the use of bamboo which has an immense tensile strength that
 *   rivals that of many metal alloys, the ASUS Bamboo notebook is highly
 *   resilient - an attribute proven conclusively by the fact that it is
 *   the first notebook to have survived the unforgiving conditions of
 *   snow-capped Qomolangma Peak, which stands at a staggering height of
 *   8,848 meters (29,028 feet). Bamboo also has a renewal rate that no
 *   other plant can match. It has been known to grow 60cm in just 24
 *   hours, reaching its maximum height in several years. Bamboo is also
 *   capable of regenerating itself upon harvesting without necessitating
 *   replanting, making it possibly the perfect renewable resource. The
 *   crux of the message borne by the ASUS Bamboo Series notebook is that
 *   "it's easy being green". Being green is a simple matter of making
 *   smart, environmentally-conscious purchasing decisions. Choosing the
 *   ASUS Bamboo Series notebook - or any of ASUS' notebooks, all of
 *   which were designed and manufactured in strict adherence to the same
 *   rigorous green policies and standards that governed the development
 *   of the ASUS Bamboo Series notebook - over less green alternatives,
 *   will help to preserve the Earth in no small measure.
 * 
 *    -- Sales pitch for the Laptop (Asus U6V-2P048C Mobility Notebook)
 *       chosen to control Pioneer robots used for the EC FP7-IST grant
 *       215181-CogX. Author is unknown.
 *   
 */

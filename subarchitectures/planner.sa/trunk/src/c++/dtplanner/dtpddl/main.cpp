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
#include "policy_iteration_over_information_state_space.hh"
#include "policy_iteration_over_information_state_space__GMRES.hh"

using std::endl;

/* WARNING :: C++ zero initialisation is not safe.*/
Command_Line_Arguments command_Line_Arguments;


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


int main(int argc, char** argv)
{    
    /*Some preliminary testing.*/
    Turnstyle::test__turnstyle_hh();//turnstyle.hh
    
    assert(command_Line_Arguments.size() == 0);
    command_Line_Arguments = Command_Line_Arguments(argc, argv);
    
    int seed = 2010;
    srandom(seed);
    srand(seed);

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
            auto solver = new Planning::Solver(*problem->second);//Planning::Solver*
            solver->set__sink_state_penalty(0);//-1e6);
            
            INTERACTIVE_VERBOSER(true, 12000, "Made a solver, starting preprocessing.")
            
            solver->preprocess();

            INTERACTIVE_VERBOSER(true, 12000, "Done preprocessing, now looking towards state expansion.");
            
            QUERY_UNRECOVERABLE_ERROR(0 == solver->peek__next_belief_state_for_expansion()
                                      , "There is no starting belief state.\n"
                                      <<"This might be because no rewards are available in the problem.");
            
            assert(0 != solver->peek__next_belief_state_for_expansion());//expansion_queue.size());
//             solver->expand_belief_state_space();
            auto current_state = solver->peek__next_belief_state_for_expansion();//expansion_queue.front();
            
            INTERACTIVE_VERBOSER(true, 15000, "Current state is :: "
                                 <<*current_state<<std::endl
                                 <<*dynamic_cast<const Planning::State*>(current_state->get__belief_state().back().first)<<std::endl);

            
//             Planning::Policy_Iteration
            Planning::Policy_Iteration__GMRES policy_Iteration(solver->belief_state__space, solver->get__sink_state_penalty());
//             Planning::Policy_Iteration policy_Iteration(solver->belief_state__space, solver->get__sink_state_penalty());
            for(uint i = 0; i < 100; i++){
                if(!solver->expand_belief_state_space()){
                    break;
                    VERBOSER(15000, "No starting state!"<<std::endl);
                } else {
                    VERBOSER(15000, "Expanding (so far we have "
                             <<solver->belief_state__space.size()<<" beliefs)!"<<std::endl
                             <<"Expected reward is :: "
                             <<current_state->get__expected_value()<<std::endl);
                
//                     policy_Iteration();
                }
            }

//             exit(0);
            
//             double old_expected_reward = -1000000.0;
            
            for(uint i = 0; i < 500; i++){
                policy_Iteration();
//                 if(old_expected_reward != current_state->get__expected_value()){
//                      old_expected_reward = current_state->get__expected_value();
//                 } else {
//                     break;
//                 }
                
                
                INTERACTIVE_VERBOSER(true, 15000, "Expected reward is :: "
                         <<current_state->get__expected_value()<<std::endl);
            }
            

//             Planning::Policy_Iteration policy_Iteration(solver->belief_state__space);
//             policy_Iteration();
            
            for(auto i = 0; i < 10; i++){
            
                INTERACTIVE_VERBOSER(true, 15000, "Current state is :: "
                                     <<*current_state<<std::endl
                                     <<"First element is :: "
                                     <<*dynamic_cast<const Planning::State*>(current_state->get__belief_state().back().first)<<std::endl);
                
                std::pair<Planning::Formula::Action_Proposition, uint> _action
                    = solver->get_prescribed_action(current_state);
            
                INTERACTIVE_VERBOSER(true, 15000, "Prescribed action :: "<<_action.first<<" "<<_action.second<<std::endl);
            
                auto observations = current_state->get__possible_observations_given_action(_action.second);
            
                auto random_index = random() % observations.size();
                auto observation = observations[random_index];
            
                Planning::POMDP_State* successor_state
                    = solver->take_observation(current_state,
                                               observation,
                                               _action.second);
            
                current_state = successor_state;
            
                INTERACTIVE_VERBOSER(true, 15000, "Current belief state is :: "<<*current_state<<std::endl);
            }

        
//             delete solver;
        }
    
    INTERACTIVE_VERBOSER(true, 10004, "Passed test2 :: "<<std::endl);
    
    return 0;
}

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

 
#include "global.hh"
#include "Command_Line_Arguments.hh"
#include "dtp_pddl_parsing_interface.hh"
#include "dtp_pddl_parsing_data.hh"
#include "dtp_pddl_parsing_data_problem.hh"



#include "turnstyle.hh"

#include "solver.hh"
#include "policy_iteration_over_information_state_space.hh"

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
    
    INTERACTIVE_VERBOSER(true, 10010, "Passed test 1 :: "<<std::endl);

    uint count_test2 = 0;
//     while(true){
        
        for(auto problem = Planning::Parsing::problems.begin()
                ; problem != Planning::Parsing::problems.end()
                ; problem++){
            Planning::Solver* solver = new Planning::Solver(*problem->second);
            solver->preprocess();
            solver->expand_belief_state_space();
            auto current_state = solver->expansion_queue.front();
            if(!solver->expand_belief_state_space()){
                UNRECOVERABLE_ERROR("No starting state!"<<std::endl);
            }
        
            for(auto i = 0; i < 10; i++){
            
                std::pair<Planning::Formula::Action_Proposition, uint> _action
                    = solver->get_prescribed_action(current_state);
            
                INTERACTIVE_VERBOSER(true, 10015, "Prescribed action :: "<<_action.first<<" "<<_action.second<<std::endl);
            
                auto observations = current_state->get__possible_observations_given_action(_action.second);
            
                auto random_index = random() % observations.size();
                auto observation = observations[random_index];
            
                Planning::POMDP_State* successor_state
                    = solver->take_observation(current_state,
                                               observation,
                                               _action.second);
            
                current_state = successor_state;
            
                INTERACTIVE_VERBOSER(true, 10002, "Current belief state is :: "<<*current_state<<std::endl);
            }

        
//             delete solver;
        }
    
        INTERACTIVE_VERBOSER(true, 10004, "Passed test 2 :: "<<++count_test2<<std::endl);
  //   }
    
    
//     for(auto problem = Planning::Parsing::problems.begin()
//             ; problem != Planning::Parsing::problems.end()
//             ; problem++){
//         Planning::Solver* solver = new Planning::Solver(*problem->second);
//         solver->preprocess();
//         solver->expand_belief_state_space();
//         auto current_state = solver->expansion_queue.front();
//         if(!solver->expand_belief_state_space()){
//             UNRECOVERABLE_ERROR("No starting state!"<<std::endl);
//         }
        
//         for(auto i = 0; i < 10; i++){
            
//             std::pair<Planning::Formula::Action_Proposition, uint> _action
//                 = solver->get_prescribed_action(current_state);
            
//             INTERACTIVE_VERBOSER(true, 10002, "Prescribed action :: "<<_action.first<<" "<<_action.second<<std::endl);
            
//             auto observations = current_state->get__possible_observations_given_action(_action.second);
            
//             auto random_index = random() % observations.size();
//             auto observation = observations[random_index];
            
//             Planning::POMDP_State* successor_state
//                 = solver->take_observation(current_state,
//                                            observation,
//                                            _action.second);
            
//             current_state = successor_state;
            
//             INTERACTIVE_VERBOSER(true, 10002, "Current belief state is :: "<<*current_state<<std::endl);
//         }

        
//         delete solver;
//     }
    
    INTERACTIVE_VERBOSER(true, 10004, "Passed test2 :: "<<std::endl);
    
//     for(auto problem = Planning::Parsing::problems.begin()
//             ; problem != Planning::Parsing::problems.end()
//             ; problem++){
        
//         INTERACTIVE_VERBOSER(true, 10004, "Making new solver :: "<<std::endl);
//         Planning::Solver* solver = new Planning::Solver(*problem->second);
//         INTERACTIVE_VERBOSER(true, 10004, "Made new solver, now preprocessing :: "<<std::endl);
        
//         solver->preprocess();
//         INTERACTIVE_VERBOSER(true, 10004, "Done preprocessing, now expanding belief space :: "<<std::endl);
//         solver->expand_belief_state_space();
//         INTERACTIVE_VERBOSER(true, 10004, "Done belief expansion, now running PI :: "<<std::endl);
        
//         Planning::Policy_Iteration policy_Iteration(solver->belief_state__space);
//         while(solver->expand_belief_state_space()){
//             INTERACTIVE_VERBOSER(true, 10004, "Expanded belief for state count :: "
//                                  <<solver->belief_state__space.size()<<std::endl);
//             policy_Iteration();
//         }
        
//         delete solver;
//     }

    
//     for(auto problem = Planning::Parsing::problems.begin()
//             ; problem != Planning::Parsing::problems.end()
//             ; problem++){
        
//         std::cout<<*problem->second->get__domain_Data()<<std::endl;
//         std::cout<<*problem->second<<std::endl;
        
//         for(int i =0 ; i < 100; i++){
//             std::cout<<problem->second->get__prescribed_action()<<std::endl;
//         }
        
//         Planning::Solver solver(*problem->second);
//         solver.preprocess();
//         while(solver.expand_belief_state_space()){
//             INTERACTIVE_VERBOSER(true, 9096, "Expanding POMDP state"<<std::endl);
//         };
        
//         std::cout<<*problem->second->get__domain_Data()<<std::endl;
//         std::cout<<*problem->second<<std::endl;
//     }


    
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

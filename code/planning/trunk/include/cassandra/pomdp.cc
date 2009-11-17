
#include "utilities.hh"

/*---------------------------------

Usual suspects (C++) Linear Algebra and Linear Programming

---------------------------------*/
#include <glpk.h>

#include <ClpSimplex.hpp>
/*---------------------------------*/


#include "cassandra_POMDP__parser.hh"
#include "Command_Line_Arguments.hh"

#include "pomdp__finite_state_controller__evaluation.hh"
#include "pomdp__finite_state_controller__policy_improvement.hh"

using std::endl;
using std::count;
using std::cin;



int main(int argc, char** argv)
{
    Command_Line_Arguments command_Line_Arguments;
    command_Line_Arguments = Command_Line_Arguments(argc, argv);
    
    
    std::string problem_file_name = "problem.pomdp";
    if(!(command_Line_Arguments.got_guard("--problem"))){
        WARNING("missing argument ::"<<endl
                <<"--problem filename"<<endl
                <<"Using :: "<<problem_file_name<<endl);
    } else {
        problem_file_name = command_Line_Arguments.get_string();
        VERBOSER(1, "Using file :: "
                 <<problem_file_name<<" :: for problem definition."); 
    }

    
    POMDP::Parsing::parse_Cassandra_POMDP_problem(problem_file_name);
    
    VERBOSER(100, "Just parsed the following problem \n");
    VERBOSER(100, *POMDP::Parsing::problem_Data);

    
    VERBOSER(200, "Testing problem sanity \n");
    assert(POMDP::Parsing::problem_Data->sanity__transition_model__has_non_zero_entries());
    assert(POMDP::Parsing::problem_Data->sanity__transition_model());
    assert(POMDP::Parsing::problem_Data->sanity__observation_model__has_non_zero_entries());
    assert(POMDP::Parsing::problem_Data->sanity__observation_model());
    VERBOSER(200, "Tested problem sanity \n");
    
    const int number_of_controller_nodes = 4;
    POMDP::FSC fsc(POMDP::Parsing::problem_Data, number_of_controller_nodes);

    POMDP::FSC__Randomizer fsc__Randomizer;
    fsc__Randomizer(fsc);

    assert(fsc.sanity__action_execution_probabilities());
    assert(fsc.sanity__node_transition_probabilities());
    
    VERBOSER(200, "Generated a randomised finite-state-controller :: "<<std::endl);
    VERBOSER(200, fsc<<std::endl);
        
    POMDP::Solving::FSC__Evaluator fsc__Evaluator(fsc);

    /* Evaluate the randomised controller -- i.e., compute the value
     * of being in a state at a given controller node */
    fsc__Evaluator();

    VERBOSER(200, fsc__Evaluator<<std::endl);

    
    for(auto node_index = 0; node_index < fsc.get__nodes_count(); node_index++){
        POMDP::Solving::FSC__Node_Improvement fsc__Node_Improvement(node_index, fsc, fsc__Evaluator);
        auto improvement_was_possible = fsc__Node_Improvement();
        
        if(improvement_was_possible){
            VERBOSER(200, "Node :: "<<node_index<<" was improved."<<std::endl);
            
            VERBOSER(200, "The new controller looks like :: "<<std::endl);
            VERBOSER(200, fsc<<std::endl);
            
            
            
        } else {
            VERBOSER(200, "Node :: "<<node_index<<" could not be improved."<<std::endl);
            VERBOSER(200, fsc<<std::endl);
        }
        
    }
    
    POMDP::Solving::FSC__Evaluator fsc__Evaluator_2(fsc);

    /* Evaluate the randomised controller -- i.e., compute the value
     * of being in a state at a given controller node */
    fsc__Evaluator_2();

    VERBOSER(200, "Second evaluator :: \n"<<fsc__Evaluator_2<<std::endl);

//     POMDP::Solving::FSC__Node_Improvement fsc__Node_Improvement(node_index, fsc, fsc__Evaluator);
//     auto improvement_was_possible = fsc__Node_Improvement();

    
    
    
    return 0;
}

/* apoptosis: (biology) cell death.*/


#include "utilities.hh"

/*---------------------------------

Usual suspects (C++) Linear Algebra and Linear Programming

---------------------------------*/
#include <glpk.h>

#include <ClpSimplex.hpp>
/*---------------------------------*/


#include "cassandra_POMDP__parser.hh"
#include "Command_Line_Arguments.hh"

using std::endl;
using std::count;
using std::cin;


/* Function parses POMDP problem definitons expressed in the Anthony
 * R. Cassandra format.*/
int parse_cassandra_POMDP()
{
    
}

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

    std::cout<<*POMDP::Parsing::problem_Data;
    
    return 0;
}

/* apoptosis: (biology) cell death.*/

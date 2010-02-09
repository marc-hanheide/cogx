
#include "global.hh"
#include "Command_Line_Arguments.hh"
#include "dtp_pddl_parsing_interface.hh"

using std::endl;

/* WARNING :: C++ zero initialisation is not safe.*/
Command_Line_Arguments command_Line_Arguments;

int main(int argc, char** argv)
{
    assert(command_Line_Arguments.size() == 0);
    command_Line_Arguments = Command_Line_Arguments(argc, argv);
    
    int seed = 2010;
    srandom(seed);
    srand(seed);

    std::string domain_file_name = "domain.pddl";
    if(!(command_Line_Arguments.got_guard("--domain"))){
        WARNING("missing argument ::"<<endl
                <<"--domain filename"<<endl
                <<"Using :: "<<domain_file_name<<endl);
    } else {
        domain_file_name = command_Line_Arguments.get_string(); 
        VERBOSER(1, "Using file :: "
                 <<domain_file_name<<" :: for domain definition.");
    }
    
    std::string problem_file_name = "problem.pddl";
    if(!(command_Line_Arguments.got_guard("--problem"))){
        WARNING("missing argument ::"<<endl
                <<"--problem filename"<<endl
                <<"Using :: "<<problem_file_name<<endl);
    } else {
        problem_file_name = command_Line_Arguments.get_string();
        VERBOSER(1, "Using file :: "
                 <<problem_file_name<<" :: for problem definition."); 
    }
    
    Planning::Parsing::parse_domain(domain_file_name);
    Planning::Parsing::parse_problem(problem_file_name);
    
    return 0;
}

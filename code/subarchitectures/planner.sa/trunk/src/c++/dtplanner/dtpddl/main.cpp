
#include "global.hh"
#include "Command_Line_Arguments.hh"
#include "dtp_pddl_parsing_interface.hh"
#include "dtp_pddl_parsing_data.hh"


#include "turnstyle.hh"

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

    for(auto domain = Planning::Parsing::domains.begin()
            ; domain != Planning::Parsing::domains.end()
            ; domain++){
        std::cout<<*domain->second<<std::endl;
    }
    
    for(auto problem = Planning::Parsing::problems.begin()
            ; problem != Planning::Parsing::problems.end()
            ; problem++){
        std::cout<<*problem->second<<std::endl;
    }
    
    return 0;
}

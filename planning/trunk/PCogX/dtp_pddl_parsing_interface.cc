#include "dtp_pddl_parsing_interface.hh"


#include "dtp_pddl.hh"

using namespace pegtl;

using namespace Planning::Parsing;


void Planning::Parsing::parse_domain(const std::string& domain_file_name)
{
    Planning::Parsing::domain_Stack.reset(new Domain_Stack());
    
    pegtl::smart_parse_file< Planning::Parsing::PDDL_Preamble_Domain >
        ( true/*_trace*/, domain_file_name, *Planning::Parsing::domain_Stack );
    
    domains[domain_Stack->get__domain_name()] = Planning::Parsing::domain_Stack;
}

/* We assume a domain has already been parsed when we are given a
 * problem to parse. By default, the domain associated with the
 * problem is whatever is at
 * \global{Planning::Parsing::domain_stack}.*/
void Planning::Parsing::parse_problem(const std::string& problem_file_name,
                           std::shared_ptr<Domain_Stack>& domain_Stack)
{
    assert(domain_Stack.use_count());

    Planning::Parsing::problem_Stack.reset(new Problem_Stack(domain_Stack));
    
    pegtl::smart_parse_file< Planning::Parsing::PDDL_Preamble_Problem >
        ( true/*_trace*/, problem_file_name, *Planning::Parsing::problem_Stack );

    Problem_Data::Identifier identifier(problem_Stack->get__domain_Data()->get__domain_name(),
                             problem_Stack->get__problem_name());
    
    problems[identifier] = Planning::Parsing::problem_Stack;
}

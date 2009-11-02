#ifndef DTP_PDDL_PARSING_INTERFACE_HH
#define DTP_PDDL_PARSING_INTERFACE_HH

#include "dtp_pddl_parsing_data.hh"

namespace Planning
{
    namespace Parsing
    {
        /* Parses a problem definition occurring in a file with name
         * \argument{problem__file_name}. The result of the parse is
         * stored in \global{Planning::Parsing::problem_stack}. Unless
         * otherwise specified, the domain associated with the problem
         * is that at \global{Planning::Parsing::domains} (with the
         * specified name -- as parsed) and otherwise that at
         * \global{Planning::Parsing::problem_stack} (or otherwise
         * specified at \argument{shared_ptr<Domain_Stack>&}).
         *
         * Watch out, if you parse the same problem (wrt a particular
         * domain) twice using this function, then changes are not
         * merged. Rather the first parsed data is lost.*/ 
        void parse_problem(const std::string& problem_file_name,
                           std::shared_ptr<Domain_Stack>& = Planning::Parsing::domain_Stack);

        /* Parses a domain definition occurring in a file with name
         * \argument{domain__file_name}. The result of the parse is
         * stored in \global{Planning::Parsing::domain_stack}.
         *
         * Watch out, if you parse the same domain twice using this
         * function, then changes are not merged. Rather the first
         * parsed data is lost.*/
        void parse_domain(const std::string& domain__file_name);
    }
}


#endif

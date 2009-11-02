#ifndef CAST_BASED_INTERFACE_HH
#define CAST_BASED_INTERFACE_HH

#include "dtp_pddl_parsing_data.hh"

namespace Planning
{
    /* CAST is an acronym: CoSy Architecture Schema Toolkit. It is how
     * CoSy and CogX "stuff" (my own word) talks.*/
    namespace CAST_Interface
    {

        
        void report__STRIPS_action(char*, char**, char**, char**);
        void report__STRIPS_action(int, int*, int*, int*);
    }
}

#endif

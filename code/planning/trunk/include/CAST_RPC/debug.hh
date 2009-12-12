
/* Copyright (C) 2009
 * Charles Gretton (charles.gretton@gmail.com)
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
 * ABOUT :: 
 *
 * Error stream and CAST-based debugging macros.
 */

#ifndef DEBUG_HH
#define DEBUG_HH

#include "utilities.hh"

/*---------------------------------

Old fashioned debugging -- ERROR STREAM

---------------------------------*/
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 17
#endif

#ifndef DEBUG_LINE_WIDTH
#define DEBUG_LINE_WIDTH 66
#endif

#ifndef DEBUG_SLEEP_DELAY
#define DEBUG_SLEEP_DELAY 0
#endif

#define STANDARD_DEBUGGING_PREFIX                                       \
    " -- "<<__PRETTY_FUNCTION__<<" -- :: \n**\t"<<std::endl             \
        <<__FILE__<<std::endl                                           \
        <<__LINE__<<std::endl                                           \
            
#define DEBUG_SEPERATING_LINE(X, N)           \
    std::setw(N-1)                            \
        <<std::setfill(X)                     \
        <<X<<std::endl                        \

#define DEBUG_OUT(X)                                                    \
    {                                                                   \
        sleep(DEBUG_SLEEP_DELAY);                                       \
        std::ostringstream oss;                                         \
        std::ostringstream oss2;                                        \
        oss<<STANDARD_DEBUGGING_PREFIX<<X;                              \
        oss2<<std::setw(DEBUG_LINE_WIDTH)                               \
            <<std::left<<oss.str();                                     \
        std::cerr<<DEBUG_SEPERATING_LINE('-', DEBUG_LINE_WIDTH)         \
                 <<std::setfill('-')                                    \
                 <<std::endl                                            \
                 <<oss2.str().c_str()                                   \
                 <<std::endl                                            \
                 <<DEBUG_SEPERATING_LINE('-', DEBUG_LINE_WIDTH);        \
    }                                                                   \


#define VERBOSE(X)                                       \
    DEBUG_OUT(" ** INFO -- "<<std::endl                  \
              <<X);                                      \
        
#define VERBOSER(Y, X) {                                        \
        if(Y > DEBUG_LEVEL)                                     \
            VERBOSE(" -- DEBUG_LEVEL :: "<<Y<<" -- "<<std::endl \
                    <<X);                                       \
    }                                                           \
        

#define UNRECOVERABLE_ERROR(X) {                                 \
        DEBUG_OUT(" ** UNRECOVERABLE ERROR -- "<<std::endl       \
                  <<X);                                          \
        assert(0);                                               \
        exit(0);                                                 \
    }                                                            \

#define QUERY_UNRECOVERABLE_ERROR(Q,X) {if(Q)UNRECOVERABLE_ERROR(X);}

#ifndef RPC_WARNINGS
#define RPC_WARNINGS 1
#endif

#define WARNING(X) { if(RPC_WARNINGS){                            \
            DEBUG_OUT(" ** WARNING -- "<<std::endl                 \
                      <<X);                                        \
        }                                                          \
    }                                                              \
        

#define QUERY_WARNING(Q,X) {if(Q)WARNING(X);}

/*---------------------------------*/

/*---------------------------------

Old fashioned debugging -- CAST STREAM

---------------------------------*/

/* Macro that prints \argument{X} debug messages to the CAST
 * \class{cast::ManagedComponent} \member{.println()}.
 * 
 * FIX :: \program{valgrind} doesn't like CAST when \member{.c_str()}
 * is passed to \function{.println()}. Generally, I think it is a good
 * idea to keep C data away from CAST. */
#define CAST__DEBUG_OUT(X)                      \
    {                                           \
        sleep(DEBUG_SLEEP_DELAY);               \
        std::ostringstream oss;                 \
        oss<<STANDARD_DEBUGGING_PREFIX<<X;      \
        this->println(oss.str());               \
    }                                           \


/* ... and so on.*/

#define CAST__VERBOSE(X)                                           \
        CAST__DEBUG_OUT(" ** INFO -- "<<std::endl                  \
                        <<X);                                      \
        



        
#define CAST__VERBOSER(Y, X) {                                         \
        if(Y > DEBUG_LEVEL)                                            \
            CAST__VERBOSE(" -- DEBUG_LEVEL :: "<<Y<<" -- "<<std::endl  \
                          <<X);                                        \
    }                                                                  \
        
/*-------------------------------*/
/*---------------------------------

Macros for fatal and lower-level errors.

---------------------------------*/

#define CAST__UNRECOVERABLE_ERROR(X) {                           \
        CAST__DEBUG_OUT(" ** UNRECOVERABLE ERROR -- "<<std::endl \
                        <<X);                                    \
        assert(0);                                               \
        exit(0);                                                 \
    }                                                            \

#define CAST__QUERY_UNRECOVERABLE_ERROR(Q,X) {if(Q)CAST__UNRECOVERABLE_ERROR(X);}

#define CAST__WARNING(X) {                                       \
        CAST__DEBUG_OUT(" ** WARNING -- "<<std::endl             \
                        <<X);                                    \
    }                                                            \


#define CAST__QUERY_WARNING(Q,X) {if(Q)CAST__WARNING(X);}


/*-------------------------------*/


/* Gives the \class{std::string} representation of the argument
 * type.*/
inline std::string _DEMANGLE_TYPE(decltype(typeid(NA)) in)
{
    int status;
    auto realname = abi::__cxa_demangle(in.name(), 0, 0, &status);
    std::ostringstream oss;
    oss<<realname<<" -s- "<<status;
    
    return std::move(oss.str());
}

#define DEMANGLE_TYPE(X) _DEMANGLE_TYPE(typeid(X))


#endif

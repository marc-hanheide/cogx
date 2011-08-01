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
 * CogX ::
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

#ifndef DEBUG_HH
#define DEBUG_HH

#include "utilities.hh"

/*---------------------------------

Old fashioned debugging -- ERROR STREAM

---------------------------------*/
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 18999
#endif

#ifndef DEBUG_LINE_WIDTH
#define DEBUG_LINE_WIDTH 66
#endif

#ifndef DEBUG_SLEEP_DELAY
#define DEBUG_SLEEP_DELAY 0
#endif

#ifndef PRINTING_WITH_THREAD_INTEGER
#define PRINTING_WITH_THREAD_INTEGER 0
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
        
#define INTERACTIVE_VERBOSER(Z, Y, X)           \
    {                                           \
        VERBOSER(Y, X)                          \
            if(Z && (Y > DEBUG_LEVEL)){         \
                char ch;                        \
                std::cin>>ch;                   \
            }                                   \
    }                                           \
        
        

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

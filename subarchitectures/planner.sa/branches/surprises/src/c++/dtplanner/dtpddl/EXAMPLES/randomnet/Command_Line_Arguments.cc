
/*---------------------------------

Usual suspects C

---------------------------------*/
#include<cstddef>
#include<cstdio>
#include<cmath>
#include<cstdlib>
//#include<cmalloc>
#include<cassert>
#include<cctype>
#include<csignal>
#include<cstdarg>
#include<cstddef>
#include<cstring>
/*---------------------------------*/

/*---------------------------------

Usual suspects C++::1998

---------------------------------*/
#include <string>
#include <vector>
#include <map>
#include <algorithm>
#include <set>
#include <stack>
#include <queue>
#include <list>

/* Streaming...*/
#include <iostream>
#include <iomanip> /*setw, etc.*/
#include <sstream> /*string-stream*/



/* (also, see \module{tr1/type_traits} ).*/
#include <cxxabi.h>
/*---------------------------------*/

/*---------------------------------

Usual suspects C++:: c++-0x

---------------------------------*/

/* The kind folks at the GCC team have put all the TR1 and C++-0X
 * stuff in tr1. Some of it is also off-tr1, but I prefer to note
 * where features are not entirely concrete...*/
#include <tr1/tuple>
#include <tr1/unordered_map>
#include <tr1/unordered_set>
#include <tr1/memory>

/* Functionality for bringing (static -- i.e., compile time) type
 * information into the running system.*/
#include <tr1/type_traits>

/*---------------------------------*/


/*---------------------------------

Usual suspects (C++) BOOST

---------------------------------*/
#include<boost/functional/hash.hpp>
/*---------------------------------*/


#include "Command_Line_Arguments.hh"

using std::istringstream;
using std::string;
using std::map;

Command_Line_Arguments::Command_Line_Arguments(Command_Line_Arguments&& command_Line_Arguments)
    :arguments(std::move(command_Line_Arguments.arguments)),
     assignments(std::move(command_Line_Arguments.assignments)),
     lastGuardTest(std::move(command_Line_Arguments.lastGuardTest))
{
}

Command_Line_Arguments&
Command_Line_Arguments::operator=(Command_Line_Arguments&& command_Line_Arguments)
{
    assignments = std::move(command_Line_Arguments.assignments);
    arguments = std::move(command_Line_Arguments.arguments);
    lastGuardTest = std::move(command_Line_Arguments.lastGuardTest);

    return *this;
}

Command_Line_Arguments::Command_Line_Arguments(int argc, char** argv)
    :lastGuardTest("")
{
    for(int i = 0; i < argc; i++){
        bool gotMap = false;
        string str = string(argv[i]);
        
        if(str.size() > 2){
            if(str[1] == '-' && str[0] == '-'){
                gotMap = true;
            }
        }

        if(str.size() > 1){
            if(str[0] == '-'){
                gotMap = true;
            }
        }

        arguments.push_back(std::move(argv[i]));

        if(gotMap){
            assert((i + 1) < argc);

            i++;
            arguments.push_back(std::move(argv[i]));
            
//             string str = std::move(argv[i-1]);
//             assignments[std::move(str)] = std::move(argv[i]);
            assignments[arguments[arguments.size() - 2]]
                = arguments[arguments.size() - 1];
        }
    }
}

Command_Line_Arguments::~Command_Line_Arguments()
{
}

bool Command_Line_Arguments::got_guard(string&& str)
{
    assert(str != "");
    lastGuardTest = std::move(str);
    
    return (assignments.find(str) != assignments.end());
}

template<typename X>
X get_thing(const string& s,
           const map<string, string>& assignments,
           const string& lastGuardTest)
{
    istringstream iss(assignments.find((s!="")?s:lastGuardTest)->second);

    X i;
    
    iss>>i;
    
    return i; 
}



uint Command_Line_Arguments::size() const {return arguments.size();}

int Command_Line_Arguments::get_int(const string& str) const
{
    return get_thing<int>(str, assignments, lastGuardTest);
}

double Command_Line_Arguments::get_double(const string& str) const
{
    return get_thing<double>(str, assignments, lastGuardTest);
}

bool Command_Line_Arguments::get_bool(const string& str) const
{
    return get_thing<bool>(str, assignments, lastGuardTest);
}

string Command_Line_Arguments::get_string(const string& str) const
{
    return get_thing<string>(str, assignments, lastGuardTest);
}

bool Command_Line_Arguments::is_argument(const string& str) const
{
    return (find(arguments.begin(), arguments.end(), str) != arguments.end());
}

bool Command_Line_Arguments::is_not_mapped_to(const string& str) const
{
    return (is_argument(str))
        && (assignments.find(str) == assignments.end());
}

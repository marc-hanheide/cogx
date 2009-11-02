// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "Command_Line_Arguments.hh"

using std::istringstream;
using std::string;
using std::map;

Command_Line_Arguments::Command_Line_Arguments(Command_Line_Arguments&& command_Line_Arguments)
    :assignments(std::move(command_Line_Arguments.assignments)),
     arguments(std::move(command_Line_Arguments.arguments)),
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

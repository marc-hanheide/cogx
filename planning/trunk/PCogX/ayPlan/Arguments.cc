// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#include "Arguments.hh"

using namespace std;

Arguments::Arguments(int argc, char** argv)
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

        arguments.push_back(argv[i]);

        if(gotMap){
            assert((i + 1) < argc);

            i++;
            arguments.push_back(argv[i]);
            
            string str = argv[i-1];
            assignments[str] = argv[i];
        }
    }
}

Arguments::~Arguments()
{
}

bool Arguments::gotGuard(const string& str)
{
    assert(str != "");
    lastGuardTest = str;
    
    return (assignments.find(str) != assignments.end());
}

template<typename X>
X getThing(const string& s,
           const map<string, string>& assignments,
           const string& lastGuardTest)
{
    string str;
    
    if(s == ""){
        assert(lastGuardTest != "");
        
        str = lastGuardTest;
    } else {
        str = s;
    }
    
    istringstream iss(assignments.find(str)->second);

    X i;
    
    iss>>i;
    
    return i; 
}


int Arguments::getInt(const string& str) const
{
    return getThing<int>(str, assignments, lastGuardTest);
}

double Arguments::getDouble(const string& str) const
{
    return getThing<double>(str, assignments, lastGuardTest);
}

bool Arguments::getBool(const string& str) const
{
    return getThing<bool>(str, assignments, lastGuardTest);
}

string Arguments::getString(const string& str) const
{
    return getThing<string>(str, assignments, lastGuardTest);
}

bool Arguments::isArgument(const string& str) const
{
    return (find(arguments.begin(), arguments.end(), str) != arguments.end());
}

bool Arguments::isNMap(const string& str) const
{
    return (isArgument(str))
        && (assignments.find(str) == assignments.end());
}

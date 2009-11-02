// Copyright (c) 2008 National ICT Australia Limited (NICTA)
//
// Author: Charles Gretton (charles.gretton@nicta.com.au)
//
// Redistribution and use in source and binary forms, with or without
// modification and only for non-commercial research and educational
// purposes are permitted provided that the conditions in the
// accompanying file "licence.txt" are met.

#ifndef ARGUMENTS_HH
#define ARGUMENTS_HH

#include"global.hh"

class Arguments
{
public:
    Arguments(int, char**);
    virtual ~Arguments();

    /*Is there an argument in \member{arguments} guarded by the
     * argument \class{std::string}. This is not constant because the
     * argument is cached in the \member{lastGuardTest}.*/
    bool gotGuard(const std::string&);
    
    /*Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{int}.*/
    int getInt(const std::string& = "") const;
    
    /*Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{double}.*/
    double getDouble(const std::string& = "") const;
    
    /*Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{bool}.*/
    bool getBool(const std::string& = "") const;
    
    /*Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{std::string}.*/
    std::string getString(const std::string& = "") const;

    /*Get the \argument{int} argument.*/
    std::string getString(int) const;
    
    /*Is non-map? Does the argument std::string appear in
     * \member{arguments}, and also, does it not appear in the domain
     * of the map \member{assignments}.*/
    bool isNMap(const std::string&) const;

    /*Does the argument appear in \member{Arguments}.*/
    bool isArgument(const std::string&) const;
protected:
    std::vector<std::string> arguments;
    std::map<std::string, std::string> assignments;

    /*Argument to \function{gotGuard}.*/
    std::string lastGuardTest;
    
};

#endif

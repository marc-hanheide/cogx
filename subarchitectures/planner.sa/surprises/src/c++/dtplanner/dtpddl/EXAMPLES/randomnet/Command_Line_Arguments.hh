
#include <iostream>
#include <fstream>
#include <sstream>
#include <cassert>
#include <map>
#include <vector>
#include <set>
#include <string>


#ifndef ARGUMENTS_HH
#define ARGUMENTS_HH


using namespace std;

class Command_Line_Arguments
{
public:
    
    explicit Command_Line_Arguments(int, char**);
    explicit Command_Line_Arguments(const Command_Line_Arguments&) = delete;
    explicit Command_Line_Arguments(Command_Line_Arguments&) = delete;
    explicit Command_Line_Arguments(Command_Line_Arguments&&);
    Command_Line_Arguments(){};

    Command_Line_Arguments& operator=(Command_Line_Arguments&&);
    
    virtual ~Command_Line_Arguments();

    /* (see \member{arguments}.size() )*/
    uint size()const;//{return arguments.size();};
    
    /* Is there an argument in \member{arguments} guarded by the
     * argument \class{std::string}. This is not constant because the
     * argument is cached in the \member{lastGuardTest}.*/
    bool got_guard(std::string&&);
    
    /* Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{int}.*/
    int get_int(const std::string& = "") const;
    
    /* Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{double}.*/
    double get_double(const std::string& = "") const;
    
    /* Return the argument of the last queried std::string (via
     * \function{got_guard}) as an \type{bool}.*/
    bool get_bool(const std::string& = "") const;
    
    /* Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{std::string}.*/
    std::string get_string(const std::string& = "") const;

    /* Get the \argument{int} argument.*/
    std::string get_string(int) const;
    
    /* Is non-map? Does the argument std::string appear in
     * \member{arguments}, and also, does it not appear in the domain
     * of the map \member{assignments}.*/
    bool is_not_mapped_to(const std::string&) const;

    /* Does the \argument{string} appear in \member{arguments}.*/
    bool is_argument(const std::string&) const;
protected:
    /* List of arguments passed to the application*/
    std::vector<std::string> arguments;

    /* Assignments that can be inferred from the argument list in
     * \member{arguments}.*/
    std::map<std::string, std::string> assignments;

    /* Argument to \function{got_guard}.*/
    mutable std::string lastGuardTest;
};

#endif


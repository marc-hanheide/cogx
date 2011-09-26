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


#ifndef ARGUMENTS_HH
#define ARGUMENTS_HH

#include"global.hh"

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
    bool got_guard(const std::string&);
    
    /* Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{int}.*/
    int get_int(const std::string& = "") const;
    
    /* Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{double}.*/
    double get_double(const std::string& = "") const;
    
    /* Return the argument of the last queried std::string (via
     * \function{gotGuard}) as an \type{bool}.*/
    bool get_bool(const std::string& = "") const;
    
    /* Return the argument of the last queried std::string (via
     * \function{got_guard}) as an \type{std::string}.*/
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


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

bool Command_Line_Arguments::got_guard(string& str)
{
    assert(str != "");
    lastGuardTest = str;
    
    return (assignments.find(str) != assignments.end());
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

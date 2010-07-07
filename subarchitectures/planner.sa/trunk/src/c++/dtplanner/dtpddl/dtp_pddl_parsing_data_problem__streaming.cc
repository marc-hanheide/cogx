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

#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"

using namespace Planning::Parsing;


namespace std
{
    std::ostream& operator<<(std::ostream& o, const Planning::Parsing::Problem_Data::Objective& objective)
    {
        using namespace Planning::Parsing;//::Problem_Data;
        if(objective == Problem_Data::Objective::maximise){
            o<<"maximise";
        } else if (objective == Problem_Data::Objective::minimise) {
            o<<"minimise";
        } else {
            UNRECOVERABLE_ERROR("Unknown objective :: "<<objective<<std::endl);
        }
        
        return o;
    }
}



std::ostream& std::operator<<(std::ostream& o, const Planning::Parsing::Problem_Data& data)
{

    o<<"(define ("<<data.get__problem_Name()<<")"<<std::endl;
    o<<"(:domain "<<data.domain_Data->get__domain_Name()<<")"<<std::endl;

    
    o<<"(:objects "<<endl;
    for(auto constant = data.constants_Description.begin()
            ; constant != data.constants_Description.end()
            ; constant++){

        QUERY_UNRECOVERABLE_ERROR(!constant->second.size(),
                                  "Object :: "<<constant->first<<std::endl
                                  <<"was given without a type specification."<<std::endl);
        
        o<<constant->first<<" - (either ";
        
        for(auto r_type = constant->second.begin()
                ; r_type != constant->second.end()
                ; r_type++){
            o<<*r_type<<" ";
        }
        
        o<<")"<<endl;
    }
    o<<" ) "<<endl;    

    
    CXX__deref__shared_ptr<basic_type> thingi;
    assert(!thingi.use_count());
    
    if(data.starting_state.use_count()){
        o<<"(:init ";
        o<<data.starting_state<<std::endl;
        o<<")"<<std::endl;
    }
    
    if(data.goal_formula.use_count()){
        o<<"(:goal "<<data.goal_formula<<" ) "<<std::endl;
    }

    if(data.objective_function.use_count()){
        o<<"(:metric "<<data.objective<<" "<<data.objective_function<<" ) "<<std::endl;//maximize (reward) )
    }
    
    
    o<<")";
    
    return o;
}


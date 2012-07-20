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

#include "dtp_pddl_parsing_data_domain.hh"

using namespace Planning::Parsing;

void Domain_Data::add__action_precondition()
{
    got__action_precondition = true;
    
    QUERY_UNRECOVERABLE_ERROR(0 != formula_parsing_level,
                              "Expecting formulae to appear at parse level :: "<<0<<std::endl
                              <<"But got :: "<<formula_parsing_level<<std::endl
                              <<"When parsing precondition for action :: "<<action_Name<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(1) == subformulae.end()
        , "Parsed an empty precondition for action :: "<<action_Name<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[1].size() != 1
        , "Parsed multiple separate formula while getting precondition for :: "
        <<action_Name<<std::endl);
    
    this->action_precondition = subformulae[1].back();

    subformulae[1] = Formula::Subformulae();
}

void Domain_Data::add__action_effect()
{
    got__action_effect = true;
    QUERY_UNRECOVERABLE_ERROR(0 != formula_parsing_level,
                              "Expecting formulae to appear at parse level :: "<<0<<std::endl
                              <<"But got :: "<<formula_parsing_level<<std::endl
                              <<"When parsing effect for action :: "<<action_Name<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(1) == subformulae.end()
        , "Parsed an empty effect for action :: "<<action_Name<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[1].size() != 1
        , "Parsed multiple separate formula while getting effect for :: "
        <<action_Name<<std::endl);
    
    this->action_effect = subformulae[1].back();
    subformulae[1] = Formula::Subformulae();
}

void Domain_Data::add__action()
{
    if(!got__action_effect){
        NEW_object_referenced_WRAPPED_deref_visitable_POINTER
            (Planning::Formula::Vacuous
             , tmp
             , static_cast<void*>(0));
        
        action_effect = tmp;
    }
    
    if(!got__action_precondition){
        
        NEW_object_referenced_WRAPPED_deref_visitable_POINTER
            (Planning::Formula::Vacuous
             , tmp
             , static_cast<void*>(0));
        action_precondition = tmp;
    }
    
    
    NEW_object_referenced_WRAPPED(Planning::Action_Schema
                                  , new_action_schema
                                  , action_Header
                                  , action_precondition
                                  , action_effect);

    action_Schemas.insert(new_action_schema);
    got__action_precondition = false;
    got__action_effect = false;
}


void Domain_Data::report__action_name(const std::string& str)
{
    NEW_object_referenced_WRAPPED(Planning::Action_Name
                                  , tmp
                                  , str);

    action_Name = tmp;
}

void Domain_Data::add__action_header()
{
    NEW_object_referenced_WRAPPED(Planning::Action_Header
                                  , action 
                                  , action_Name
                                  , typed_Arguments);
    
    action_Header = action;

    typed_Arguments = Typed_Arguments();
}

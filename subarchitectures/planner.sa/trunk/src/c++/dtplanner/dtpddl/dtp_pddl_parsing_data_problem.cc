/* Copyright (C) 2010 Charles Gretton (charles.gretton@gmail.com)
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


void Problem_Data::report__observations(const std::vector<std::string>& observationSeq)
{
}

Planning::Formula::Action_Proposition Problem_Data::get__prescribed_action()
{
    
    VERBOSER(1000, "Trying for prescribed action.");
    auto action_Schemas = domain_Data->get__action_Schemas();

    assert(action_Schemas.size());

    auto action_index = (random() % action_Schemas.size());

    auto i = 0;
    auto _action_Schema = action_Schemas.begin();
    for(i = 0; i != action_index; i++,_action_Schema++);
    auto action_Schema = *_action_Schema;
    
    auto action_name = action_Schema.get__name();

    auto action_headder = action_Schema.get__header();

    auto headder_contents = action_headder.contents();
    
    auto arguments = std::tr1::get<1>(headder_contents);
    
    auto variables = get__symbols(arguments);
    auto types = get__types(arguments);
    
    Planning::Constant_Arguments constants;

    VERBOSER(1000, "Looking into :: "<<action_name<<std::endl);
    
    for(auto argument = types.begin()
            ; argument != types.end()
            ; argument ++){

        assert(argument->size());
        auto _type = argument->begin();
        
        NEW_object_referenced_WRAPPED(Planning::Type, type, _type->get__name());

        Planning::Constant_Arguments potential;
        for(auto constant = constants_Description.begin()
                ; constant != constants_Description.end()
                ; constant++){
            if(constant->second.find(type) != constant->second.end()){
                potential.push_back(constant->first);
            }
        }

        auto index = (random() % potential.size());
        constants.push_back(potential[index]);
    }

    
    NEW_object_referenced_WRAPPED(Planning::Formula::Action_Proposition, answer, action_name, constants);

    return answer;
}


void Problem_Data::report__starting_state()
{
    QUERY_UNRECOVERABLE_ERROR(0 != formula_parsing_level,
                              "Expecting formulae to appear at parse level :: "<<0<<std::endl
                              <<"But got :: "<<formula_parsing_level<<std::endl
                              <<"When parsing starting state."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(1) == subformulae.end()
        , "Parsed an empty starting state."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[1].size() != 1
        , "Parsed multiple separate formula while getting starting state."<<std::endl);
    
    this->starting_state = subformulae[1].back();
    subformulae[1] = Formula::Subformulae();
}

void Problem_Data::report__objective_function()
{
    QUERY_UNRECOVERABLE_ERROR(0 != formula_parsing_level,
                              "Expecting formulae to appear at parse level :: "<<0<<std::endl
                              <<"But got :: "<<formula_parsing_level<<std::endl
                              <<"When parsing objective."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(1) == subformulae.end()
        , "Parsed an empty objective."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[1].size() != 1
        , "Parsed multiple separate formula while getting objective."<<std::endl);
    
    this->objective_function = subformulae[1].back();
    subformulae[1] = Formula::Subformulae();
}


void Problem_Data::report__goal_formula()
{
    QUERY_UNRECOVERABLE_ERROR(0 != formula_parsing_level,
                              "Expecting formulae to appear at parse level :: "<<0<<std::endl
                              <<"But got :: "<<formula_parsing_level<<std::endl
                              <<"When parsing goal."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae.find(1) == subformulae.end()
        , "Parsed an empty goal."<<std::endl);
    
    QUERY_UNRECOVERABLE_ERROR(
        subformulae[1].size() != 1
        , "Parsed multiple separate formula while getting goal."<<std::endl);
    
    this->goal_formula = subformulae[1].back();
    subformulae[1] = Formula::Subformulae();
}


const CXX__PTR_ANNOTATION(Domain_Data)& Problem_Data::get__domain_Data() const
{return domain_Data;}


CXX__PTR_ANNOTATION(Domain_Data)  Problem_Data::get__domain_Data()
{return domain_Data;}


Problem_Data::Problem_Data(CXX__PTR_ANNOTATION(Domain_Data)& domain_Data)
    :domain_Data(domain_Data)
{}

void Problem_Data::reset__domain_Data(CXX__PTR_ANNOTATION(Domain_Data)& in__domain_Data)
{
    domain_Data = in__domain_Data;
}

void Problem_Data::reset__domain_Data(const Planning::Domain_Name& domain_Name)
{
    if(domain_Name == domain_Data->domain_Name){
        WARNING("No change occurred when re-specifying problem associated to a domain.");
        return;
    }
                
    auto domains_iterator = Planning::Parsing::domains.find(domain_Name);
    if(domains_iterator != Planning::Parsing::domains.end()){
        reset__domain_Data(domains_iterator->second);
    }
}

void Problem_Data::reset__domain_Data(const std::string& str)
{
    for(auto domain = Planning::Parsing::domains.begin()
            ; domain != Planning::Parsing::domains.end()
            ; domain++){

        /* If we have parsed a domain description that is suitable for
         * this problem...*/
        if(domain->first.get__name() == str){
            reset__domain_Data(domain->first);
            return;
        }
    }
    
    WARNING("Problem :: "<<problem_Name<<std::endl
            <<"Is supposed to be associated with a domain identified as :: "<<str<<std::endl
            <<"Yet as far as I can tell, we have not parsed a domain file"<<std::endl
            <<"That that defines such a domain.");
}

void Problem_Data::add__problem_Name(const std::string& str)
{
    NEW_object_referenced_WRAPPED(Planning::Problem_Name, prob_nam, str);
    problem_Name = prob_nam;
}

const Planning::Problem_Name& Problem_Data::get__problem_Name() const
{
    return problem_Name;
}

void Problem_Data::report__minimisation_objective()
{
    objective = minimise;
}

void Problem_Data::report__maximisation_objective()
{
    objective = maximise;
}

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

#include "solver.hh"

using namespace Planning;
using namespace Planning::Parsing;

#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"
#include "problem_grounding.hh"
#include "planning_state.hh"

Solver::Solver(Planning::Parsing::Problem_Data& problem_Data)
    :problem_Data(problem_Data),
     preprocessed(false)// ,
//      constants_Description(0),
//      constants(0)
{
}

State& report__state(State&){
    UNRECOVERABLE_ERROR("unimplemented");
}

void Solver::preprocess()
{
    if(preprocessed) return;
    
    domain_Data = problem_Data.get__domain_Data();

    VERBOSER(3101, "Got domain :: "<<*domain_Data<<std::endl);
    
    
    
    proprocess__Constants_Data();
    
    configure__extensions_of_types();
    
    problem_Grounding = CXX__PTR_ANNOTATION(Problem_Grounding)
        (new Problem_Grounding(problem_Data,
                               domain_Data,
                               constants_Description,
                               extensions_of_types));

    problem_Grounding->ground_actions();
    problem_Grounding->ground_derived_predicates();
    problem_Grounding->ground_derived_perceptions();
    problem_Grounding->ground_starting_states();

    
    
    preprocessed = true;
}

void Solver::domain_constants__to__problem_objects()
{
    VERBOSER(3001, "Adding domain constants to the problem description.");
    
//     const Constants_Data& problem__Constants_Data = problem_Data;
    const Constants_Data& domain__Constants_Data = *domain_Data;
    auto domain__constants_Description = domain__Constants_Data.get__constants_Description();
    
    for(auto _constant = domain__constants_Description.begin()
            ; _constant != domain__constants_Description.end()
            ; _constant++){

        const Constant& constant = _constant->first;
        
        VERBOSER(3001, "Adding domain constant :: "<<constant<<std::endl
                 <<"As problem object. "<<std::endl);
        
        problem_Data
            .add__constant(constant.get__name());
        
        auto types = domain__Constants_Data.get__constantx_types(constant);

        QUERY_UNRECOVERABLE_ERROR
            (!types.size(),
             "No types were specified for domain constant :: "<<constant<<std::endl);
        
        for(auto type = types.begin()
                ; type != types.end()
                ; type++){

            QUERY_UNRECOVERABLE_ERROR(domain_Data->get__types_description().find(*type)
                                      == domain_Data->get__types_description().end(),
                                      "Thread :: "<<domain_Data->get__types_description().begin()->first.get__runtime_Thread()<<std::endl
                                      <<"Got query from thread :: "<<type->get__runtime_Thread()<<std::endl
                                      <<"For domain at :: "<<reinterpret_cast<basic_type::Runtime_Thread>(domain_Data.get())<<std::endl);
            
            assert(type->get__runtime_Thread() == reinterpret_cast<basic_type::Runtime_Thread>(domain_Data.get()));
            assert(domain_Data->get__types_description().find(*type) != domain_Data->get__types_description().end());
            
            problem_Data
                .add__type_of_constant(type->get__name());
        }
        
        problem_Data
            .add__constants();
    }

    constants_Description = problem_Data.get__constants_Description();
    
}

void Solver::configure__extensions_of_types()
{
    /* For each problem constant. */
    for(auto constant_Description = constants_Description.begin()
            ; constant_Description != constants_Description.end()
            ; constant_Description++){
        auto types = constant_Description->second;
        auto constant = constant_Description->first;


        QUERY_UNRECOVERABLE_ERROR(types.size() > 1
                                  , "Each object is supposed to be of exactly one type."<<std::endl
                                  <<"However :: "<<constant<<" was declared with type :: "<<types<<std::endl);
        QUERY_UNRECOVERABLE_ERROR(types.size() == 0
                                  , "Each object is supposed to be of exactly one type."<<std::endl
                                  <<"However :: "<<constant<<" was declared without a type."<<std::endl);
        
        for(auto type = types.begin()
                ; type != types.end()
                ; type++){

            auto types_description = domain_Data->get__types_description();


//             std::string for_debug;
//             {
//                 std::ostringstream oss;
//                 oss<<*domain_Data<<std::endl;
//                 for(auto thing = types_description.begin()
//                         ; thing != types_description.end()
//                         ; thing++){
//                     oss<<thing->first<<" "<<thing->second<<std::endl;
//                 }
//                 for_debug = oss.str();  
//             }
            
            QUERY_UNRECOVERABLE_ERROR(types_description.find(*type) == types_description.end(),
                                      "Unable to find :: "<<*type<<std::endl
                                      <<"In the domain type hierarchy :from : "<<*domain_Data<<std::endl);

            if(extensions_of_types.find(*type) == extensions_of_types.end()){
                extensions_of_types[*type] = Constants();
            }

            VERBOSER(3001, "Adding :: "<<constant<<" of type ::"<<*type<<std::endl);
            
            extensions_of_types[*type].insert(constant);
            
            for(auto super_type = types_description.find(*type)->second.begin()
                    ; super_type != types_description.find(*type)->second.end()
                    ; super_type++){
                extensions_of_types[*super_type].insert(constant);
            }
        }
    }
}

void Solver::proprocess__Constants_Data()
{
    domain_constants__to__problem_objects();
    
    
}



bool Solver::sanity() const
{
    if(!preprocessed) {
        WARNING("Tested sanity on :: "<<problem_Data.get__problem_Name()
                <<"before preprocessing."<<std::endl);
        return false;
    }   
}



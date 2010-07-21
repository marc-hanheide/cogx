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

Solver::Solver(Planning::Parsing::Problem_Data& problem_Data)
    :problem_Data(problem_Data),
     preprocessed(false)// ,
//      constants_Description(0),
//      constants(0)
{
}

void Solver::preprocess()
{
    domain_Data = problem_Data.get__domain_Data();

    problem_Grounding = CXX__PTR_ANNOTATION(Problem_Grounding)
        (new Problem_Grounding(problem_Data, domain_Data));
    
    proprocess__Constants_Data();

    problem_Grounding->ground_actions();
}


void Solver::proprocess__Constants_Data()
{
    const Constants_Data& problem__Constants_Data = problem_Data;
    //Constants_Data& domain__Constants_Data = *domain_Data;

    for(auto constant = problem__Constants_Data.get__constants().begin()
            ;constant != problem__Constants_Data.get__constants().end()
            ; constant++){
        domain_Data
            ->add__constant(constant->get__name());
        
        auto types = problem__Constants_Data.get__constantx_types(*constant);
        assert(types.size());
        
        for(auto type = types.begin()
                ; type != types.end()
                ; type++){
            domain_Data
                ->add__type_of_constant(type->get__name());
        }
        
        domain_Data
            ->add__constants();
    }

//     constants_Description
//         = const_cast<Planning::Parsing::Problem_Data::Constants_Description*>
//         (&(domain_Data->get__constants_Description()));
//     constants
//         = const_cast<Constants*>(&(domain_Data->get__constants()));

    constants_Description = domain_Data->get__constants_Description();
    constants = domain_Data->get__constants();
}



bool Solver::sanity() const
{
    if(!preprocessed) {
        WARNING("Tested sanity on :: "<<problem_Data.get__problem_Name()
                <<"before preprocessing."<<std::endl);
        return false;
    }   
}



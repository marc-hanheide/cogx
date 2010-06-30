#include "solver.hh"

using namespace Planning;
using namespace Planning::Parsing;

#include "dtp_pddl_parsing_data_problem.hh"
#include "dtp_pddl_parsing_data_domain.hh"

Solver::Solver(Planning::Parsing::Problem_Data& problem_Data)
    :problem_Data(problem_Data),
     preprocessed(false),
     constants_Description(0),
     constants(0)
{
}

void Solver::preprocess()
{
    domain_Data = problem_Data.get__domain_Data();

    proprocess__Constants_Data();
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

    constants_Description = &(domain_Data->get__constants_Description());
    constants = &(domain_Data->get__constants());
}



bool Solver::sanity() const
{
    if(!preprocessed) {
        WARNING("Tested sanity on :: "<<problem_Data.get__problem_Name()
                <<"before preprocessing."<<std::endl);
        return false;
    }   
}



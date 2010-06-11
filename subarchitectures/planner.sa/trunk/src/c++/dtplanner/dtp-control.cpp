
#include "dtp-control.hpp"

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new DTPCONTROL();
    }
}

#include "dtp_pddl_parsing_interface.hh"
#include "dtp_pddl_parsing_data.hh"

void DTPCONTROL::deliverObservation(Ice::Int id,
                                    const autogen::Planner::ObservationSeq& observationSeq,
                                    const Ice::Current&){
    
}

void DTPCONTROL::newTask(Ice::Int id,
                         const std::string& probleFile,
                         const std::string& domainFile, const Ice::Current&)
{
    
    Planning::Parsing::parse_domain(domainFile);
    Planning::Parsing::parse_problem(probleFile);
}

void DTPCONTROL::cancelTask(Ice::Int id, const Ice::Current&)
{
    
}

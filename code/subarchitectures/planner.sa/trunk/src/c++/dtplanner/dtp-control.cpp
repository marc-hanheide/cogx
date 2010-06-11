
#include "dtp-control.hpp"

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new DTPCONTROL();
    }
}


void DTPCONTROL::deliverObservation(Ice::Int id, const autogen::Planner::ObservationSeq& observationSeq, const Ice::Current&){}

void DTPCONTROL::newTask(Ice::Int id, const std::string& probleFile, const std::string& domainFile, const Ice::Current&) {}

void DTPCONTROL::cancelTask(Ice::Int id, const Ice::Current&){}

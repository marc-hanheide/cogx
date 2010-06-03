
#include "dtp-control.hpp"

extern "C" {
    cast::CASTComponentPtr newComponent() {
	return new DTPCONTROL();
    }
}


void DTPCONTROL::deliverObservation(const Ice::Current&, int id, ObservationSeq observationSeq){}

void DTPCONTROL::newTask(const Ice::Current&, int id, string probleFile, string domainFile); {}

void DTPCONTROL::cancelTask(const Ice::Current&, int id){}

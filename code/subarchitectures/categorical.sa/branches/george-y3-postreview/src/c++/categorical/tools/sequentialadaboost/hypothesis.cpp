//-----------------------------------------
// base class for weak hypothesis
//
// oscar martinez
//-----------------------------------------

#include "hypothesis.h"

using namespace oscar;

//-----------------------------------------
// constructor
//-----------------------------------------
Hypothesis::Hypothesis() {
	feature = NULL;
}

//-----------------------------------------
// destructor
//-----------------------------------------
Hypothesis::~Hypothesis() {
	if ( feature == NULL ) {
		delete feature;
		feature = NULL;
	}
}

//-----------------------------------------
// base class for examples
//
// oscar martinez
//-----------------------------------------

#include "example.h"
#include <stdlib.h>

using namespace oscar;

//-----------------------------------------
// constructor
//-----------------------------------------
Example::Example() {
	list_features = NULL;
	num_features = 0;
	classification = -1;
}

//-----------------------------------------
// destructor
//-----------------------------------------
Example::~Example() {
	if (list_features != NULL ) {
		delete [] list_features;
		list_features = NULL;
		num_features = 0;
	}
}

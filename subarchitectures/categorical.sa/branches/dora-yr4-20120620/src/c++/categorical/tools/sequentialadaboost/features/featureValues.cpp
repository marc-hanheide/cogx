//-----------------------------------------
// class for features
//
// oscar martinez
//-----------------------------------------

#include <string.h>
#include "featureValues.h"
#include <iostream>
#include <string.h>
#include <stdlib.h>

using namespace std;
using namespace oscar;

//-----------------------------------------
// constructor
//-----------------------------------------
FeatureValues::FeatureValues() {
	name = NULL;
}

//-----------------------------------------
// destructor
//-----------------------------------------
FeatureValues::~FeatureValues() {
	if ( name != NULL ){
		free(name);
		name = NULL;
	}
}


//-----------------------------------------
//
//-----------------------------------------
int FeatureValues::setName(char *n) {
	if ( name != NULL ){
		free(name);
		name = NULL;
	}

	int l = strlen(n);
	name = (char *)malloc(l*sizeof(char)+2);

	strcpy(name, n);

	return 1; //ok
}



//-----------------------------------------
// operator =
//-----------------------------------------
FeatureValues & FeatureValues::operator=(const FeatureValues &fv) {

	value = fv.value;
	//setName(fv.name);

	//cerr << "operator= called" << endl;

	return *this;
}



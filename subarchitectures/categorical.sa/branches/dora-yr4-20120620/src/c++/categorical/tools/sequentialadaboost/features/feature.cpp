//-----------------------------------------
// base class for features
//
// oscar martinez
//-----------------------------------------

#include "feature.h"
#include <fstream>
#include <iostream>
#include <math.h>
#include <string.h>

using namespace oscar;
using namespace std;

//-----------------------------------------
// constructor
//-----------------------------------------
Feature::Feature() {
	name = NULL;
}

//-----------------------------------------
// destructor
//-----------------------------------------
Feature::~Feature() {
	if (name != NULL ) {
		free(name);
		name = NULL;
	}

}

//-----------------------------------------
//
//-----------------------------------------
void Feature::setName(char *n) {
	if (name != NULL ) {
		free(name);
		name = NULL;
	}

	int l = strlen(n);
	name = (char*)malloc(l*sizeof(char)+2);

	strcpy(name,n);
}

//-----------------------------------------
//
//-----------------------------------------
void Feature::setType(char *t) {

	if ( strlen(t) >9 ) {
		sprintf(type, "?");
	}
	else {
		strcpy(type, t);
	}
}

//-----------------------------------------
// save it
//-----------------------------------------
int Feature::save(char *filename) {
	ofstream ofs;
	ofs.open(filename);

	if ( !ofs.is_open() ) {
		cerr << "ERROR:Feature4::save():opening " << filename << endl;
		return -1;
	}
	ofs << value;
	ofs.close();

	return 1;
}


//-----------------------------------------
// get feature
//-----------------------------------------
int Feature::load(char *filename) {
	ifstream ifs;
	ifs.open(filename);

	if ( !ifs.is_open() ) {
		cerr << "ERROR:Feature::load():opening " << filename << endl;
		return -1;
	}

	ifs >> value;

	ifs.close();

	return 1;
}




















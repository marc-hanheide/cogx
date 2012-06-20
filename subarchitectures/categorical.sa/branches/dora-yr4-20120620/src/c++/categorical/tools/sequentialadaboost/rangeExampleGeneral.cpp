//-----------------------------------------
//
// oscar martinez
//-----------------------------------------


#include "rangeExampleGeneral.h"


using namespace std;
using namespace oscar;

//-----------------------------------------
// constructors
//-----------------------------------------
RangeExampleGeneral::RangeExampleGeneral() {
	pos_probability = NULL;	
}

//-----------------------------------------
// constructors
//-----------------------------------------
RangeExampleGeneral::RangeExampleGeneral(int n): RangeExample(n){

	pos_probability = NULL;
}



//-----------------------------------------
// destructor
//-----------------------------------------
RangeExampleGeneral::~RangeExampleGeneral(){
	
	if ( pos_probability != NULL ){
		delete [] pos_probability;
	}
	pos_probability = NULL;
}















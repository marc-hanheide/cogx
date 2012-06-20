


//-----------------------------------------
// example with more information
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_RANGE_EXAMPLE_GENERAL__
#define __OSCAR_RANGE_EXAMPLE_GENERAL__



#include "rangeExample.h"


namespace oscar{

//-----------------------------------------
// class RangeExampleGeneral
//-----------------------------------------
class RangeExampleGeneral: public RangeExample {

	// methods
	public:
		RangeExampleGeneral();
		RangeExampleGeneral(int n);
		~RangeExampleGeneral();

	// member data
	double confidence;     // logistic function
	double confidence_sum; // \sum_t alpha_t \times h_t(x)
	
	double *pos_probability; //probability of each class
};

}

#endif // __OSCAR_RANGE_EXAMPLE_GENERAL__

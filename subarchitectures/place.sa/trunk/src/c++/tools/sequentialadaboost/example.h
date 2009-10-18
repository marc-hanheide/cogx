
//-----------------------------------------
// base class for examples
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_EXAMPLE__
#define __OSCAR_EXAMPLE__


#include "ada_global.h"
#include "features/featureValues.h"
#include "features/feature.h"

namespace oscar {

//-----------------------------------------
// class Example
//-----------------------------------------
class Example {

    // functions
    public:
        Example();
		~Example();


	// data member
	public:
	    enum {negative=0, positive=1};

		int classification; // classification of the point by a classifier
	    int type;   // positive, negative
		double weight;

		FeatureValues *list_features;
		int num_features;

		carmen_point_t pose;

};

}

#endif //__OSCAR_EXAMPLE__


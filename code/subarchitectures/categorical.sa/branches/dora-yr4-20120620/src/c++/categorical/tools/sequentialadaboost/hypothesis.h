//-----------------------------------------
// base class for weak hypothesis
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_WEAK_HYPOTHESIS__
#define __OSCAR_WEAK_HYPOTHESIS__


#include "features/feature.h"

namespace oscar {

//-----------------------------------------
// class Hypothesis
//-----------------------------------------
class Hypothesis {

    // functions
    public:
        Hypothesis();
		~Hypothesis();

		Feature *feature;

};

}

#endif // __OSCAR_WEAK_HYPOTHESIS__


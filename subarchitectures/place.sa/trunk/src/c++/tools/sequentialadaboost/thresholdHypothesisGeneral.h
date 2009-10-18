//-----------------------------------------
// Threshold Hypothesis for Adaboost generalized:
//  "Improved Boosting Algorithmsusing confidence predictions", Robert E. Schapire, Yoram Singer.
//  Machine Learning, 37(3):297-336, 1999.
//
// Look for a threshold that best classify the examples
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_THRESHOLD_HYPOTHESIS_GENERAL__
#define __OSCAR_THRESHOLD_HYPOTHESIS_GENERAL__

#include "rangeExample.h"
#include "thresholdHypothesis.h"

namespace oscar {


//-----------------------------------------
// class Hypothesis
//-----------------------------------------
class ThresholdHypothesisGeneral: public ThresholdHypothesis {

    // functions
    public:
        ThresholdHypothesisGeneral();
		~ThresholdHypothesisGeneral();

		// members
		void classify(oscar::RangeExample *e, int index_feature, double *confidence);
		void classifyFeature(double featureValue, double *confidence);

};

}

#endif // __OSCAR_THRESHOLD_HYPOTHESIS_GENERAL__















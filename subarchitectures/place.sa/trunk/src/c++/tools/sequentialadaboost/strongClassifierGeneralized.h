//-----------------------------------------
// strong classifier
//
// strong classifier obtained by adaboost
//
// "The boosting Approach to machine learning: and overview" robert e. schapire. 2002
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_STRONG_CLASSIFIER_GENERALIZED__
#define __OSCAR_STRONG_CLASSIFIER_GENERALIZED__

#include "rangeExample.h"
#include "dynamictable.h"
#include "thresholdHypothesisGeneral.h"

#include "features/feature.h"
#include "features/feature13.h"
#include "features/feature20.h"

#include <fstream>

using namespace std;
using namespace oscar;

typedef DynamicTable<oscar::ThresholdHypothesisGeneral> dyntab_hypotheses;

namespace oscar {

//-----------------------------------------
// class Adaboost
//-----------------------------------------
class StrongClassifierGeneralized {

    // functions
    public:
        StrongClassifierGeneralized();
		~StrongClassifierGeneralized();

		void readHypotheses(ifstream &ifstream);
		void classify(RangeExample *e, int num_hypotheses, double *confidence, double *confidence_sum, int classification_type);
//		void classifyFeatures(PlaceData::LaserFeatures& featureVector, int num_hypotheses, double *confidence, double *confidence_sum, int classification_type);

	// data member
	public:

		dyntab_hypotheses *hypotheses;
		double *alpha;
		bool debug;

		// featuress used by other
		oscar::Feature13 f13;
		oscar::Feature20 f20;

};

}

#endif //__OSCAR_STRONG_CLASSIFIER_GENERALIZED__

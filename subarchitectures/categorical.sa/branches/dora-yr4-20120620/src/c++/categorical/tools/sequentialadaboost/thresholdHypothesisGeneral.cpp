//-----------------------------------------
// Threshold Hypothesis for Adaboost generalized:
// Look for a threshold that best classify the examples
//
// oscar martinez
//-----------------------------------------

#include "thresholdHypothesisGeneral.h"
#include <iostream>
#include "features/featureValues.h"

using namespace std;
using namespace oscar;

//-----------------------------------------
// constructor
//-----------------------------------------
ThresholdHypothesisGeneral::ThresholdHypothesisGeneral() {}


//-----------------------------------------
// destructor
//-----------------------------------------
ThresholdHypothesisGeneral::~ThresholdHypothesisGeneral() {}




//-----------------------------------------
// classify one example: [-1,+1]
//-----------------------------------------
void ThresholdHypothesisGeneral::classify(oscar::RangeExample *e, int index_feature, double *confidence) {
	FeatureValues *fv;

	int type=0;
	fv = e->getFeature(index_feature);

	if ( direction==1 ) {
		// positives are >= than threshold
		if ( fv->value >= threshold ) {
			type = 1;
		}
		else {
			type = -1;
		}
	}
	else
	if ( direction==2 ) {
		// positives are < than threshold
		if ( fv->value < threshold ) {
			type = 1; // positive
		}
		else {
			type = -1; // negative
		}
	}

	*confidence = 1.0 - (misclassified / sum_weights);

	if ( type == -1 ) {
		// neagtive
		*confidence = -(*confidence);
	}

	return;
}

void ThresholdHypothesisGeneral::classifyFeature(double featureValue, double *confidence) {
  int type=0;

  if ( direction==1 ) {
    // positives are >= than threshold
    if ( featureValue >= threshold ) {
      type = 1;
    }
    else {
      type = -1;
    }
  }
  else
    if ( direction==2 ) {
      // positives are < than threshold
      if ( featureValue < threshold ) {
	type = 1; // positive
      }
      else {
	type = -1; // negative
      }
    }

  *confidence = 1.0 - (misclassified / sum_weights);

  if ( type == -1 ) {
    // neagtive
    *confidence = -(*confidence);
  }

  return;
}





















//-----------------------------------------
// Threshold Hypothesis:
// Look for a threshold that best classify the examples
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_THRESHOLD_HYPOTHESIS__
#define __OSCAR_THRESHOLD_HYPOTHESIS__

#include "rangeExample.h"
#include "hypothesis.h"
#include "dynamictable.h"

namespace oscar {


//-----------------------------------------
// class Hypothesis
//-----------------------------------------
class ThresholdHypothesis: public Hypothesis {

    // functions
    public:
        ThresholdHypothesis();
		~ThresholdHypothesis();

		int train(DynamicTable<oscar::RangeExample> *list_examples, int index_feature);
		int classify(oscar::RangeExample *e, int index_feature);
		int classifyFeature(double featureValue);

	// data members
	public:
		typedef struct {
			double value;
			int type;
			double weight;
		}Instance;

		int direction;   // 1: consider positive the elements to the right of pos
						 // 2: consider positive the elements to the left of pos
		double threshold;
		double misclassified;
		double sum_weights;
		int index;
		double alpha;
		int num_examples;


		bool debug;

	protected:
		//int compar(const void *, const void *); // comparison function

};

}

#endif // __OSCAR_THRESHOLD_HYPOTHESIS__















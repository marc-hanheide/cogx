//-----------------------------------------
// Threshold Hypothesis:
// Look for a threshold that best classify the examples
//
// oscar martinez
//-----------------------------------------

#include "thresholdHypothesis.h"
#include <iostream>
#include "features/featureValues.h"

using namespace std;
using namespace oscar;

//-----------------------------------------
// comparison function
//-----------------------------------------
int compar(const void *a, const void *b) {

	ThresholdHypothesis::Instance *i1 = (ThresholdHypothesis::Instance *)a;
	ThresholdHypothesis::Instance *i2 = (ThresholdHypothesis::Instance *)b;

	if ( i1->value < i2->value ) {
		return -1;
	}
	else
	if ( i1->value > i2->value ) {
		return 1;
	}
	else {
		return 0;
	}
}


//-----------------------------------------
// constructor
//-----------------------------------------
ThresholdHypothesis::ThresholdHypothesis() {
	//cerr << "ThresholdHypothesis(): constructor" << endl;
}


//-----------------------------------------
// destructor
//-----------------------------------------
ThresholdHypothesis::~ThresholdHypothesis() {
	//cerr << "ThresholdHypothesis(): desctructor" << endl;
}


//-----------------------------------------
// train
//-----------------------------------------
int ThresholdHypothesis::train(DynamicTable<oscar::RangeExample> *list_examples, int index_feature)
{

	Instance *list;
	RangeExample *re;
	FeatureValues *fv;
	int pos, neg;

	num_examples = list_examples->num();
	list = new Instance[num_examples];

	// insert in a list the features and type (pos or neg) of each example
	pos = 0;
	neg = 0;
	sum_weights = 0.0;
	for(int i=0; i<num_examples; i++) {
		re = list_examples->getElement(i);
		fv = re->getFeature(index_feature);
		list[i].value = fv->value;
		list[i].type = re->type;
		list[i].weight = re->weight;
		sum_weights += re->weight;
		if ( list[i].type ==Example::positive ) {
			pos++;
		}
		else {
			neg++;
		}
	}

	//sort
	qsort(list, num_examples, sizeof(Instance), compar);

	// debug
	/*
	for(int i=0; i<num; i++) {
		cerr << fixed << list[i].value << " " << list[i].type << endl;

	}
	*/


	// Look for a threshold that best classify the examples.
	// Two sets are maintained:
	// 	left: elements to the left of the threshold
	// 	right: elements to the right of the threshold
	int pos_left, neg_left;
	int pos_right, neg_right;

	// values to the left and right of threshold. 1 value = weight
	double pos_left_value, neg_left_value;
	double pos_right_value, neg_right_value;


	// at the begining left is empty
	pos_left=0;
	neg_left=0;
	pos_right=pos;
	neg_right=neg;
	// values
	pos_left_value=0.0;
	neg_left_value=0.0;

	pos_right_value=0.0;
	neg_right_value=0.0;
	for(int i=0; i<num_examples; i++) {
		if ( list[i].type ==Example::positive ) {
			pos_right_value += list[i].weight;
		}
		else {
			neg_right_value += list[i].weight;
		}
	}


	int min_pos = -1; //position with minimum misclassification
	//double min_mis = pos+neg;  // minimum misclasification
	double min_mis = pos_right_value + neg_right_value;  // minimum misclasification
	double mis=-1;
	direction =-1;

	for( int i=0; i<num_examples; i++) {
		// two cases
		// 1: consider values >= than position as positive
		//mis = neg_right + pos_left;
		mis = neg_right_value + pos_left_value;
		if ( mis < min_mis ) {
			min_mis = mis;
			direction = 1;
			min_pos = i;
		}
		// 2: consider values < than position as positive
		//mis = neg_left + pos_right;
		mis = neg_left_value + pos_right_value;
		if ( mis < min_mis ) {
			min_mis = mis;
			direction = 2;
			min_pos = i;
		}
		// advance pos
		if ( list[i].type == Example::positive ) {
			//pos_right--;
			//pos_left++;
			pos_right_value -= list[i].weight;
			pos_left_value += list[i].weight;
		}
		else {
			//neg_right--;
			//neg_left++;
			neg_right_value -= list[i].weight;
			neg_left_value += list[i].weight;
		}
	}

	misclassified = min_mis;
	if ( min_pos == 0 ) {
		//threshold = list[min_pos].value - 1;
		threshold = list[min_pos].value;
	}
	else
	if ( min_pos == num_examples-1 ) {
		//threshold = list[min_pos].value + 1;
		threshold = list[min_pos].value;
	}
	else {
		threshold = (list[min_pos-1].value + list[min_pos].value) / 2.0;
	}

	delete list;

	return 1;
}


//-----------------------------------------
// classify one example
//-----------------------------------------
int ThresholdHypothesis::classify(oscar::RangeExample *e, int index_feature) {
	FeatureValues *fv;

	fv = e->getFeature(index_feature);

	if ( direction==1 ) {
		// positives are >= than threshold
		if ( fv->value >= threshold ) {
			return Example::positive;
		}
		else {
			return Example::negative;
		}
	}
	else
	if ( direction==2 ) {
		// positives are < than threshold
		if ( fv->value < threshold ) {
			return Example::positive;
		}
		else {
			return Example::negative;
		}
	}

	return -1; // something is wrong
}


int ThresholdHypothesis::classifyFeature(double featureValue) {
	if ( direction==1 ) {
		// positives are >= than threshold
		if ( featureValue >= threshold ) {
			return Example::positive;
		}
		else {
			return Example::negative;
		}
	}
	else
	if ( direction==2 ) {
		// positives are < than threshold
		if ( featureValue < threshold ) {
			return Example::positive;
		}
		else {
			return Example::negative;
		}
	}

	return -1; // something is wrong
}





















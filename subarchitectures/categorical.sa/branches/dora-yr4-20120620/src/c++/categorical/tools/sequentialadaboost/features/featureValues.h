//-----------------------------------------
// class for features
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE_VALUES__
#define __OSCAR_FEATURE_VALUES__


namespace oscar {

//-----------------------------------------
// class FeatureType
//-----------------------------------------
class FeatureValues {


	public:
		// constructor;
		FeatureValues();
		//destructor
		~FeatureValues();

		int setName(char *name);
		FeatureValues & operator=(const FeatureValues &fv);

	public:
		double value;
		char *name;  // type of feature
};

}
#endif  //__OSCAR_FEATURE_VALUES__


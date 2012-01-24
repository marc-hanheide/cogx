
//-----------------------------------------
// base class for feature
//
// oscar martinez
//-----------------------------------------

#ifndef __OSCAR_FEATURE__
#define __OSCAR_FEATURE__


#include "../dynamictable.h"
#include "../example.h"
#include <iostream>
#include <math.h>

using namespace std;


namespace oscar{
  
typedef struct {
        char name[100];
        char description[2000];
        bool hasParam;
        double param;
        double result;  // resulting feature value
        double weight;
} FeatureInfo;

typedef DynamicTable<FeatureInfo> dyntab_featuresInfo;

class Example;

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature {

    // functions
    public:
        Feature();
		virtual ~Feature();

		// return   1 ok
		//	       -1 error
		virtual int getFeature(Example *e, int index_feature)=0;

		virtual void test() {
			cerr << "Feature::test" << endl;
		}

		virtual int load(char *filename);
		virtual int calculate(Example *e) { return 1; };
		virtual int save(char *n);

		void setName(char *n);
		void setType(char *t);


	// data members
	public:
		char *name;
		char type[10];
		double value;
};


} // end namespace

#endif // __OSCAR_FEATURES__

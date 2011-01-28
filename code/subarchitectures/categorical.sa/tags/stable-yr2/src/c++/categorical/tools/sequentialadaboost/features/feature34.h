//-----------------------------------------------------------
// feature 34:
//		  gaps / (number_of_beams - 1)
// type 34
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE34__
#define __OSCAR_FEATURE34__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature34: public Feature {

    // functions
    public:
		Feature34();
		~Feature34();
		
		void setup(double t);

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature34::test()" << endl; }
		
		double threshold;
};


} // end namespace

#endif // __OSCAR_FEATURE34__






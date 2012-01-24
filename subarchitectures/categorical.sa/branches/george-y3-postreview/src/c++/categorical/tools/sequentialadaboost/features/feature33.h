//-----------------------------------------------------------
// feature 33:
//		  gaps / (number_of_beams - 1)
// type 33
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE33__
#define __OSCAR_FEATURE33__

#include "../rangeExample.h"


namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature33: public Feature {

    // functions
    public:
		Feature33();
		~Feature33();
		
		void setup(double t);

		int getFeature(oscar::Example *example, int index_feature);

		void test() { cerr << "Feature33::test()" << endl; }
		
		
	public:	
		double threshold;
};


} // end namespace

#endif // __OSCAR_FEATURE33__






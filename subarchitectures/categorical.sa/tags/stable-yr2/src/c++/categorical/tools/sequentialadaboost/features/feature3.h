//-----------------------------------------------------------
// feature 3:
//		average of the difference between consecutive beams
//		beams are cut when they are > beam_length
//
// oscar martinez
//-----------------------------------------------------------

#ifndef __OSCAR_FEATURE3__
#define __OSCAR_FEATURE3__

#include "../rangeExample.h"
#include <math.h>

namespace oscar{

//-----------------------------------------
// class Feature
//-----------------------------------------
class Feature3: public Feature {

    // functions
    public:
		Feature3();
		Feature3(double beam_length);
		~Feature3();

		int getFeature(oscar::Example *example, int index_feature);
		void setup(double beam_length);

		// data members
		double beam_length;

		void test() { cerr << "Feature3::test()" << endl; }
};


} // end namespace

#endif // __OSCAR_FEATURE3__

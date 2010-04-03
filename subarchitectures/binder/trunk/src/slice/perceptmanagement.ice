#ifndef PERCEPTMANAGEMENT_ICE
#define PERCEPTMANAGEMENT_ICE 
 

#include <cast/slice/CDL.ice>


module binder {
module autogen {
module perceptmanagement {


dictionary<cast::cdl::WorkingMemoryAddress,cast::cdl::WorkingMemoryAddress> WMAReferece;

class PerceptBeliefMaps {
	WMAReferece percept2Belief;
}; 


}; // end perceptmanagement
}; // end autogen
}; // end binder


#endif
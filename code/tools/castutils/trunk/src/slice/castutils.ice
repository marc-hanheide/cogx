#ifndef CASTUTILS_ICE
#define CASTUTILS_ICE

#include <cast/slice/CDL.ice>

module castutils {
    module slice {

    
        class WMMutex {
              string name;
              string holderName;
              cast::cdl::WorkingMemoryAddress addr;
        };
    };
};

#endif
#ifndef CASTUTILS_ICE
#define CASTUTILS_ICE

#include <cast/slice/CDL.ice>

module castutils {

  module slice {

    // General string to object sequence dictionary
    sequence<Object> ObjectSeq;
    dictionary<string, ObjectSeq > ObjectSeqDict;



		dictionary<cast::cdl::WorkingMemoryAddress,cast::cdl::WorkingMemoryAddress> WMAddressMap;
		
		class WMMap {
			WMAddressMap map;
		};		
    
    	class WMTrackedBeliefMap extends WMMap {};

    	class GroundedToSharedBeliefMap extends WMMap {};
    	
    	class PrivateToAssumedBeliefMap extends WMMap {};
    	
    	class AssumedToMergedBeliefMap extends WMMap {};

    
        class WMMutex {
              string name;
              string holderName;
              cast::cdl::WorkingMemoryAddress addr;
        };

		///Prototyping the idea of storing future operations

		/**
			Highest level class to describe an operation on WM
		*/
		class WMOperation {
			cast::cdl::WorkingMemoryAddress address;
		};

/**
An operation that involves a particular entry
*/
		class WMOperationWithContent extends WMOperation {
			 Object entry;
		};
		
		class WMAdd extends WMOperationWithContent {
		};

		class WMOverwrite extends WMOperationWithContent {
		};

		class WMDelete extends WMOperation {
		};


    };
};

#endif

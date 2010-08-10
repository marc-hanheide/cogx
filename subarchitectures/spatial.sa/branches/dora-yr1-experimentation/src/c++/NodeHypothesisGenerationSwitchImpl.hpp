#include <cast/architecture.hpp>
#include "PlaceManager.hpp"
#include <Ice/Handle.h>
#include <string>
#include <NodeHypothesisGenerationSwitch.hpp>

namespace spatial {

class NodeHypothesisGenerationSwitchImpl : 
	public spatial::NodeHypothesisGenerationSwitch {
	
	public:
		NodeHypothesisGenerationSwitchImpl(IceInternal::Handle<spatial::PlaceManager> pmPtr);
		virtual void setNodeHypothesisGeneration(bool on, const Ice::Current&);
		
	private:
		IceInternal::Handle<spatial::PlaceManager> pmPtr;
		
};

}

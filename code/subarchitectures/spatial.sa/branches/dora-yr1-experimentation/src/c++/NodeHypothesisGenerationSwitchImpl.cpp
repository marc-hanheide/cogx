#include "NodeHypothesisGenerationSwitchImpl.hpp"

namespace spatial {

NodeHypothesisGenerationSwitchImpl::NodeHypothesisGenerationSwitchImpl(IceInternal::Handle<spatial::PlaceManager> pmPtr) {
	this->pmPtr = pmPtr;
	
	IceInternal::Handle<NodeHypothesisGenerationSwitchImpl> ptr = IceInternal::Handle<NodeHypothesisGenerationSwitchImpl>(this);    
    		
    std::string compID = this->pmPtr.get()->getComponentID(); // name
    std::string category = this->pmPtr.get()->toServantCategory<NodeHypothesisGenerationSwitch>(); // category 
    
	this->pmPtr.get()->registerIceServer<NodeHypothesisGenerationSwitchImpl,NodeHypothesisGenerationSwitch>(ptr);
    
}

void NodeHypothesisGenerationSwitchImpl::setNodeHypothesisGeneration(bool on, const Ice::Current&) {

	this->pmPtr.get()->log("setting m_NodeHypothesisGenerationOn to %s", on ? "true" : "false");
	this->pmPtr.get()->m_NodeHypothesisGenerationOn = on;
}

}

#include "DoorDetectionSwitchImpl.hpp"

namespace navsa {

DoorDetectionSwitchImpl::DoorDetectionSwitchImpl(IceInternal::Handle<navsa::NavGraphProcess> ngpPtr) {
	this->ngpPtr = ngpPtr;
	
	IceInternal::Handle<DoorDetectionSwitchImpl> ptr = IceInternal::Handle<DoorDetectionSwitchImpl>(this);    
//    // register server -- old way to register server
//    ngpPtr.get()->registerIceServer<DoorDetectionToggleServer>(
//    		"ToggleDoorDetectionServer", 
//    		"ToggleDoorDetectionServer", 
//    		ptr
//    		);
    		
//    std::string compID = this->ngpPtr.get()->getComponentID(); // name
//    std::string category = this->ngpPtr.get()->toServantCategory<DoorDetectionSwitch>(); // category    
//    this->ngpPtr.get()->println("name = %s and category = %s", compID.c_str(), category.c_str());
    
	this->ngpPtr.get()->registerIceServer<DoorDetectionSwitchImpl,DoorDetectionSwitch>(ptr);
    
}

void DoorDetectionSwitchImpl::setDoorDetection(bool on, const Ice::Current&) {

	this->ngpPtr.get()->log("setting m_DoorDetectionOn to %s", on ? "true" : "false");
	this->ngpPtr.get()->m_DoorDetectionOn = on;
}

}

#include <cast/architecture.hpp>
#include "NavGraphProcess.hpp"
#include <Ice/Handle.h>
#include <string>
#include <DoorDetectionSwitch.hpp>

namespace navsa {

class DoorDetectionSwitchImpl : 
	public DoorDetectionSwitch {
	
	public:
		DoorDetectionSwitchImpl(IceInternal::Handle<navsa::NavGraphProcess> ngpPtr);
		virtual void setDoorDetection(bool on, const Ice::Current&);
		
	private:
		IceInternal::Handle<navsa::NavGraphProcess> ngpPtr;
		
};

}

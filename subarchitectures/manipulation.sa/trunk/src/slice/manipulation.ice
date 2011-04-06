#ifndef MANIPULATION_ICE
#define MANIPULATION_ICE

#include <cast/slice/CDL.ice>
#include <VisionData.ice>

module manipulation {
    module slice {
		enum Completion {
			// failed to grasp the object
			FAILED,
			// grasped the object
			SUCCEEDED
		};
		

		class GraspCommand {
			VisionData::VisualObject targetObject;
			Completion comp;
		};
		
		class PutDownCommand {
			VisionData::VisualObject targetObject;
			VisionData::VisualObject basedOnObject;
			Completion comp;			
		};
		

    };
};

#endif

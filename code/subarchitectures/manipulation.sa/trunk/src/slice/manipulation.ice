#ifndef MANIPULATION_ICE
#define MANIPULATION_ICE

#include <cast/slice/CDL.ice>
#include <VisionData.ice>

module manipulation {
    module slice {
		enum ManipulationCompletion {
			// failed to grasp the object
			FAILED,
			// grasped the object
			SUCCEEDED
		};
		
		enum ManipulationCommandStatus {
			PENDING,
			FINISHED,
		};
		
		class ManipulationCommand {
			ManipulationCommandStatus status;			
			ManipulationCompletion comp;
		};
		
		/**
   		* @brief put down a given visual object
   		* @param basedOnObject object to put down the current object
   		* @param comp returns the completion of the put down task
   		* @author Torben Toeniges
   		**/
		class PutDownCommand extends ManipulationCommand {
			VisionData::VisualObject basedOnObject;
		
		
		};
		
		
		/**
   		* @brief perform a far arm movement to place the gripper in front of the given object
   		* @param targetObject object to grasp 
   		* @param comp returns the completion of the task
   		* @author Torben Toeniges
   		**/
		class FarArmMovementCommand extends ManipulationCommand {
			VisionData::VisualObject targetObject;
			double xError;
			double yError;
			double zError;
		
		};
		
		/**
   		* @brief grasp a given visual object with a linear arm movement approach
   		* @param targetObject object to grasp 
   		* @param comp returns the completion of the task
   		* @author Torben Toeniges
   		**/
		class LinearGraspApproachCommand extends ManipulationCommand {
			VisionData::VisualObject targetObject;
		};
		
		/**
   		* @brief simulate the grasp command
   		* @param targetObject object to simulate to 
   		* @param translational error value of the inverse kinematic
   		* @author Torben Toeniges
   		**/
		class SimulateGraspCommand extends ManipulationCommand {
			VisionData::VisualObject targetObject;
			double xError;
			double yError;
			double zError;
		};
		
		/**
   		* @brief approach a given object with linear robot base movements
   		* @param targetObject the object to approach
   		* @param comp returns the completion of the approaching task
   		* @author Torben Toeniges
   		**/
		class LinearBaseMovementApproachCommand extends ManipulationCommand {
			VisionData::VisualObject targetObject;
		};
    };
};

#endif

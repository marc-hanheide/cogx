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
   		* @brief puts down a given visual object
   		* @author Torben Toeniges
   		**/
		class PutDownCommand extends ManipulationCommand {
			VisionData::VisualObject basedOnObject;
		
		
		};
		
		
		/**
   		* @brief performs a far arm movement to place the gripper in front of the given object
   		* @author Torben Toeniges
   		**/
		class FarArmMovementCommand extends ManipulationCommand {
			VisionData::VisualObject targetObject;
			double xError;
			double yError;
			double zError;
		
		};
		
		/**
   		* @brief grasps a given visual object with a linear arm movement approach
   		* @author Torben Toeniges
   		**/
		class LinearGraspApproachCommand extends ManipulationCommand {
			VisionData::VisualObject targetObject;
		};
		
		/**
   		* @brief simulates the grasp command
   		* @author Torben Toeniges
   		**/
		class SimulateGraspCommand extends ManipulationCommand {
			VisionData::VisualObject targetObject;
			double xError;
			double yError;
			double zError;
		};
		
		/**
   		* @brief approaches a given object with linear robot base movements
   		* @author Torben Toeniges
   		**/
		class LinearBaseMovementApproachCommand extends ManipulationCommand {
			VisionData::VisualObject targetObject;
		};
		
		/**
   		* @brief stops the manipulator movement
   		* @author Torben Toeniges
		**/
		class StopCommand extends ManipulationCommand {
		};
    };
};

#endif

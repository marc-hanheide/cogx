#ifndef MANIPULATION_ICE
#define MANIPULATION_ICE

#include <cast/slice/CDL.ice>
#include <VisionData.ice>

module manipulation {
    module slice {
		enum ManipulationCompletion {
			FAILED,
			SUCCEEDED,
			ONTHEWAY
		};
		
		enum ManipulationCommandStatus {
			NEW,
			CHANGED,
			PENDING,
			FINISHED,
			COMMANDFAILED
		};
		
		enum GraspingStatus {
			GRASPING,
			NOTGRASPING
		};
		
		/**
   		* @brief Manipulation Command class
   		* @author Torben Toeniges
   		**/
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
			GraspingStatus graspStatus;
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
   		* @brief stops the manipulator movement
   		* @author Torben Toeniges
		**/
		class StopCommand extends ManipulationCommand {
		};
		
		/**
   		* @brief stops the manipulator movement
   		* @author Torben Toeniges
		**/
		class MoveArmToHomePositionCommand extends ManipulationCommand {
		};
		
		/**
   		* @brief open gripper
   		* @author Torben Toeniges
		**/
		class OpenGripperCommand extends ManipulationCommand {
		};
		
		/**
   		* @brief close gripper
   		* @author Torben Toeniges
		**/
		class CloseGripperCommand extends ManipulationCommand {
			GraspingStatus graspStatus;
		};
    };
};

#endif

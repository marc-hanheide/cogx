#ifndef MANIPULATION_ICE
#define MANIPULATION_ICE

#include <cast/slice/CDL.ice>
#include <Math.ice>

module manipulation {
    module slice {
		enum ManipulationCompletion {
			FAILED,
			SUCCEEDED,
			ONTHEWAY,
			COMPINIT
		};
		
		enum ManipulationCommandStatus {
			NEW,
			CHANGED,
			PENDING,
			FINISHED,
			COMMANDFAILED
		};
		
		enum GraspingStatus {
			GRASPINGSTATUSINIT,
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
		
		class ManipulationExternalCommand extends ManipulationCommand {
		};

		class ManipulationInternalCommand extends ManipulationCommand {
		};
		
		/**
   		* @brief puts down a given visual object
   		* @author Torben Toeniges
   		**/
		class PutDownCommand extends ManipulationExternalCommand {
			cast::cdl::WorkingMemoryAddress basedObjectAddr;
		};
		
		
		/**
   		* @brief performs a far arm movement to place the gripper in front of the given object
   		* @author Torben Toeniges
   		**/
		class FarArmMovementCommand extends ManipulationExternalCommand {
			cast::cdl::WorkingMemoryAddress targetObjectAddr;
		
			cogx::Math::Pose3 reachedPose;
		
		};
		
		/**
   		* @brief grasps a given visual object with a linear arm movement approach
   		* @author Torben Toeniges
   		**/
		class LinearGraspApproachCommand extends ManipulationExternalCommand {
			cast::cdl::WorkingMemoryAddress targetObjectAddr;
			GraspingStatus graspStatus;
		};
		
		/**
   		* @brief simulates the grasp command
   		* @author Torben Toeniges
   		**/
		class SimulateGraspCommand extends ManipulationExternalCommand {
			cast::cdl::WorkingMemoryAddress targetObjectAddr;

			cogx::Math::Pose3 simulatedReachablePose;
		};
		
		/**
   		* @brief stops the manipulator movement
   		* @author Torben Toeniges
		**/
		class StopCommand extends ManipulationExternalCommand {
		};
		
		/**
   		* @brief stops the manipulator movement
   		* @author Torben Toeniges
		**/
		class MoveArmToHomePositionCommand extends ManipulationExternalCommand {
		};
		
		/**
   		* @brief open gripper
   		* @author Torben Toeniges
		**/
		class OpenGripperCommand extends ManipulationExternalCommand {
		};
		
		/**
   		* @brief close gripper
   		* @author Torben Toeniges
		**/
		class CloseGripperCommand extends ManipulationExternalCommand {
			GraspingStatus graspStatus;
		};
		
		class MoveArmToPose extends ManipulationExternalCommand {
			cogx::Math::Pose3 targetPose;
			cogx::Math::Pose3 reachedPose;			
		};
		
		class GetCurrentArmPose extends ManipulationExternalCommand {
			cogx::Math::Pose3 currentPose;
		};

		/**
			* Golem type set of joint angles
			* @author Michael Zillich
			**/
		sequence<double> ConfigspaceCoord;

		/**
			* Golem type set of joint position and velocity values.
			* Note: we actually ignore velocities, but keep them in order to remain
			* similar to Golem data types.
			* @author Michael Zillich
			**/
		struct GenConfigspaceCoord {
    	ConfigspaceCoord pos;
    	ConfigspaceCoord vel;
		};
		
		/**
			* Golem type set of joint positions (and velocities) with a time stamp
			* @author Michael Zillich
			**/
		struct GenConfigspaceState {
			GenConfigspaceCoord coord;
			double t;
		};
		
		/**
			* Golem type sequence of time stamped joint positions (and velocities),
			* making up a trajectory.
			* @author Michael Zillich
			**/
		sequence<GenConfigspaceState> GenConfigspaceStateSeq;
		
		/**
			* Send trajectory calculated by Golem to a component controlling
			* the arm in the player/gazebo simulated environment.
			* @author Michael Zillich
			*
      * Completion status will be set to ONTHEWAY while executing the trajectory.
      * Completion status will be set to SUCCEEDED upon finishing
      * execution of the trajectory if the last position of the
      * trajectory could actually be reached or FAILED if the arm was blocked
      * due to a collision,
			**/
		class PlayerBridgeSendTrajectoryCommand extends ManipulationInternalCommand {
			GenConfigspaceStateSeq trajectory;
		};

		/**
			* Send open gripper command to a component controlling
			* the arm in the player/gazebo simulated environment.
			* @author Michael Zillich
			*
      * Completion status will be set to ONTHEWAY while opening the gripper.
      * Completion status will be set to SUCCEEDED once the gripper is opened.
			**/
		class PlayerBridgeOpenGripperCommand extends ManipulationInternalCommand {
		};

		/**
			* Send close gripper to a component controlling
			* the arm in the player/gazebo simulated environment.
			* @author Michael Zillich
			*
      * Completion status will be set to ONTHEWAY while closing the gripper.
      * Completion status will be set to SUCCEEDED once the gripper is closed.
			* Grasp status will be set to GRASPING or NOTGRASPING, depending on
			* whether the gripper fingers could fully close.
			**/
		class PlayerBridgeCloseGripperCommand extends ManipulationInternalCommand {
			GraspingStatus graspStatus;
		};
    };
};

#endif

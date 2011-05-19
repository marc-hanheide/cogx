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
   		* This class represents an abstract manipulation command.
   		* The status field can be set to NEW or CHANGED.
   		* The values PENDING, FINISHED and COMMANDFAILED are set as a return 
   		* value by the system.
   		* The completion field can be set to COMPINIT. 
   		* The values FAILED, SUCCEEDED and ONTHEWAY are set as a return value 
   		* by the system.  
   		* @author Torben Toeniges
   		**/
		class ManipulationCommand {
			ManipulationCommandStatus status;			
			ManipulationCompletion comp;
		};
		
		/**
		* This class represents an abstract external manipulation command which
		* can be used.
		* @author Torben Toeniges
		**/
		class ManipulationExternalCommand extends ManipulationCommand {
		};

		/**
		* This class represents an abstract internal manipulation command.
		* All subclasses of this class are only for internal communication and
		* should not be used. 
		* @author Torben Toeniges
		**/
		class ManipulationInternalCommand extends ManipulationCommand {
		};
		
		/**
   		* This command implements a simple put down action. 
   		* It tries to put a given object on a visual object which WM address is
   		* known.
   		* It has to be noticed, that the arm movement planning software return a
   		* valid solution for every visual object most of the time. 
   		* The command itself will not check, if the visual object is in the
   		* range of the manipulator.
   		* It will move to the nearest possible position to the visual object 
   		* and open its gripper 5 cm over the midpoint of the object.
   		* As long the manipulator is moving, the comp field is set to ONTHEWAY 
   		* and the status field to PENDING. 
   		* If a position is reached, the comp field is set to SUCCEEDED and the 
   		* status field to FINISHED.
   		* If an error occurs or another ManipulationExternalCommand is detected, 
   		* the status field is set to COMMANDFAILED and the comp field to FAILED.
   		* @author Torben Toeniges
   		**/
		class PutDownCommand extends ManipulationExternalCommand {
			cast::cdl::WorkingMemoryAddress basedObjectAddr;
	
		};
		
		
		/**
   		* This command represents a simple far arm movement.
   		* It tries to reach the midpoint of the object with a fixed gripper 
   		* orientation (parallel to the floor).
   		* The field reachedPose will return the position the manipulator has 
   		* reached. 
   		* As long the manipulator is moving, the comp field is set to ONTHEWAY 
   		* and the status field to PENDING. 
   		* If a position is reached, the comp field is set to SUCCEEDED and the 
   		* status field to FINISHED.
   		* If an error occurs or another ManipulationExternalCommand is detected,
   		* the status field is set to COMMANDFAILED and the comp field to FAILED.
   		* @author Torben Toeniges
   		**/
		class FarArmMovementCommand extends ManipulationExternalCommand {
			cast::cdl::WorkingMemoryAddress targetObjectAddr;
		
			cogx::Math::Pose3 reachedPose;
		
		};
		
		/**
		* This command represents a simple grasp approach arm movement. 
		* The manipulator will move from the current position to the midpoint of 
		* the object. The orientation will be the same orientation like the 
		* orientation of the movement starting point of this command. 
		* As long the manipulator is moving, the comp field is set to ONTHEWAY 
		* and the status field to PENDING. 
   		* If a position is reached, the comp field is set to SUCCEEDED and the 
   		* status field to FINISHED.
   		* If an error occurs or another ManipulationExternalCommand is detected,
   		* the status field is set to COMMANDFAILED and the comp field to FAILED.
   		* @author Torben Toeniges
   		**/
		class FineArmMovementCommand extends ManipulationExternalCommand {
			cast::cdl::WorkingMemoryAddress targetObjectAddr;
			GraspingStatus graspStatus;
		};
		
		/**
		* This command simulates the movement described in the FarArmMovmentCommand.
		* The field simulatedReachablePose represents the pose of the 
		* manipulator which can be reached with the orientation of the gripper
		* parallel to the floor. 
		* If an error occurs or another ManipulationExternalCommand is detected,
   		* the status field is set to COMMANDFAILED and the comp field to FAILED. 
		* 
   		* @author Torben Toeniges
   		**/
		class SimulateFarArmMovementCommand extends ManipulationExternalCommand {
			cast::cdl::WorkingMemoryAddress targetObjectAddr;

			cogx::Math::Pose3 simulatedReachablePose;
		};
		
		/**
		* This command stops the arm movement.
		* The status field will be set to FINISHED and the comp field to
		* SUCCEEDED, if the stopping was successful.
		* If an error occurs or another ManipulationExternalCommand is detected,
		* the status field is set to COMMANDFAILED and the comp field to FAILED.
   		* @author Torben Toeniges
		**/
		class StopCommand extends ManipulationExternalCommand {
		};
		
		/**
   		* This command moves the arm to its home position (initial position of
   		* the arm after the calibration).
 		* As long the manipulator is moving, the comp field is set to ONTHEWAY 
 		* and the status field to PENDING. 
   		* If a position is reached, the comp field is set to SUCCEEDED and the 
   		* status field to FINISHED.
   		* If an error occurs or another ManipulationExternalCommand is detected,
   		* the status field is set to COMMANDFAILED and the comp field to FAILED.
   		* @author Torben Toeniges
		**/
		class MoveArmToHomePositionCommand extends ManipulationExternalCommand {
		};
		
		/**
		* This command opens the gripper of the manipulator.
		* The status field will be set to FINISHED and the comp field to 
		* SUCCEEDED, if the gripper is opened.
		* If an error occurs or another ManipulationExternalCommand is detected,
		* the status field is set to COMMANDFAILED and the comp field to FAILED.
   		* @author Torben Toeniges
		**/
		class OpenGripperCommand extends ManipulationExternalCommand {
		};
		
		/**
		* This command closes the gripper of the manipulator.
		* The status field will be set to FINISHED and the comp field to 
		* SUCCEEDED, if the gripper is opened.
		* If an error occurs or another ManipulationExternalCommand is detected,
		* the status field is set to COMMANDFAILED and the comp field to FAILED.
   		* If the gripper grasp something, the field graspStatus will be set to 
   		* GRASPING. Otherwise the field is set to NOTGRASPING. 
   		* @author Torben Toeniges
		**/
		class CloseGripperCommand extends ManipulationExternalCommand {
			GraspingStatus graspStatus;
		};
		
		
		/**
		* This command moves the manipulator to a pose (position and rotation) 
		* defined by the field targetPose.
		* The field reachedPose will return the position the manipulator has 
		* reached. 
   		* As long the manipulator is moving, the comp field is set to ONTHEWAY
   		* and the status field to PENDING. 
   		* If a position is reached, the comp field is set to SUCCEEDED and the
   		* status field to FINISHED.
   		* If an error occurs or another ManipulationExternalCommand is detected,
   		* the status field is set to COMMANDFAILED and the comp field to FAILED. 
		* 
		**/
		class MoveArmToPose extends ManipulationExternalCommand {
			cogx::Math::Pose3 targetPose;
			cogx::Math::Pose3 reachedPose;			
		};
		
		/**
		* This command simulates the MoveArmToPose command.
		* The field simulatedReachablePose will return the position the 
		* manipulator would reach in real.  
   		* If an error occurs or another ManipulationExternalCommand is detected,
   		* the status field is set to COMMANDFAILED and the comp field to FAILED. 
		* 
		**/
		class SimulateMoveToPose extends ManipulationExternalCommand {
			cogx::Math::Pose3 targetPose;
			cogx::Math::Pose3 simulatedReachablePose;			
		};
		
		
		
		/**
		* This command returns the current pose of the manipulator.
		* The status field will be set to FINISHED and the comp field to 
		* SUCCEEDED, if the pose can be obtained.
		* If an error occurs or another ManipulationExternalCommand is detected,
		* the status field is set to COMMANDFAILED and the comp field to FAILED.
		*/
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

/**
 * @author Team RED
 */

package manipulation.planner;

import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.FineArmMovementCommand;
import manipulation.slice.GetCurrentArmPose;
import manipulation.slice.ManipulationCommand;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.MoveArmToPose;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateFarArmMovementCommand;
import manipulation.slice.SimulateMoveToPose;
import manipulation.slice.StopCommand;

import VisionData.GripperPose;

import java.util.List;
import java.util.LinkedList;

public class PlanGenerator {

	public static List<ManipulationCommand> generateGrabPlan(GripperPose gp) {

		// Make new plan queue...
		List<ManipulationCommand> plan = new LinkedList<ManipulationCommand>();
		
		// Let's start adding things to the plan queue...
		
		// 1. Open the gripper...
		OpenGripperCommand openGripperCmd = new OpenGripperCommand();
		openGripperCmd.comp = ManipulationCompletion.COMPINIT;
		openGripperCmd.status = ManipulationCommandStatus.NEW;

		plan.add(openGripperCmd);
	       

		// 2. Move to initial gripper pose near the object...
		MoveArmToPose moveArmToPose = new MoveArmToPose();
		moveArmToPose.comp = ManipulationCompletion.COMPINIT;
		moveArmToPose.status = ManipulationCommandStatus.NEW;

		moveArmToPose.targetPose = gp.initialPose;
		
        plan.add(moveArmToPose);
		
        // 3. Move to final gripper pose near the object...
		moveArmToPose = new MoveArmToPose();
		moveArmToPose.comp = ManipulationCompletion.COMPINIT;
		moveArmToPose.status = ManipulationCommandStatus.NEW;

		moveArmToPose.targetPose = gp.finalPose;

		plan.add(moveArmToPose);
	

        // 4. Close the gripper...
        CloseGripperCommand closeGripperCmd = new CloseGripperCommand();
        closeGripperCmd.comp = ManipulationCompletion.COMPINIT;
        closeGripperCmd.status = ManipulationCommandStatus.NEW;

		plan.add(closeGripperCmd);
        
        
        // 5. Move to release pose...
		moveArmToPose = new MoveArmToPose();
		moveArmToPose.comp = ManipulationCompletion.COMPINIT;
		moveArmToPose.status = ManipulationCommandStatus.NEW;

		moveArmToPose.targetPose = gp.releasePose;

		plan.add(moveArmToPose);


		// 6. Open the gripper...
		openGripperCmd = new OpenGripperCommand();
		openGripperCmd.comp = ManipulationCompletion.COMPINIT;
		openGripperCmd.status = ManipulationCommandStatus.NEW;

		plan.add(openGripperCmd);
        
        
        // 7. Move to home position...
        MoveArmToHomePositionCommand moveHomeCmd = new MoveArmToHomePositionCommand();
        moveHomeCmd.comp = ManipulationCompletion.COMPINIT;
        moveHomeCmd.status = ManipulationCommandStatus.NEW;

		plan.add(moveHomeCmd);


		return plan;
	}

	public static List<ManipulationCommand> generateFallbackPlan() {

		// Make new plan queue...
		List<ManipulationCommand> plan = new LinkedList<ManipulationCommand>();

		// 1. Open the gripper...
		OpenGripperCommand openGripperCmd = new OpenGripperCommand();
		openGripperCmd.comp = ManipulationCompletion.COMPINIT;
		openGripperCmd.status = ManipulationCommandStatus.NEW;

		plan.add(openGripperCmd);
		
		// 2. Return to home position
		MoveArmToHomePositionCommand cmd = new MoveArmToHomePositionCommand();
		cmd.comp = ManipulationCompletion.COMPINIT;
		cmd.status = ManipulationCommandStatus.NEW;
		plan.add(cmd);
	       
		return plan;
	}

}
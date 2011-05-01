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

	public static List<ManipulationCommand> generatePlan(GripperPose gp) {
		List<ManipulationCommand> plan = new LinkedList<ManipulationCommand>();
		return plan;
	}

}

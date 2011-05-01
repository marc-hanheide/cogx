/**
 * @author Team RED
 */

package manipulation.planner;

import cast.architecture.ManagedComponent;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ItemException;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.runner.cogx.CogXRunner;
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

import org.apache.log4j.Logger;

import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.GripperPose;
import VisionData.Vertex;
import VisionData.VisualObject;
import VisionData.VisualObjectView;
import cast.AlreadyExistsOnWMException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Rect2;
import cogx.Math.Sphere3;
import cogx.Math.Vector2;
import cogx.Math.Vector3;

import java.util.List;
import java.util.LinkedList;
import java.util.Map;
import java.util.concurrent.ConcurrentLinkedQueue;

public class ManipulationPlanner extends ManagedComponent {

	private ManipulationCommand currentCommand = null;
	private ConcurrentLinkedQueue<ManipulationCommand> currentPlan =
			new ConcurrentLinkedQueue<ManipulationCommand>();

	@Override
	protected void configure(Map<String, String> config) {
		// TODO: config
	}

	@Override
	protected void start() {
		// register GripperPose listener
		addChangeFilter(ChangeFilterFactory.createTypeFilter(
				GripperPose.class, WorkingMemoryOperation.ADD),
				new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleGripperPose(_wmc);
					}
				});
	}

	@Override
	protected void runComponent() {
		while (this.isRunning()) {
			this.sleepComponent(100);
			if (currentCommand != null) {
				if (currentCommand.comp == ManipulationCompletion.SUCCEEDED) {
					log("command succeeded, will execute the next one");
					removeCurrentCommand();

					ManipulationCommand cmd = currentPlan.poll();
					if (cmd != null) {
						addCurrentCommand(cmd);
					}
					else {
						log("plan empty");
					}

				}
				else if (currentCommand.comp == ManipulationCompletion.FAILED) {
					log("command failed, will empty the plan");
					removeCurrentCommand();
					currentPlan.clear();
				}
				else {
					// do nothing
				}
			}
			else {
				ManipulationCommand cmd = currentPlan.poll();
				if (cmd != null) {
					log("starting a plan");
					addCurrentCommand(cmd);
				}
			}
		}
	}

	synchronized void addCurrentCommand(ManipulationCommand cmd) {
		log("adding new command");
		currentCommand = cmd;
		String id = newDataID();

		try {
			addToWorkingMemory(id, cmd);
		}
		catch (AlreadyExistsOnWMException ex) {
			log(ex);
		}

		// TODO: register the change filter for the cmd
	}

	synchronized void overwriteCurrentCommand(ManipulationCommand cmd) {
		currentCommand = cmd;
	}

	synchronized void removeCurrentCommand() {
		log("removing current command");

		// TODO: unregister the change filter

		currentCommand = null;
	}

	public void handleGripperPose(WorkingMemoryChange wmc) {
		List<ManipulationCommand> newPlan = new LinkedList<ManipulationCommand>();

		// TODO: generate the plan

		log("adding plan of length " + newPlan.size() + " to the overall plan");
		currentPlan.addAll(newPlan);
	}

	public void handleCommandOverwrite(WorkingMemoryChange wmc) {
		log("detected a change on address [" + wmc.address.id + "]");
		// TODO: overwrite currentCommand with the newly changed object
	}

}

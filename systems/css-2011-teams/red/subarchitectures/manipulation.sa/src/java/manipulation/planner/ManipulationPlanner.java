/**
 * @author Team RED
 */

package manipulation.planner;

import cast.architecture.ManagedComponent;

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
import cast.SubarchitectureComponentException;
import cast.CASTException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryOperation;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTData;
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

	private WorkingMemoryChangeReceiver recv = null;

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
					log("yay! command succeeded!");
					removeCurrentCommand();
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
					log("plan nonempty (" + currentPlan.size() + " items left)");
					addCurrentCommand(cmd);
				}
				else {
					// plan empty, do nothing
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

		if (recv == null) {

			recv = new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						handleCommandOverwrite(_wmc);
					}
			};

			addChangeFilter(ChangeFilterFactory.createAddressFilter(
					id, getSubarchitectureID(), WorkingMemoryOperation.OVERWRITE),
					recv);
		}
		else {
			log("WARNING WARNING NUCLEAR ATTACK, recv != NULL (I'm not registering it) !!!");
		}
	}

	synchronized void overwriteCurrentCommand(ManipulationCommand cmd) {
		currentCommand = cmd;
	}

	synchronized void removeCurrentCommand() {
		log("removing current command");

		try {
			removeChangeFilter(recv);
		}
		catch (SubarchitectureComponentException ex) {
			log(ex);
		}

		recv = null;

		currentCommand = null;
	}

	public void handleGripperPose(WorkingMemoryChange wmc) {
		List<ManipulationCommand> newPlan = PlanGenerator.generatePlan(null);
		if (newPlan != null) {
			log("adding plan of length " + newPlan.size() + " to the overall plan");
			currentPlan.addAll(newPlan);
		}
		else {
			log("got a NULL plan, ignoring");
		}
	}

	public void handleCommandOverwrite(WorkingMemoryChange wmc) {
		log("woohoo, detected a change on address [" + wmc.address.id + "]");
		try {
			CASTData data = getWorkingMemoryEntry(wmc.address.id);
			if (data.getData() instanceof ManipulationCommand) {
				ManipulationCommand cmd = (ManipulationCommand) data.getData();
				overwriteCurrentCommand(cmd);
			}
			else {
				log("oops, got something that isn't a ManipulationCommand (ignoring this)!");
			}
		}
		catch (SubarchitectureComponentException ex) {
			log(ex);
		}
	}

}

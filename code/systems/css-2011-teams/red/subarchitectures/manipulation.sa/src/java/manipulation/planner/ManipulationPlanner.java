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

import de.dfki.lt.tr.dialogue.slice.synthesize.SpokenOutputItem;
import de.dfki.lt.tr.dialogue.slice.ref.NominalReference;

import mathlib.Functions;

import VisionData.Face;
import VisionData.GeometryModel;
import VisionData.GripperPose;
import VisionData.Vertex;
import VisionData.VisualObject;
import VisionData.VisualObjectView;
import cast.AlreadyExistsOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
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

	private boolean verbalization = false;
	private static final String verbalization_sa = "dialogue";

	@Override
	protected void configure(Map<String, String> config) {
		if (config.containsKey("--verbalize")) {
			verbalization = true;
		}
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
		verbalize("Okay. I think I'm ready.");

		while (this.isRunning()) {
			this.sleepComponent(200);
			if (currentCommand != null) {
				boolean fail = false;

				if (currentCommand.comp == ManipulationCompletion.SUCCEEDED
						|| currentCommand.comp == ManipulationCompletion.FAILED) {
					
					if (currentCommand.comp == ManipulationCompletion.SUCCEEDED) {
						if (currentCommand instanceof MoveArmToPose) {
							MoveArmToPose matp = (MoveArmToPose) currentCommand;
							fail = !isPoseFine(matp.targetPose, matp.reachedPose);
							log("failed to reached the target position");
						}
						if (!fail) {
							log("yay! command succeeded!");
							removeCurrentCommand();
						}
					}
					if (fail || currentCommand.comp == ManipulationCompletion.FAILED) {
						log("command failed, will execute the fallback plan");
						verbalize("Damn. I failed to " + commandToInfString(currentCommand) + ".");
						removeCurrentCommand();
						currentPlan.clear();

						List<ManipulationCommand> newPlan = PlanGenerator.generateFallbackPlan();
						if (newPlan != null) {
							log("adding plan of length " + newPlan.size() + " to the overall plan");
							currentPlan.addAll(newPlan);
						}
						else {
							log("got a NULL plan, ignoring");
						}
					}
				}
				else {
					// do nothing
				}
			}
			else {
				ManipulationCommand cmd = currentPlan.poll();
				if (cmd != null) {
					log("plan nonempty (" + currentPlan.size() + " items left)");
					verbalize("I will " + commandToInfString(currentCommand) + " now.");
					addCurrentCommand(cmd);
				}
				else {
					// plan empty
					log("Okay, I'm done.");
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
		GripperPose gp = null;
		try {
			CASTData data = getWorkingMemoryEntry(wmc.address.id);
			if (data.getData() instanceof GripperPose) {
				verbalize("Right. I see the object now.");

				gp = (GripperPose) data.getData();

				log("got the following gripper pose:\n" 
						+ "initial = " + Functions.toString(gp.initialPose) + "\n"
						+ "final   = " + Functions.toString(gp.finalPose) + "\n"
						+ "release = " + Functions.toString(gp.releasePose));

				List<ManipulationCommand> newPlan = PlanGenerator.generateGrabPlan(gp);

				if (newPlan != null) {
					log("adding plan of length " + newPlan.size() + " to the overall plan");
					currentPlan.addAll(newPlan);
				}
				else {
					log("got a NULL plan, ignoring");
				}
			}
			else {
				log("oops, got something that isn't a GripperPose (ignoring this)!");
			}
		}
		catch (SubarchitectureComponentException ex) {
			log(ex);
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

	private void verbalize(String s) {
		if (verbalization) {
			try {
				log("will try to verbalize \"" + s + "\"");
				String id = newDataID();
				SpokenOutputItem soi = new SpokenOutputItem(id, s, "", new NominalReference());
				addToWorkingMemory(id, verbalization_sa, soi);
			}
			catch (AlreadyExistsOnWMException ex) {
				log(ex);
			}
			catch (UnknownSubarchitectureException ex) {
				log(ex);
			}
		}
	}

	private String commandToInfString(ManipulationCommand cmd) {
		if (cmd instanceof CloseGripperCommand) {
			return "close the gripper";
		}
		if (cmd instanceof FarArmMovementCommand) {
			return "stretch the arm far away";
		}
		if (cmd instanceof FineArmMovementCommand) {
			return "move the arm with some finesse";
		}
		if (cmd instanceof MoveArmToHomePositionCommand) {
			return "return the arm to the home position";
		}
		if (cmd instanceof MoveArmToPose) {
			return "move the arm";
		}
		if (cmd instanceof OpenGripperCommand) {
			return "close the gripper";
		}
		if (cmd instanceof PutDownCommand) {
			return "put down the object";
		}
		if (cmd instanceof SimulateFarArmMovementCommand) {
			return "simulate the arm stretching";
		}
		if (cmd instanceof SimulateMoveToPose) {
			return "simulate the arm move";
		}
		if (cmd instanceof StopCommand) {
			return "stop the arm";
		}

		// fallback
		return "do stuff";
	}

	private static boolean isPoseFine(Pose3 intended, Pose3 reached) {
		return true;
	}

}

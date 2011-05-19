package springschool;

import java.util.ArrayList;
import java.util.List;

import manipulation.slice.CloseGripperCommand;
import manipulation.slice.GraspingStatus;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.ManipulationExternalCommand;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.MoveArmToPose;
import manipulation.slice.OpenGripperCommand;
import mathlib.Functions;
import ptz.PTZCompletion;
import ptz.PTZPose;
import ptz.SetPTZPoseCommand;
import NavData.RobotPose2d;
import SpatialData.CommandType;
import SpatialData.Completion;
import SpatialData.NavCommand;
import SpatialData.Place;
import SpatialData.Priority;
import SpatialData.StatusError;
import VisionData.DetectionCommand;
import VisionData.VisualObject;
import cast.CASTException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.castextensions.WMEventQueue;
import cogx.Math.Matrix33;
import cogx.Math.Pose3;
import cogx.Math.Vector3;

public final class SimpleInterface {

	ManagedComponent component;

	public SimpleInterface(ManagedComponent component) {
		this.component = component;
	}

	public static NavCommand createNavCommand() {
		NavCommand nc = new NavCommand(CommandType.GOTOPOSITION,
				Priority.NORMAL, null, null, null, null, null,
				StatusError.NONE, Completion.COMMANDPENDING);
		nc.pose = new double[3];
		nc.pose[0] = 0.0;
		nc.pose[1] = 0.0;
		nc.pose[2] = 0.0;

		nc.tolerance = new double[3];
		nc.tolerance[0] = 0.1;
		nc.tolerance[1] = 0.1;
		nc.tolerance[2] = Math.PI * 10.0 / 180.0;

		nc.destId = new long[1];
		nc.destId[0] = 0;
		nc.distance = new double[1];
		nc.angle = new double[1];
		return nc;
	}

	public static SetPTZPoseCommand createPtzCommand() {
		return new SetPTZPoseCommand(new PTZPose(0, 0, 0),
				PTZCompletion.COMPINIT);
	}

	public static DetectionCommand createDetectCommand() {
		return new DetectionCommand(new String[1]);
	}

	public VisualObject executeDetectCommand(String label) {
		DetectionCommand dc = createDetectCommand();
		dc.labels[0] = label;
		return executeDetectCommand(dc);
	}

	public NavCommand executeNavCommand(NavCommand nc) {
		WorkingMemoryAddress wma = new WorkingMemoryAddress(component
				.newDataID(), "spatial.sa");
		WMEventQueue eq = new WMEventQueue();

		component.addChangeFilter(ChangeFilterFactory.createAddressFilter(wma,
				WorkingMemoryOperation.OVERWRITE), eq);
		try {
			component.addToWorkingMemory(wma, nc);
			while (true) {
				WorkingMemoryChange wmc = eq.take();
				NavCommand changedNc = component.getMemoryEntry(wmc.address,
						NavCommand.class);
				component.log("NavCommand status update: " + changedNc.comp);
				switch (changedNc.comp) {
				case COMMANDABORTED:
				case COMMANDFAILED:
				case COMMANDSUCCEEDED:
					component.removeChangeFilter(eq);
					// component.deleteFromWorkingMemory(wma);
					return changedNc;
				}

			}
		} catch (CASTException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}
		return null;
	}

	public SetPTZPoseCommand executePTZCommand(double pan, double tilt) {
		SetPTZPoseCommand pc = createPtzCommand();
		pc.pose.pan = pan;
		pc.pose.tilt = tilt;
		return executePTZCommand(pc);
	}

	public SetPTZPoseCommand executePTZCommand(SetPTZPoseCommand nc) {
		WorkingMemoryAddress wma = new WorkingMemoryAddress(component
				.newDataID(), "vision.sa");
		WMEventQueue eq = new WMEventQueue();

		component.addChangeFilter(ChangeFilterFactory.createAddressFilter(wma,
				WorkingMemoryOperation.OVERWRITE), eq);
		try {
			component.addToWorkingMemory(wma, nc);
			while (true) {
				WorkingMemoryChange wmc = eq.take();
				SetPTZPoseCommand changedNc = component.getMemoryEntry(
						wmc.address, SetPTZPoseCommand.class);
				component.log("SetPTZPoseCommand status update: "
						+ changedNc.comp);
				switch (changedNc.comp) {
				case FAILED:
				case SUCCEEDED:
					component.removeChangeFilter(eq);
					component.deleteFromWorkingMemory(wma);
					return changedNc;
				}

			}
		} catch (CASTException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}
		return null;
	}

	public MoveArmToPose reachForPose(Pose3 pose) {
		ManipulationExternalCommand nc = new MoveArmToPose(
				ManipulationCommandStatus.NEW, ManipulationCompletion.COMPINIT,
				pose, pose);
		return (MoveArmToPose) executeArmCommand(nc);
	}

	public MoveArmToPose reachForPosition(Vector3 vec) {
		Matrix33 rot = new Matrix33(0, 1, 0, -1, 0, 0, 0, 0, 1);
		Pose3 p = new Pose3(vec, rot);
		ManipulationExternalCommand nc = new MoveArmToPose(
				ManipulationCommandStatus.NEW, ManipulationCompletion.COMPINIT,
				p, p);
		return (MoveArmToPose) executeArmCommand(nc);
	}

	public MoveArmToHomePositionCommand moveArmHome() {
		ManipulationExternalCommand nc = new MoveArmToHomePositionCommand(
				ManipulationCommandStatus.NEW, ManipulationCompletion.COMPINIT);
		return (MoveArmToHomePositionCommand) executeArmCommand(nc);
	}

	public CloseGripperCommand closeGripper() {
		ManipulationExternalCommand nc = new CloseGripperCommand(
				ManipulationCommandStatus.NEW, ManipulationCompletion.COMPINIT,
				GraspingStatus.GRASPINGSTATUSINIT);
		return (CloseGripperCommand) executeArmCommand(nc);
	}

	public OpenGripperCommand openGripper() {
		ManipulationExternalCommand nc = new OpenGripperCommand(
				ManipulationCommandStatus.NEW, ManipulationCompletion.COMPINIT);
		return (OpenGripperCommand) executeArmCommand(nc);
	}

	public NavCommand navigateToPlace(Place p) {
		NavCommand nc = createNavCommand();
		nc.cmd = CommandType.GOTOPLACE;
		nc.destId[0] = p.id;
		return executeNavCommand(nc);
	}

	public Pose3 getRobotPose() throws UnknownSubarchitectureException {
		List<RobotPose2d> poses = new ArrayList<RobotPose2d>();
		component.getMemoryEntries(RobotPose2d.class, poses, "spatial.sa");
		if (poses.size() < 1) {
			component.getLogger().error("no pose");
			return null;
		}
		RobotPose2d pose = poses.get(0);
		return Functions.pose3FromEuler(new Vector3(pose.x, pose.y, 0), 0, 0,
				pose.theta);
	}

	public double getRobotTheta() throws UnknownSubarchitectureException {
		List<RobotPose2d> poses = new ArrayList<RobotPose2d>();
		component.getMemoryEntries(RobotPose2d.class, poses, "spatial.sa");
		if (poses.size() < 1) {
			component.getLogger().error("no pose");
			return -1;
		}
		RobotPose2d pose = poses.get(0);
		return pose.theta;
	}

	public NavCommand navigateToPos(double x, double y, double theta) {
		NavCommand nc = createNavCommand();
		nc.cmd = CommandType.GOTOPOSITION;
		nc.pose[0] = x;
		nc.pose[1] = y;
		nc.pose[2] = theta;
		return executeNavCommand(nc);
	}

	public NavCommand turnBy(double angle) {
		NavCommand nc = createNavCommand();
		nc.cmd = CommandType.TURN;
		nc.angle[0] = angle;
		return executeNavCommand(nc);
	}

	public ManipulationExternalCommand executeArmCommand(
			ManipulationExternalCommand nc) {
		WorkingMemoryAddress wma = new WorkingMemoryAddress(component
				.newDataID(), "manipulation.sa");
		WMEventQueue eq = new WMEventQueue();

		component.addChangeFilter(ChangeFilterFactory.createAddressFilter(wma,
				WorkingMemoryOperation.OVERWRITE), eq);
		try {
			component.addToWorkingMemory(wma, nc);
			while (true) {
				WorkingMemoryChange wmc = eq.take();
				ManipulationExternalCommand changedNc = component
						.getMemoryEntry(wmc.address,
								ManipulationExternalCommand.class);
				component.log("ManipulationExternalCommand status update: "
						+ changedNc.comp);
				switch (changedNc.comp) {
				case FAILED:
				case SUCCEEDED:
					component.removeChangeFilter(eq);
					//component.deleteFromWorkingMemory(wma);
					return changedNc;
				}

			}
		} catch (CASTException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}
		return null;
	}

	public VisualObject executeDetectCommand(DetectionCommand nc) {
		WorkingMemoryAddress wma = new WorkingMemoryAddress(component
				.newDataID(), "vision.sa");

		WMEventQueue eq = new WMEventQueue();

		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				VisualObject.class, WorkingMemoryOperation.ADD), eq);
		component.addChangeFilter(ChangeFilterFactory.createTypeFilter(
				VisualObject.class, WorkingMemoryOperation.OVERWRITE), eq);
		try {
			component.addToWorkingMemory(wma, nc);
			WorkingMemoryChange wmc = eq.take();
			VisualObject vo = component.getMemoryEntry(wmc.address,
					VisualObject.class);
			component.removeChangeFilter(eq);
			// component.deleteFromWorkingMemory(wma);
			component
					.log("VisualObject showed up (it's not guaranteed to be the corresponding one!)");
			return vo;
		} catch (CASTException e) {
			component.logException(e);
		} catch (InterruptedException e) {
			component.logException(e);
		}
		return null;
	}

}

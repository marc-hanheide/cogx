package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.commandWatcher.CommandWatcher.ArmReachingStatus;
import manipulation.core.cogx.converter.CogXConverter;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.FineArmMovementCommand;
import manipulation.slice.GetCurrentArmPose;
import manipulation.slice.GraspingStatus;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.ManipulationExternalCommand;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.MoveArmToPose;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateFarArmMovementCommand;
import manipulation.slice.SimulateMoveToPose;
import manipulation.slice.StopCommand;
import manipulation.strategies.CommandExecution;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;
import mathlib.Functions;

import org.apache.log4j.Logger;

import cogx.Math.Pose3;
import cogx.Math.Vector3;

import VisionData.VisualObject;
import cast.SubarchitectureComponentException;
import cast.cdl.WorkingMemoryAddress;

/**
 * defines a behaviour to linear approach an object with the gripper
 * 
 * @author Torben Toeniges
 * 
 */
public class FineArmMovementCommandPart extends StrategyPart implements
		Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	private boolean manipulationFailed = false;

	public FineArmMovementCommandPart(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FINE_ARM_MOVEMENT_COMMAND_PART);
	}

	private void fineApproach() {
		WorkingMemoryAddress wma = ((FineArmMovementCommand) ((CommandExecution) getGlobalStrategy())
				.getCurrentCommand()).targetObjectAddr;

		try {
			VisualObject targetVisOb = (((CogXRunner) getManipulator()
					.getRunner()).getMemoryEntry(wma, VisualObject.class));

			Vector3D currentGoalPosition = new Vector3D(targetVisOb.pose.pos.x,
					targetVisOb.pose.pos.y, targetVisOb.pose.pos.z);

			Matrix currentArmRot = getManipulator().getArmConnector()
					.getCurrentRotation();

			Pose3 pInRob = CogXConverter.convertToPose3(currentGoalPosition,
					currentArmRot);

			Vector2D position = getManipulator().getBaseConnector()
					.getCurrentPosition().getPoint();

			Pose3 tRobotToWorld = Functions.pose3FromEuler(
					new Vector3(position.getX(), position.getY(), 0), 0.0, 0.0,
					getManipulator().getBaseConnector().getCurrentPosition()
							.getAngle());

			Pose3 pInWorld = Functions.transform(tRobotToWorld, pInRob);

			getManipulator().getArmConnector().reach(
					CogXConverter.convertPose3ToPose(pInWorld));

		} catch (SubarchitectureComponentException e) {
			logger.error(e);
			manipulationFailed = true;
			return;
		} catch (ManipulatorException e) {
			logger.error(e);
			manipulationFailed = true;
			return;
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		getManipulator().getWatcher().addObserver(this);
		manipulationFailed = false;

		fineApproach();

		if (!manipulationFailed) {
			synchronized (this) {
				try {
					wait();
				} catch (InterruptedException e) {
					logger.error(e);
				}
			}
		} else {
			FineArmMovementCommand currentCom = ((FineArmMovementCommand) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.COMMANDFAILED;
			currentCom.comp = ManipulationCompletion.FAILED;
			currentCom.graspStatus = GraspingStatus.NOTGRASPING;

			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getNewCommandAddress(), currentCom);

			setNextPartName(PartName.WAIT_PART);
		}

		logger.debug("we go on!");

		changeToNextPart();

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {
		getManipulator().getWatcher().deleteObserver(this);

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));
	}

	/**
	 * 
	 * {@inheritDoc}
	 */
	@Override
	public void update(Observable observable, Object arg) {
		if (observable instanceof CommandWatcher) {
			if (arg instanceof ArmReachingStatus) {
				if ((ArmReachingStatus) arg == ArmReachingStatus.REACHED) {
					logger.info("reached position with the arm");
					setNextPartName(PartName.WAIT_PART);

					logger.info("close gripper");
					getManipulator().getArmConnector().closeGripper(10);

					FineArmMovementCommand currentCom = ((FineArmMovementCommand) ((CommandExecution) getGlobalStrategy())
							.getCurrentCommand());
					currentCom.status = ManipulationCommandStatus.FINISHED;
					currentCom.comp = ManipulationCompletion.SUCCEEDED;

					if (getManipulator().getArmConnector().isGraspingObject()) {
						logger.info("Object in the gripper");
						currentCom.graspStatus = GraspingStatus.GRASPING;
					} else {
						logger.info("Object not in the gripper");
						currentCom.graspStatus = GraspingStatus.NOTGRASPING;
					}

					((CogXRunner) (getManipulator().getRunner()))
							.updateWorkingMemoryCommand(getManipulator()
									.getWatcher().getNewCommandAddress(),
									currentCom);

					synchronized (this) {
						notifyAll();
					}
				}
			}
			if (arg instanceof ManipulationExternalCommand) {
				ManipulationExternalCommand currentCom = ((CommandExecution) getGlobalStrategy())
						.getCurrentCommand();
				currentCom.status = ManipulationCommandStatus.COMMANDFAILED;
				currentCom.comp = ManipulationCompletion.FAILED;

				WorkingMemoryAddress address = getManipulator().getWatcher()
						.getLastCommandAddress();

				if (address != null) {
					((CogXRunner) (getManipulator().getRunner()))
							.updateWorkingMemoryCommand(getManipulator()
									.getWatcher().getLastCommandAddress(),
									currentCom);
				} else {
					((CogXRunner) (getManipulator().getRunner()))
							.updateWorkingMemoryCommand(getManipulator()
									.getWatcher().getNewCommandAddress(),
									currentCom);
				}

				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationExternalCommand) arg);
			}

			if (arg instanceof FarArmMovementCommand) {
				logger.info("far arm movement command");
				setNextPartName(PartName.FAR_ARM_MOVEMENT_COMMAND_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof PutDownCommand) {
				logger.info("put down command");
				setNextPartName(PartName.PUT_DOWN_COMMAND_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof FineArmMovementCommand) {
				logger.info("linear grasp approach command");
				setNextPartName(PartName.FINE_ARM_MOVEMENT_COMMAND_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof SimulateFarArmMovementCommand) {
				logger.info("simulate grasp command");
				setNextPartName(PartName.SIMULATE_FAR_ARM_MOVEMENT_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof StopCommand) {
				logger.info("stop command");
				setNextPartName(PartName.STOP_COMMAND_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof MoveArmToHomePositionCommand) {
				logger.info("move arm to home position command");
				setNextPartName(PartName.MOVE_ARM_TO_HOME_POSITION_COMMAND_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof OpenGripperCommand) {
				logger.info("open gripper command");
				setNextPartName(PartName.OPEN_GRIPPER_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof CloseGripperCommand) {
				logger.info("close gripper command");
				setNextPartName(PartName.CLOSE_GRIPPER_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof MoveArmToPose) {
				logger.info("move arm to pose command");
				setNextPartName(PartName.MOVE_ARM_TO_POSE_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof GetCurrentArmPose) {
				logger.info("get current pose command");
				setNextPartName(PartName.GET_CURRENT_ARM_POSE_PART);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof SimulateMoveToPose) {
				logger.info("simulate move to pose command");
				setNextPartName(PartName.SIMULATE_MOVE_TO_POSE_PART);
				synchronized (this) {
					notifyAll();
				}
			}
		}
	}
}

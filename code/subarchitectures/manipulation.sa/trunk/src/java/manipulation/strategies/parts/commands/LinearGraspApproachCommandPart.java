package manipulation.strategies.parts.commands;

import java.util.HashMap;
import java.util.Observable;
import java.util.Observer;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.exceptions.ViewPointException;
import manipulation.core.share.types.ArmError;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.ViewPoint;
import manipulation.core.share.types.ViewPoints;
import manipulation.core.share.types.SensorData.SensorPosition;
import manipulation.itemMemory.Item;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.LinearBaseMovementApproachCommand;
import manipulation.slice.LinearGraspApproachCommand;
import manipulation.slice.ManipulationCommand;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateGraspCommand;
import manipulation.slice.StopCommand;
import manipulation.strategies.CommandExecution;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;
import manipulation.strategies.parts.StrategyPart.PartName;

import org.apache.log4j.Logger;

import VisionData.VisualObject;

/**
 * defines a behaviour to linear approach an object with the gripper
 * 
 * @author ttoenige
 * 
 */
public class LinearGraspApproachCommandPart extends StrategyPart implements
		Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	public LinearGraspApproachCommandPart(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.LINEAR_GRASP_APPROACH_COMMAND_PART);
	}

	private void fineApproach() {

		VisualObject targetVisOb = ((FarArmMovementCommand) ((CommandExecution) getGlobalStrategy())
				.getCurrentCommand()).targetObject;

		Vector3D currentGoalPosition = new Vector3D(targetVisOb.pose.pos.x,
				targetVisOb.pose.pos.y, targetVisOb.pose.pos.z);

		try {
			Matrix currentArmRot = getManipulator().getArmConnector()
					.getCurrentRotation();

			getManipulator().getArmConnector().reach(currentGoalPosition,
					currentArmRot);

		} catch (ManipulatorException e3) {
			logger.error("ANFANG FEHLER");
		}

		getManipulator().getArmConnector().closeGripper(10);

		if (getManipulator().getArmConnector().isGraspingObject()) {
			logger.error("Object in the gripper");
		} else {
			logger.error("Object not in the gripper");
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		getManipulator().getWatcher().addObserver(this);

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);
			}
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

			if (arg instanceof ManipulationCommand) {
				ManipulationCommand currentCom = ((CommandExecution) getGlobalStrategy())
						.getCurrentCommand();
				currentCom.status = ManipulationCommandStatus.COMMANDFAILED;
				currentCom.comp = ManipulationCompletion.FAILED;
				((CogXRunner) (getManipulator().getRunner()))
						.updateWorkingMemoryCommand(getManipulator()
								.getWatcher().getCurrentCommandAddress(),
								currentCom);
			}

			if (arg instanceof FarArmMovementCommand) {
				logger.info("far arm movement command");

				setNextPartName(PartName.FAR_ARM_MOVEMENT_COMMAND_PART);
				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationCommand) arg);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof PutDownCommand) {
				logger.info("put down command");

				setNextPartName(PartName.PUT_DOWN_COMMAND_PART);
				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationCommand) arg);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof LinearGraspApproachCommand) {
				logger.info("linear grasp approach command");

				setNextPartName(PartName.LINEAR_GRASP_APPROACH_COMMAND_PART);
				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationCommand) arg);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof SimulateGraspCommand) {
				logger.info("simulate grasp command");

				setNextPartName(PartName.SIMULATE_GRASP_COMMAND_PART);
				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationCommand) arg);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof LinearBaseMovementApproachCommand) {
				logger.info("linear base movement approach command");

				setNextPartName(PartName.LINEAR_BASE_MOVEMENT_APPROACH_COMMAND_PART);
				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationCommand) arg);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof StopCommand) {
				logger.info("stop command");

				setNextPartName(PartName.STOP_COMMAND_PART);
				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationCommand) arg);
				synchronized (this) {
					notifyAll();
				}
			} else if (arg instanceof MoveArmToHomePositionCommand) {
				logger.info("move arm to home position command");

				setNextPartName(PartName.MOVE_ARM_TO_HOME_POSITION_COMMAND_PART);
				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationCommand) arg);
				synchronized (this) {
					notifyAll();
				}
			}
		}
	}
}

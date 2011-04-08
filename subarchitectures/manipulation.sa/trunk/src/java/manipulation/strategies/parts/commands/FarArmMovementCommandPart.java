package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.ArmError;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.math.MathOperation;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.LinearBaseMovementApproachCommand;
import manipulation.slice.LinearGraspApproachCommand;
import manipulation.slice.ManipulationCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateGraspCommand;
import manipulation.strategies.CommandExecution;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

import VisionData.VisualObject;

/**
 * defines a behaviour to reach a position in front of an object with the arm
 * 
 * @author ttoenige
 * 
 */
public class FarArmMovementCommandPart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	public FarArmMovementCommandPart(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FAR_ARM_MOVEMENT_COMMAND_PART);
	}

	private void farGraspApproach() {

		VisualObject targetVisOb = ((FarArmMovementCommand) ((CommandExecution) getGlobalStrategy())
				.getCurrentCommand()).targetObject;

		Vector3D currentGoalPosition = new Vector3D(targetVisOb.pose.pos.x,
				targetVisOb.pose.pos.y, targetVisOb.pose.pos.z);

		Vector3D currentArmPos = null;
		try {
			currentArmPos = getManipulator().getArmConnector()
					.getCurrentPosition();
		} catch (ManipulatorException e) {
			logger.error(e);
		}

		double posInFront = 0.1;

		Vector3D direction = MathOperation.getDirection(currentArmPos,
				currentGoalPosition);

		logger.error("Grasp the item!");

		Matrix rotation1 = MathOperation.getRotationAroundX(MathOperation
				.getRadiant(0));

		Matrix rotation2 = MathOperation.getRotationAroundY(MathOperation
				.getRadiant(0));

		Matrix rotation3 = MathOperation.getRotationAroundZ(MathOperation
				.getRadiant(-90));

		Matrix greifRotation = MathOperation.getMatrixMatrixMultiplication(
				MathOperation.getMatrixMatrixMultiplication(rotation1,
						rotation2), rotation3);

		Vector3D goalWithDistance = new Vector3D(
				(currentGoalPosition.getX() - posInFront * direction.getX()),
				currentGoalPosition.getY() - posInFront * direction.getY(),
				currentGoalPosition.getZ());

		ArmError armError = null;

		try {
			armError = getManipulator().getArmConnector().getPosError(
					goalWithDistance, greifRotation);
		} catch (ManipulatorException e) {
			logger.error(e);
		}

		try {
			getManipulator().getArmConnector().reach(goalWithDistance,
					greifRotation);
		} catch (ManipulatorException e) {
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

		farGraspApproach();

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
			}
		}
	}
}

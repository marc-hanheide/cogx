package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.commandWatcher.CommandWatcher.ArmReachingStatus;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
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

import org.apache.log4j.Logger;

import VisionData.VisualObject;

/**
 * defines a behaviour to put an object on another object
 * 
 * @author ttoenige
 * 
 */
public class PutDownCommandPart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	private boolean manipulationFailed = false;

	public PutDownCommandPart(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.PUT_DOWN_COMMAND_PART);
	}

	private void putDownApproach() {

		VisualObject targetVisOb = ((FarArmMovementCommand) ((CommandExecution) getGlobalStrategy())
				.getCurrentCommand()).targetObject;

		Vector3D currentGoalPosition = new Vector3D(targetVisOb.pose.pos.x,
				targetVisOb.pose.pos.y, targetVisOb.pose.pos.z);

		double posOver = 0.1;

		Matrix rotation1 = MathOperation.getRotationAroundX(MathOperation
				.getRadiant(0));
		Matrix rotation2 = MathOperation.getRotationAroundY(MathOperation
				.getRadiant(0));
		Matrix rotation3 = MathOperation.getRotationAroundZ(MathOperation
				.getRadiant(-90));
		Matrix graspRotation = MathOperation.getMatrixMatrixMultiplication(
				MathOperation.getMatrixMatrixMultiplication(rotation1,
						rotation2), rotation3);

		Vector3D goalWithDistance = new Vector3D(currentGoalPosition.getX(),
				currentGoalPosition.getY(), currentGoalPosition.getZ()
						+ posOver);

		try {
			getManipulator().getArmConnector().reach(goalWithDistance,
					graspRotation);

			PutDownCommand currentCom = ((PutDownCommand) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.PENDING;
			currentCom.comp = ManipulationCompletion.ONTHEWAY;

			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);
		} catch (ManipulatorException e) {
			logger.error(e);
			manipulationFailed = true;
			return;
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

		putDownApproach();

		if (!manipulationFailed) {
			synchronized (this) {
				try {
					wait();
				} catch (InterruptedException e) {
					logger.error(e);
				}
			}
		} else {
			PutDownCommand currentCom = ((PutDownCommand) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.COMMANDFAILED;
			currentCom.comp = ManipulationCompletion.FAILED;

			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);

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

					logger.info("open gripper");
					getManipulator().getArmConnector().openGripper();

					setNextPartName(PartName.WAIT_PART);
					PutDownCommand currentCom = ((PutDownCommand) ((CommandExecution) getGlobalStrategy())
							.getCurrentCommand());
					currentCom.status = ManipulationCommandStatus.FINISHED;

					((CogXRunner) (getManipulator().getRunner()))
							.updateWorkingMemoryCommand(getManipulator()
									.getWatcher().getCurrentCommandAddress(),
									currentCom);

					synchronized (this) {
						notifyAll();
					}
				}
			}
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

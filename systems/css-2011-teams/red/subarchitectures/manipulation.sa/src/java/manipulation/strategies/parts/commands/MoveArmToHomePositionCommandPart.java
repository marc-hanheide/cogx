package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.commandWatcher.CommandWatcher.ArmReachingStatus;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.FineArmMovementCommand;
import manipulation.slice.GetCurrentArmPose;
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

import org.apache.log4j.Logger;

/**
 * defines a behaviour to reach the home position of the manipulator
 * 
 * @author Torben Toeniges
 * 
 */
public class MoveArmToHomePositionCommandPart extends StrategyPart implements
		Observer {

	private Logger logger = Logger.getLogger(this.getClass());
	private boolean manipulationFailed = false;

	public MoveArmToHomePositionCommandPart(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.MOVE_ARM_TO_HOME_POSITION_COMMAND_PART);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());
		manipulationFailed = false;

		getManipulator().getWatcher().addObserver(this);

		try {
			getManipulator().getArmConnector().goHome();

			MoveArmToHomePositionCommand currentCom = ((MoveArmToHomePositionCommand) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.PENDING;
			currentCom.comp = ManipulationCompletion.ONTHEWAY;

			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);

		} catch (ManipulatorException e1) {
			manipulationFailed = true;
		}

		if (!manipulationFailed) {
			synchronized (this) {
				try {
					wait();
				} catch (InterruptedException e) {
					logger.error(e);
				}
			}
		} else {
			MoveArmToHomePositionCommand currentCom = ((MoveArmToHomePositionCommand) ((CommandExecution) getGlobalStrategy())
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
					setNextPartName(PartName.WAIT_PART);
					MoveArmToHomePositionCommand currentCom = ((MoveArmToHomePositionCommand) ((CommandExecution) getGlobalStrategy())
							.getCurrentCommand());
					currentCom.status = ManipulationCommandStatus.FINISHED;
					currentCom.comp = ManipulationCompletion.SUCCEEDED;

					((CogXRunner) (getManipulator().getRunner()))
							.updateWorkingMemoryCommand(getManipulator()
									.getWatcher().getCurrentCommandAddress(),
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
				((CogXRunner) (getManipulator().getRunner()))
						.updateWorkingMemoryCommand(getManipulator()
								.getWatcher().getCurrentCommandAddress(),
								currentCom);

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

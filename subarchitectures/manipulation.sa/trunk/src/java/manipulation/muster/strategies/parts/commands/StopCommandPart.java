package manipulation.muster.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.muster.commandWatcher.CommandWatcher;
import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.exceptions.ManipulatorException;
import manipulation.muster.runner.cogx.CogXRunner;
import manipulation.muster.strategies.CommandExecution;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.LinearGraspApproachCommand;
import manipulation.slice.ManipulationCommand;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateGraspCommand;
import manipulation.slice.StopCommand;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to stop the arm movement
 * 
 * @author ttoenige
 * 
 */
public class StopCommandPart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	private boolean manipulationFailed = false;

	public StopCommandPart(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.STOP_COMMAND_PART);
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
			getManipulator().getArmConnector().stopArm();
		} catch (ManipulatorException e1) {
			logger.error(e1);
			manipulationFailed = true;
		}

		if (!manipulationFailed) {
			StopCommand currentCom = ((StopCommand) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.FINISHED;
			currentCom.comp = ManipulationCompletion.SUCCEEDED;

			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);

		} else {
			StopCommand currentCom = ((StopCommand) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.COMMANDFAILED;
			currentCom.comp = ManipulationCompletion.FAILED;

			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);

		}

		setNextPartName(PartName.WAIT_PART);

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

				((CommandExecution) getGlobalStrategy())
						.setCurrentCommand((ManipulationCommand) arg);

			}

			if (arg instanceof FarArmMovementCommand) {
				logger.info("far arm movement command");

				setNextPartName(PartName.FAR_ARM_MOVEMENT_COMMAND_PART);
				changeToNextPart();
			} else if (arg instanceof PutDownCommand) {
				logger.info("put down command");

				setNextPartName(PartName.PUT_DOWN_COMMAND_PART);
				changeToNextPart();
			} else if (arg instanceof LinearGraspApproachCommand) {
				logger.info("linear grasp approach command");

				setNextPartName(PartName.LINEAR_GRASP_APPROACH_COMMAND_PART);
				changeToNextPart();
			} else if (arg instanceof SimulateGraspCommand) {
				logger.info("simulate grasp command");

				setNextPartName(PartName.SIMULATE_GRASP_COMMAND_PART);
				changeToNextPart();
			} else if (arg instanceof StopCommand) {
				logger.info("stop command");

				setNextPartName(PartName.STOP_COMMAND_PART);
				changeToNextPart();
			} else if (arg instanceof MoveArmToHomePositionCommand) {
				logger.info("move arm to home position command");

				setNextPartName(PartName.MOVE_ARM_TO_HOME_POSITION_COMMAND_PART);
				changeToNextPart();
			} else if (arg instanceof OpenGripperCommand) {
				logger.info("open gripper command");
				
				setNextPartName(PartName.OPEN_GRIPPER_PART);
				changeToNextPart();
			} else if (arg instanceof CloseGripperCommand) {
				logger.info("close gripper command");
				
				setNextPartName(PartName.CLOSE_GRIPPER_PART);
				changeToNextPart();
			}
		}
	}
}

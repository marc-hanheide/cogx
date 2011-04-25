package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.core.share.Manipulator;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.FarArmMovementCommand;
import manipulation.slice.GetCurrentArmPose;
import manipulation.slice.GraspingStatus;
import manipulation.slice.LinearGraspApproachCommand;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.ManipulationExternalCommand;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.MoveArmToPose;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.PutDownCommand;
import manipulation.slice.SimulateGraspCommand;
import manipulation.slice.StopCommand;
import manipulation.strategies.CommandExecution;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;
import manipulation.strategies.parts.StrategyPart.PartName;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to close the gripper
 * 
 * @author Torben Toeniges
 * 
 */
public class CloseGripperPart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	public CloseGripperPart(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.CLOSE_GRIPPER_PART);
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		getManipulator().getWatcher().addObserver(this);

		getManipulator().getArmConnector().closeGripper(10);

		CloseGripperCommand currentCom = ((CloseGripperCommand) ((CommandExecution) getGlobalStrategy())
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
				.updateWorkingMemoryCommand(getManipulator().getWatcher()
						.getCurrentCommandAddress(), currentCom);

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
			} else if (arg instanceof MoveArmToPose) {
				logger.info("move arm to pose command");
				setNextPartName(PartName.MOVE_ARM_TO_POSE_PART);
				changeToNextPart();
			} else if (arg instanceof GetCurrentArmPose) {
				logger.info("get cureent arm pose command");
				setNextPartName(PartName.GET_CURRENT_ARM_POSE_PART);
				changeToNextPart();
			}
		}
	}
}

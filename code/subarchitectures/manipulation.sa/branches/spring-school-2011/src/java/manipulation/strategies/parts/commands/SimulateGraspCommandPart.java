package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.commandWatcher.CommandWatcher;
import manipulation.core.cogx.converter.CogXConverter;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Pose;
import manipulation.core.share.types.Vector3D;
import manipulation.math.MathOperation;
import manipulation.runner.cogx.CogXRunner;
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
import manipulation.strategies.CommandExecution;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

import VisionData.VisualObject;
import cast.SubarchitectureComponentException;
import cast.cdl.WorkingMemoryAddress;

/**
 * defines a behaviour to reach a position in front of an object with the arm
 * 
 * @author Torben Toeniges
 * 
 */
public class SimulateGraspCommandPart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	private boolean manipulationFailed = false;

	private Pose simulatedPose = null;

	public SimulateGraspCommandPart(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.SIMULATE_GRASP_COMMAND_PART);
	}

	private void simulateArm() {

		WorkingMemoryAddress wma = ((SimulateGraspCommand) ((CommandExecution) getGlobalStrategy())
				.getCurrentCommand()).targetObjectAddr;

		try {
			VisualObject targetVisOb = (((CogXRunner) getManipulator()
					.getRunner()).getMemoryEntry(wma, VisualObject.class));

			Vector3D currentGoalPosition = new Vector3D(targetVisOb.pose.pos.x,
					targetVisOb.pose.pos.y, targetVisOb.pose.pos.z);

			Vector3D currentArmPos = null;
			try {
				currentArmPos = getManipulator().getArmConnector()
						.getCurrentPosition();
			} catch (ManipulatorException e) {
				logger.error(e);
				manipulationFailed = true;
				return;
			}

			Matrix rotation1 = MathOperation.getRotationAroundX(MathOperation
					.getRadiant(0));
			Matrix rotation2 = MathOperation.getRotationAroundY(MathOperation
					.getRadiant(0));
			Matrix rotation3 = MathOperation.getRotationAroundZ(MathOperation
					.getRadiant(-90));
			Matrix greifRotation = MathOperation.getMatrixMatrixMultiplication(
					MathOperation.getMatrixMatrixMultiplication(rotation1,
							rotation2), rotation3);

			simulatedPose = getManipulator().getArmConnector()
					.simulateArmMovement(
							new Pose(greifRotation, currentGoalPosition));

		} catch (SubarchitectureComponentException e) {
			logger.error(e);
			manipulationFailed = true;
			return;
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

		simulateArm();

		setNextPartName(PartName.WAIT_PART);

		if (!manipulationFailed) {
			SimulateGraspCommand currentCom = ((SimulateGraspCommand) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.FINISHED;
			currentCom.comp = ManipulationCompletion.SUCCEEDED;

			currentCom.simulatedReachablePose = CogXConverter
					.convertToPose3(simulatedPose.getTranslation(),
							simulatedPose.getRotation());

			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);
		} else {
			SimulateGraspCommand currentCom = ((SimulateGraspCommand) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.COMMANDFAILED;
			currentCom.comp = ManipulationCompletion.FAILED;
			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);
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

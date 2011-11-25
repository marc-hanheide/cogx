package manipulation.strategies.parts.commands;

import java.util.Observable;
import java.util.Observer;

import manipulation.core.cogx.converter.CogXConverter;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector2D;
import manipulation.core.share.types.Vector3D;
import manipulation.math.MathOperation;
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
import mathlib.Functions;

import org.apache.log4j.Logger;

import cogx.Math.Pose3;
import cogx.Math.Vector3;

public class GetCurrentArmPosePart extends StrategyPart implements Observer {

	private Logger logger = Logger.getLogger(this.getClass());

	private boolean manipulationFailed = false;

	public GetCurrentArmPosePart(Manipulator manipulator,
			Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.GET_CURRENT_ARM_POSE_PART);
	}

	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());
		getManipulator().getWatcher().addObserver(this);

		Vector3D pos = null;
		Matrix rot = null;

		Pose3 pInRobot = null;

		try {
			pos = getManipulator().getArmConnector().getCurrentPosition();
			rot = getManipulator().getArmConnector().getCurrentRotation();

			Pose3 pInWorld = CogXConverter.convertToPose3(pos, rot);

			Vector2D position = getManipulator().getBaseConnector()
					.getCurrentPosition().getPoint();

			Pose3 tWorldToRobot = Functions.pose3FromEuler(
					new Vector3(position.getX(), position.getY(), 0), 0.0, 0.0,
					getManipulator().getBaseConnector().getCurrentPosition()
							.getAngle());

			pInRobot = Functions.transformInverse(tWorldToRobot, pInWorld);

			setNextPartName(PartName.WAIT_PART);

		} catch (ManipulatorException e) {
			logger.error(e);
			manipulationFailed = true;
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		if (!manipulationFailed) {
			GetCurrentArmPose currentCom = ((GetCurrentArmPose) ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand());

			currentCom.status = ManipulationCommandStatus.FINISHED;
			currentCom.comp = ManipulationCompletion.SUCCEEDED;

			currentCom.currentPose = pInRobot;

			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);
		} else {
			GetCurrentArmPose currentCom = ((GetCurrentArmPose) ((CommandExecution) getGlobalStrategy())
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

	@Override
	public void changeToNextPart() {
		getManipulator().getWatcher().deleteObserver(this);

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}

	@Override
	public void update(Observable observable, Object arg) {
		if (arg instanceof ManipulationExternalCommand) {
			ManipulationExternalCommand currentCom = ((CommandExecution) getGlobalStrategy())
					.getCurrentCommand();
			currentCom.status = ManipulationCommandStatus.COMMANDFAILED;
			currentCom.comp = ManipulationCompletion.FAILED;
			((CogXRunner) (getManipulator().getRunner()))
					.updateWorkingMemoryCommand(getManipulator().getWatcher()
							.getCurrentCommandAddress(), currentCom);

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
		} else if (arg instanceof FineArmMovementCommand) {
			logger.info("linear grasp approach command");
			setNextPartName(PartName.FINE_ARM_MOVEMENT_COMMAND_PART);
			changeToNextPart();
		} else if (arg instanceof SimulateFarArmMovementCommand) {
			logger.info("simulate grasp command");
			setNextPartName(PartName.SIMULATE_FAR_ARM_MOVEMENT_PART);
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
			logger.info("get current arm pose command");
			setNextPartName(PartName.GET_CURRENT_ARM_POSE_PART);
			changeToNextPart();
		} else if (arg instanceof SimulateMoveToPose) {
			logger.info("simulate move to pose command");
			setNextPartName(PartName.SIMULATE_MOVE_TO_POSE_PART);
			changeToNextPart();
		}
	}
}
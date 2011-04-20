package manipulation.muster.core.cogx.armConnector;

import golem.tinyice.GenConfigspaceState;

import java.util.HashMap;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.armConnector.ArmConnector;
import manipulation.muster.core.share.exceptions.ManipulatorException;
import manipulation.muster.core.share.types.ArmError;
import manipulation.muster.core.share.types.Matrix;
import manipulation.muster.core.share.types.SensorData.SensorPosition;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.runner.cogx.ExemplarySolutionRunner;
import manipulation.slice.CloseGripperCommand;
import manipulation.slice.MoveArmToHomePositionCommand;
import manipulation.slice.OpenGripperCommand;
import manipulation.slice.StopCommand;

import org.apache.log4j.Logger;

import cast.AlreadyExistsOnWMException;

public class ExemplarySolutionKatanaArmConnector implements ArmConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	/**
	 * constructor of the Birmingham / CogX arm controller
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public ExemplarySolutionKatanaArmConnector(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	/**
	 * gets the current configuration space of the arm
	 * 
	 * @return current configuration space
	 * @throws ManipulatorException
	 */
	public GenConfigspaceState getCurrentConfigState()
			throws ManipulatorException {
		logger.error("NOT USE getCurrentConfigState");
		return null;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void goHome() throws ManipulatorException {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		MoveArmToHomePositionCommand moveHomeCmd = new MoveArmToHomePositionCommand();
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, moveHomeCmd);
		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void reach(Vector3D position, Matrix rotation)
			throws ManipulatorException {

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector3D getCurrentPosition() throws ManipulatorException {
		logger.error("NOT USE getCurrentPosition");
		return null;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getCurrentRotation() throws ManipulatorException {
		logger.error("NOT USE getCurrentRotation");
		return null;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopArm() throws ManipulatorException {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		StopCommand stopCommand = new StopCommand();
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, stopCommand);
		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	@Override
	public boolean isReached() {
		logger.error("NOT USE isReached");
		return false;
	}

	@Override
	public boolean isHome() {
		logger.error("NOT USE isHome");
		return false;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void setReached(boolean reached) {
		logger.error("NOT USE setReached");
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void closeGripper(int force) {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		CloseGripperCommand closeGripperCmd = new CloseGripperCommand();
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, closeGripperCmd);
		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void openGripper() {
		String id = ((ExemplarySolutionRunner) manipulator.getRunner())
				.newDataID();
		OpenGripperCommand openGripperCmd = new OpenGripperCommand();
		try {
			((ExemplarySolutionRunner) manipulator.getRunner())
					.addToWorkingMemory(id, openGripperCmd);
		} catch (AlreadyExistsOnWMException e1) {
			logger.error(e1);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void freezeGripper() {

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public HashMap<SensorPosition, Integer> receiveGripperSensorData() {
		logger.error("NOT USE receiveGripperSensorData");
		return null;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ArmError getPosError(Vector3D targetPosition, Matrix targetRotation)
			throws ManipulatorException {
		return null;
	}

	@Override
	@Deprecated
	public boolean isClosed() {
		logger.error("NOT USE isClosed");
		return false;
	}

	@Override
	public boolean isGraspingObject() {
		logger.error("NOT USE isClosed");
		return false;
	}
}

package manipulation.core.cogx.armConnector;

import golem.tinyice.GenConfigspaceState;

import java.util.HashMap;

import manipulation.core.share.Manipulator;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.ArmError;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.SensorData.SensorPosition;
import manipulation.core.share.types.Vector3D;

import org.apache.log4j.Logger;

public class ExemplarySolutionKatanaArmConnector implements ArmConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor of the Birmingham / CogX arm controller
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public ExemplarySolutionKatanaArmConnector(Manipulator manipulator) {

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

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void openGripper() {

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
		return false;
	}
}

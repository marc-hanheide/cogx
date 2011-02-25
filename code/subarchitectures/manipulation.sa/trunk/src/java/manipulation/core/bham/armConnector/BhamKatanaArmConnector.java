package manipulation.core.bham.armConnector;

import golem.tinyice.ArmPrx;
import golem.tinyice.ExTinyArm;
import golem.tinyice.ExTinyKatanaArm;
import golem.tinyice.GenConfigspaceState;
import golem.tinyice.GenWorkspaceState;
import golem.tinyice.KatanaArmPrx;
import golem.tinyice.KatanaGripperEncoderData;
import golem.tinyice.KatanaSensorData;
import golem.tinyice.Mat34;
import golem.tinyice.Twist;
import golem.tinyice.Vec3;

import java.util.HashMap;

import manipulation.core.bham.converter.BhamConverter;
import manipulation.core.share.Manipulator;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.ArmError;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Quaternion;
import manipulation.core.share.types.SensorData;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.SensorData.SensorPosition;
import manipulation.math.MathOperation;

import org.apache.log4j.Logger;

public class BhamKatanaArmConnector implements ArmConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	private GenConfigspaceState homePosition;
	private Manipulator manipulator;
	private ArmPrx arm;
	private boolean reached = false;
	private boolean closed = false;
	private boolean home = false;

	/**
	 * constructor of the Birmingham / CogX arm controller
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public BhamKatanaArmConnector(Manipulator manipulator) {
		this.manipulator = manipulator;

		arm = manipulator.getSimulationConnector().getArm();

		try {
			homePosition = arm.recvGenConfigspaceState(manipulator
					.getSimulationConnector().getTime());
		} catch (ExTinyArm e) {
			logger.error(e);
		}

	}

	/**
	 * gets the current configuration space of the arm
	 * 
	 * @return current configuration space
	 * @throws ManipulatorException
	 */
	public GenConfigspaceState getCurrentConfigState()
			throws ManipulatorException {
		GenConfigspaceState currentConfState = null;
		try {
			currentConfState = arm.recvGenConfigspaceState(manipulator
					.getSimulationConnector().getTime()
					+ arm.getTimeDeltaAsync());
		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
		}

		return currentConfState;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void goHome() throws ManipulatorException {
		stopArm();
		manipulator.getSimulationConnector().clearScene();
		homePosition.t = manipulator.getSimulationConnector().getTime() + 5;

		try {
			GenConfigspaceState cBegin = arm
					.recvGenConfigspaceState(manipulator
							.getSimulationConnector().getTime());
			GenConfigspaceState cEnd = homePosition;

			GenConfigspaceState[] trajectory;
			trajectory = arm.findTrajectory(cBegin, cEnd);
			logger.debug("Moving to home position...");
			arm.send(trajectory, Double.MAX_VALUE);
			home = true;
		} catch (Exception e) {
			logger.error(e);
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void reach(Vector3D position, Matrix rotation)
			throws ManipulatorException {
		stopArm();

		home = false;

		GenWorkspaceState genPosition = new GenWorkspaceState();
		genPosition.pos = new Mat34();
		genPosition.pos.p = new Vec3(position.getX(), position.getY(), position
				.getZ());

		// genPosition.pos.R = BhamConverter.convMatrixToGolem(MathOperation
		// .multMatrixWithMatrix(rotation, initArmRotation));
		genPosition.pos.R = BhamConverter.convMatrixToGolem(rotation);

		genPosition.vel = new Twist(new Vec3(0, 0, 0), new Vec3(0, 0, 0));
		genPosition.t = manipulator.getSimulationConnector().getTime()
				+ arm.getTimeDeltaAsync();

		try {
			GenConfigspaceState cbegin = arm
					.recvGenConfigspaceState(manipulator
							.getSimulationConnector().getTime()
							+ arm.getTimeDeltaAsync());

			GenConfigspaceState cend = arm.findTarget(cbegin, genPosition);

			GenConfigspaceState[] trajectory = arm.findTrajectory(cbegin, cend);

			reached = false;
			arm.send(trajectory, 1);

			Thread t = new Thread(new GoalReachedRunnable(manipulator));
			t.start();

		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Vector3D getCurrentPosition() throws ManipulatorException {
		Vector3D returnValue = null;
		GenWorkspaceState armPos;
		try {
			armPos = arm.recvGenWorkspaceState(manipulator
					.getSimulationConnector().getTime()
					+ arm.getTimeDeltaAsync());

			returnValue = BhamConverter.convGolemToVec(armPos.pos.p);
		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
		}

		return returnValue;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public Matrix getCurrentRotation() throws ManipulatorException {
		GenWorkspaceState armPos;
		Matrix returnValue = null;
		try {
			armPos = arm.recvGenWorkspaceState(manipulator
					.getSimulationConnector().getTime()
					+ arm.getTimeDeltaAsync());
			returnValue = BhamConverter.convGolemToMatrix(armPos.pos.R);
		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
		}

		return returnValue;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopArm() throws ManipulatorException {
		try {
			if (!reached) {
				logger.debug("STOP");
				arm.stop();
				Thread.sleep(3000);
			}
		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
		} catch (InterruptedException e) {
			throw new ManipulatorException(e.getMessage());
		}
	}

	@Override
	public boolean isReached() {
		return reached;
	}

	@Override
	public boolean isHome() {
		return home;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void setReached(boolean reached) {
		this.reached = reached;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void closeGripper(int force) {
		HashMap<SensorPosition, Integer> sensorData = null;

		sensorData = receiveGripperSensorData();

		KatanaSensorData[] threshold = new KatanaSensorData[4];

		threshold[0] = new KatanaSensorData(SensorData
				.convertSensorPositionToIndex(SensorPosition.FORCE_LEFT_FAR),
				(sensorData.get(SensorPosition.FORCE_LEFT_FAR) + force));

		threshold[1] = new KatanaSensorData(SensorData
				.convertSensorPositionToIndex(SensorPosition.FORCE_LEFT_NEAR),
				(sensorData.get(SensorPosition.FORCE_LEFT_NEAR) + force));
		threshold[2] = new KatanaSensorData(SensorData
				.convertSensorPositionToIndex(SensorPosition.FORCE_RIGHT_FAR),
				(sensorData.get(SensorPosition.FORCE_RIGHT_FAR) + force));
		threshold[3] = new KatanaSensorData(SensorData
				.convertSensorPositionToIndex(SensorPosition.FORCE_RIGHT_NEAR),
				(sensorData.get(SensorPosition.FORCE_RIGHT_NEAR) + force));

		try {
			((KatanaArmPrx) arm).gripperClose(threshold, 5);
			closed = true;
		} catch (ExTinyKatanaArm e) {
			logger.debug(e);
			closed = true;

			try {
				((KatanaArmPrx) arm).gripperClose(threshold, 5);
			} catch (ExTinyKatanaArm e1) {
				logger.error(e1);
			}
			// TODO really ignore here?
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void openGripper() {

		if (!manipulator.getConfiguration().isSimulation()) {
			try {
				((KatanaArmPrx) arm).gripperOpen(5);
				closed = false;
			} catch (ExTinyKatanaArm e) {
				logger.debug(e);
				closed = false;

				try {
					((KatanaArmPrx) arm).gripperOpen(5);
				} catch (ExTinyKatanaArm e1) {
					logger.error(e1);
				}
			}
		} else {
			logger.info("Open Gripper not implemented yet");
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void freezeGripper() {
		if (!manipulator.getConfiguration().isSimulation()) {
			try {
				((KatanaArmPrx) arm).gripperFreeze(5);
			} catch (ExTinyKatanaArm e) {
				logger.debug(e);
				// TODO really igonore the error?
			}
		} else {
			logger.info("Freeze Gripper not implemented yet");
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public HashMap<SensorPosition, Integer> receiveGripperSensorData() {
		if (!manipulator.getConfiguration().isSimulation()) {
			HashMap<SensorPosition, Integer> sensorData = new HashMap<SensorPosition, Integer>();

			// TODO was fuer einen Timeout?
			KatanaSensorData[] golemSensorData;
			try {
				golemSensorData = ((KatanaArmPrx) arm).gripperRecvSensorData(5);

				for (int i = 0; i < golemSensorData.length; i++) {
					sensorData.put(SensorData.convertIndexToSensorPosition(i),
							golemSensorData[i].value);
				}

			} catch (ExTinyKatanaArm e) {
				logger.debug(e);
				// TOOD wirklich ignorieren?
			}

			return sensorData;
		} else {
			logger.info("Receive data not implemented yet");
			return null;
		}
	}

	// TODO wirklich so machen, dass man alles nochmal berechnet?
	/**
	 * {@inheritDoc}
	 */
	@Override
	public ArmError getPosError(Vector3D targetPosition, Matrix targetRotation)
			throws ManipulatorException {
		// stopArm();

		GenWorkspaceState genPosition = new GenWorkspaceState();
		genPosition.pos = new Mat34();
		genPosition.pos.p = new Vec3(targetPosition.getX(), targetPosition
				.getY(), targetPosition.getZ());

		// genPosition.pos.R = BhamConverter.convMatrixToGolem(MathOperation
		// .multMatrixWithMatrix(rotation, initArmRotation));
		genPosition.pos.R = BhamConverter.convMatrixToGolem(targetRotation);

		genPosition.vel = new Twist(new Vec3(0, 0, 0), new Vec3(0, 0, 0));
		genPosition.t = manipulator.getSimulationConnector().getTime()
				+ arm.getTimeDeltaAsync();

		Vector3D positionError = null;
		double angularError;

		try {
			GenConfigspaceState cbegin = arm
					.recvGenConfigspaceState(manipulator
							.getSimulationConnector().getTime()
							+ arm.getTimeDeltaAsync());
			GenConfigspaceState cend = arm.findTarget(cbegin, genPosition);
			Mat34[] forwardTransArray = arm.getForwardTransform(cend.pos);
			Mat34 forwardTrans = forwardTransArray[forwardTransArray.length - 1];

			Matrix forwardTransRot = BhamConverter
					.convGolemToMatrix(forwardTrans.R);
			// logger.error(forwardTransRot);
			Vector3D forwardTransVec = BhamConverter
					.convGolemToVec(forwardTrans.p);
			// logger.error(forwardTransVec);

			Mat34 refMatrix = arm.getReferencePose();
			Matrix refRotation = BhamConverter.convGolemToMatrix(refMatrix.R);
			Vector3D refPosition = BhamConverter.convGolemToVec(refMatrix.p);

			Matrix newRotation = MathOperation.getMatrixMatrixMultiplication(
					forwardTransRot, refRotation);

			Vector3D newTranslation = MathOperation.getVectorAddition(
					MathOperation.getMatrixVectorMultiplication(
							forwardTransRot, refPosition), forwardTransVec);

			positionError = MathOperation.getDistanceSeparatedDimensions(
					targetPosition, newTranslation);

			Quaternion newRorationQuad = MathOperation
					.getQuaternion(newRotation);
			Quaternion inputRotation = MathOperation
					.getQuaternion(targetRotation);

			// angularError =
			// inputRotation.plus(newRorationQuad.negate()).norm();

			angularError = MathOperation.getAngularDist(newRorationQuad,
					inputRotation);

			logger.error("Position Error = " + positionError);
			logger.error("Angular Error = " + positionError);

		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
		}

		return new ArmError(positionError, angularError);
	}

	@Override
	@Deprecated
	public boolean isClosed() {
		return closed;

	}

	@Override
	public boolean isGraspingObject() {

		KatanaGripperEncoderData data = null;

		try {
			data = ((KatanaArmPrx) arm).gripperRecvEncoderData(5);
		} catch (ExTinyKatanaArm e) {
			logger.debug(e);
		}

		logger.error("Closed: " + data.closed);
		logger.error("Open: " + data.open);
		logger.error("Current: " + data.current);

		if (data.current > 8700 && closed) {
			logger.error("Grasping object!");
			return true;
		} else {
			logger.error("Not grasping object!");
			return false;
		}

	}
}

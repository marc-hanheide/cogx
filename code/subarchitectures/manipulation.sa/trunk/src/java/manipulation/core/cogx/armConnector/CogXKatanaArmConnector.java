package manipulation.core.cogx.armConnector;

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

import manipulation.core.cogx.converter.CogXConverter;
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

public class CogXKatanaArmConnector implements ArmConnector {

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
	public CogXKatanaArmConnector(Manipulator manipulator) {
		this.manipulator = manipulator;

		arm = manipulator.getVirtualSceneConnector().getArm();

		try {
			homePosition = arm.recvGenConfigspaceState(manipulator
					.getVirtualSceneConnector().getTime());
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
					.getVirtualSceneConnector().getTime()
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

		try {
			Thread.sleep(3000);
		} catch (InterruptedException e1) {
			logger.error(e1);
		}

		// manipulator.getVirtualSceneConnector().clearScene();
		homePosition.t = manipulator.getVirtualSceneConnector().getTime() + 5;

		try {
			GenConfigspaceState cBegin = arm
					.recvGenConfigspaceState(manipulator
							.getVirtualSceneConnector().getTime());
			GenConfigspaceState cEnd = homePosition;

			GenConfigspaceState[] trajectory;
			trajectory = arm.findTrajectory(cBegin, cEnd);
			logger.debug("Moving to home position...");
			reached = false;

			Thread t = new Thread(new GoalReachedRunnable(manipulator));
			t.start();

			arm.send(trajectory, 1);
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

		try {
			Thread.sleep(3000);
		} catch (InterruptedException e1) {
			logger.error(e1);
		}

		home = false;

		GenWorkspaceState genPosition = new GenWorkspaceState();
		genPosition.pos = new Mat34();
		genPosition.pos.p = new Vec3(position.getX(), position.getY(),
				position.getZ());

		// genPosition.pos.R = CogXConverter.convMatrixToGolem(MathOperation
		// .multMatrixWithMatrix(rotation, initArmRotation));
		genPosition.pos.R = CogXConverter.convMatrixToGolem(rotation);

		genPosition.vel = new Twist(new Vec3(0, 0, 0), new Vec3(0, 0, 0));
		genPosition.t = manipulator.getVirtualSceneConnector().getTime()
				+ arm.getTimeDeltaAsync();

		try {
			GenConfigspaceState cbegin = arm
					.recvGenConfigspaceState(manipulator
							.getVirtualSceneConnector().getTime()
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
					.getVirtualSceneConnector().getTime()
					+ arm.getTimeDeltaAsync());

			returnValue = CogXConverter.convGolemToVec(armPos.pos.p);
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
					.getVirtualSceneConnector().getTime()
					+ arm.getTimeDeltaAsync());
			returnValue = CogXConverter.convGolemToMatrix(armPos.pos.R);
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
			}
		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
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
		manipulator.getWatcher().posReached();
		this.reached = reached;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void closeGripper(int force) {
		if (!(manipulator.getConfiguration().getArmName() == ArmName.SIMULATION)) {
			KatanaSensorData[] threshold = new KatanaSensorData[4];

			HashMap<SensorPosition, Integer> sensorData = null;
			sensorData = receiveGripperSensorData();

			if ((manipulator.getConfiguration().getArmName() == ArmName.KATANA300)) {
				threshold[0] = new KatanaSensorData(
						SensorData
								.convertSensorPositionToIndex(SensorPosition.FORCE_LEFT_FAR),
						(sensorData.get(SensorPosition.FORCE_LEFT_FAR) + force));

				threshold[1] = new KatanaSensorData(
						SensorData
								.convertSensorPositionToIndex(SensorPosition.FORCE_LEFT_NEAR),
						(sensorData.get(SensorPosition.FORCE_LEFT_NEAR) + force));
				threshold[2] = new KatanaSensorData(
						SensorData
								.convertSensorPositionToIndex(SensorPosition.FORCE_RIGHT_FAR),
						(sensorData.get(SensorPosition.FORCE_RIGHT_FAR) + force));
				threshold[3] = new KatanaSensorData(
						SensorData
								.convertSensorPositionToIndex(SensorPosition.FORCE_RIGHT_NEAR),
						(sensorData.get(SensorPosition.FORCE_RIGHT_NEAR) + force));
			}

			try {
				((KatanaArmPrx) arm).gripperClose(threshold, 5);
				closed = true;
			} catch (ExTinyKatanaArm e) {
				logger.debug(e);
				closed = true;
			}
		} else {
			logger.info("Close Gripper not implemented yet");
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void openGripper() {

		if (!(manipulator.getConfiguration().getArmName() == ArmName.SIMULATION)) {
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
		if (!(manipulator.getConfiguration().getArmName() == ArmName.SIMULATION)) {
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
		if ((manipulator.getConfiguration().getArmName() == ArmName.KATANA300)) {
			HashMap<SensorPosition, Integer> sensorData = new HashMap<SensorPosition, Integer>();

			// TODO what a timeout
			KatanaSensorData[] golemSensorData;
			try {
				golemSensorData = ((KatanaArmPrx) arm).gripperRecvSensorData(5);

				for (int i = 0; i < golemSensorData.length; i++) {
					sensorData.put(SensorData.convertIndexToSensorPosition(i),
							golemSensorData[i].value);
				}

			} catch (ExTinyKatanaArm e) {
				logger.debug(e);
				// TOOD really ignore?
			}

			return sensorData;
		} else {
			logger.info("Receive data not implemented yet");
			return null;
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ArmError getPosError(Vector3D targetPosition, Matrix targetRotation)
			throws ManipulatorException {
		// stopArm();

		GenWorkspaceState genPosition = new GenWorkspaceState();
		genPosition.pos = new Mat34();
		genPosition.pos.p = new Vec3(targetPosition.getX(),
				targetPosition.getY(), targetPosition.getZ());

		// genPosition.pos.R = CogXConverter.convMatrixToGolem(MathOperation
		// .multMatrixWithMatrix(rotation, initArmRotation));
		genPosition.pos.R = CogXConverter.convMatrixToGolem(targetRotation);

		genPosition.vel = new Twist(new Vec3(0, 0, 0), new Vec3(0, 0, 0));
		genPosition.t = manipulator.getVirtualSceneConnector().getTime()
				+ arm.getTimeDeltaAsync();

		Vector3D positionError = null;
		double angularError;

		try {
			GenConfigspaceState cbegin = arm
					.recvGenConfigspaceState(manipulator
							.getVirtualSceneConnector().getTime()
							+ arm.getTimeDeltaAsync());
			GenConfigspaceState cend = arm.findTarget(cbegin, genPosition);
			Mat34[] forwardTransArray = arm.getForwardTransform(cend.pos);
			Mat34 forwardTrans = forwardTransArray[forwardTransArray.length - 1];

			Matrix forwardTransRot = CogXConverter
					.convGolemToMatrix(forwardTrans.R);
			// logger.error(forwardTransRot);
			Vector3D forwardTransVec = CogXConverter
					.convGolemToVec(forwardTrans.p);
			// logger.error(forwardTransVec);

			Mat34 refMatrix = arm.getReferencePose();
			Matrix refRotation = CogXConverter.convGolemToMatrix(refMatrix.R);
			Vector3D refPosition = CogXConverter.convGolemToVec(refMatrix.p);

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

			// logger.error("Position Error = " + positionError);
			// logger.error("Angular Error = " + positionError);

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

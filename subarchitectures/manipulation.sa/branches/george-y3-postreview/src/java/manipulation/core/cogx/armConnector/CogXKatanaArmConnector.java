package manipulation.core.cogx.armConnector;

import golem.tinyice.ArmPrx;
import golem.tinyice.ConfigspaceCoord;
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
import manipulation.core.cogx.virtualSceneConnector.CogXVirtualSceneConnector;
import manipulation.core.share.Manipulator;
import manipulation.core.share.armConnector.ArmConnector;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.ArmError;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Pose;
import manipulation.core.share.types.SensorData;
import manipulation.core.share.types.SensorData.SensorPosition;
import manipulation.core.share.types.Vector3D;
import manipulation.math.MathOperation;
import manipulation.runner.cogx.CogXRunner;
import manipulation.slice.GenConfigspaceCoord;
import manipulation.slice.GraspingStatus;
import manipulation.slice.ManipulationCommandStatus;
import manipulation.slice.ManipulationCompletion;
import manipulation.slice.PlayerBridgeCloseGripperCommand;
import manipulation.slice.PlayerBridgeOpenGripperCommand;
import manipulation.slice.PlayerBridgeSendTrajectoryCommand;

import org.apache.log4j.Logger;

import cast.AlreadyExistsOnWMException;

/**
 * connector to the COGX arm communication system
 * 
 * @author Torben Toeniges
 * 
 */
public class CogXKatanaArmConnector implements ArmConnector {

	private Logger logger = Logger.getLogger(this.getClass());

	private GenConfigspaceState homePosition;
	private Manipulator manipulator;
	private ArmPrx arm;
	private boolean reached = false;
	private boolean closed = false;
	private boolean home = false;

	/**
	 * constructor of the CogX arm controller
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 */
	public CogXKatanaArmConnector(Manipulator manipulator) {
		this.manipulator = manipulator;

		arm = ((CogXVirtualSceneConnector) manipulator
				.getVirtualSceneConnector()).getArm();

		try {
			homePosition = arm.recvGenConfigspaceState(manipulator
					.getVirtualSceneConnector().getTime());
		} catch (ExTinyArm e) {
			logger.error(e);
		}
	}

	public void initSimMove() {
		GenConfigspaceState cBegin = new GenConfigspaceState();
		cBegin.pos = new ConfigspaceCoord();
		cBegin.vel = new ConfigspaceCoord();
		cBegin.pos.c = new double[homePosition.pos.c.length];
		cBegin.vel.c = new double[homePosition.pos.c.length];
		cBegin.t = arm.getTimeDelta();

		for (int i = 0; i < homePosition.pos.c.length; i++) {
			cBegin.pos.c[i] = 0;
			cBegin.vel.c[i] = 0;
		}

		GenConfigspaceState[] trajectory;
		try {
			logger.debug("Moving to home position...");

			homePosition.t = cBegin.t + arm.getTimeDeltaAsync() + 3;

			GenConfigspaceState cEnd = homePosition;

			trajectory = arm.findTrajectory(cBegin, cEnd);

			PlayerBridgeSendTrajectoryCommand cmd = new PlayerBridgeSendTrajectoryCommand();

			manipulation.slice.GenConfigspaceState[] returnVal = new manipulation.slice.GenConfigspaceState[trajectory.length];

			for (int i = 0; i < trajectory.length; i++) {
				GenConfigspaceCoord coord = new GenConfigspaceCoord();

				coord.pos = new double[trajectory[i].pos.c.length];
				coord.vel = new double[trajectory[i].vel.c.length];

				coord.pos = trajectory[i].pos.c;
				coord.vel = trajectory[i].vel.c;

				returnVal[i] = new manipulation.slice.GenConfigspaceState(
						coord, trajectory[i].t);
			}

			cmd.trajectory = returnVal;
			cmd.status = ManipulationCommandStatus.NEW;
			cmd.comp = ManipulationCompletion.COMPINIT;

			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						cmd);
			} catch (AlreadyExistsOnWMException e) {
				logger.error(e);
			}

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
					.getVirtualSceneConnector().getTime());
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
		try {
			GenConfigspaceState cBegin = arm
					.recvGenConfigspaceState(manipulator
							.getVirtualSceneConnector().getTime());
			homePosition.t = cBegin.t + arm.getTimeDeltaAsync() + 3;

			GenConfigspaceState cEnd = homePosition;

			GenConfigspaceState[] trajectory;
			trajectory = arm.findTrajectory(cBegin, cEnd);

			logger.debug("Moving to home position...");
			reached = false;

			if (manipulator.getConfiguration().getArmName() == ArmName.SIMULATION) {
				PlayerBridgeSendTrajectoryCommand cmd = new PlayerBridgeSendTrajectoryCommand();

				manipulation.slice.GenConfigspaceState[] returnVal = new manipulation.slice.GenConfigspaceState[trajectory.length];

				for (int i = 0; i < trajectory.length; i++) {
					GenConfigspaceCoord coord = new GenConfigspaceCoord();

					coord.pos = new double[trajectory[i].pos.c.length];
					coord.vel = new double[trajectory[i].vel.c.length];

					coord.pos = trajectory[i].pos.c;
					coord.vel = trajectory[i].vel.c;

					returnVal[i] = new manipulation.slice.GenConfigspaceState(
							coord, trajectory[i].t);
				}

				cmd.trajectory = returnVal;
				cmd.status = ManipulationCommandStatus.NEW;
				cmd.comp = ManipulationCompletion.COMPINIT;

				String id = ((CogXRunner) manipulator.getRunner()).newDataID();

				try {
					((CogXRunner) manipulator.getRunner()).addToWorkingMemory(
							id, cmd);
				} catch (AlreadyExistsOnWMException e) {
					logger.error(e);
				}
			}

			Thread t = new Thread(new GoalReachedRunnable(manipulator));
			t.start();

			arm.send(trajectory, 1);
			home = true;
		} catch (Exception e) {
			logger.error(e);
		}

	}

	@Override
	public void reach(Pose pose) throws ManipulatorException {
		reach(pose.getTranslation(), pose.getRotation());
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void reach(Vector3D position, Matrix rotation)
			throws ManipulatorException {
		home = false;

		GenWorkspaceState genPosition = new GenWorkspaceState();
		genPosition.pos = new Mat34();
		genPosition.pos.p = new Vec3(position.getX(), position.getY(),
				position.getZ());

		// genPosition.pos.R = CogXConverter.convMatrixToGolem(MathOperation
		// .multMatrixWithMatrix(rotation, initArmRotation));
		genPosition.pos.R = CogXConverter.convMatrixToGolem(rotation);

		genPosition.vel = new Twist(new Vec3(0, 0, 0), new Vec3(0, 0, 0));

		try {
			GenConfigspaceState cbegin = arm
					.recvGenConfigspaceState(manipulator
							.getVirtualSceneConnector().getTime());

			genPosition.t = cbegin.t + arm.getTimeDeltaAsync() + 3;

			GenConfigspaceState cend = arm.findTarget(cbegin, genPosition);

			GenConfigspaceState[] trajectory = arm.findTrajectory(cbegin, cend);

			reached = false;

			if (manipulator.getConfiguration().getArmName() == ArmName.SIMULATION) {
				PlayerBridgeSendTrajectoryCommand cmd = new PlayerBridgeSendTrajectoryCommand();

				manipulation.slice.GenConfigspaceState[] returnVal = new manipulation.slice.GenConfigspaceState[trajectory.length];

				for (int i = 0; i < trajectory.length; i++) {
					GenConfigspaceCoord coord = new GenConfigspaceCoord();

					coord.pos = new double[trajectory[i].pos.c.length];
					coord.vel = new double[trajectory[i].vel.c.length];

					coord.pos = trajectory[i].pos.c;
					coord.vel = trajectory[i].vel.c;

					returnVal[i] = new manipulation.slice.GenConfigspaceState(
							coord, trajectory[i].t);
				}

				cmd.trajectory = returnVal;
				cmd.status = ManipulationCommandStatus.NEW;
				cmd.comp = ManipulationCompletion.COMPINIT;

				String id = ((CogXRunner) manipulator.getRunner()).newDataID();

				try {
					((CogXRunner) manipulator.getRunner()).addToWorkingMemory(
							id, cmd);
				} catch (AlreadyExistsOnWMException e) {
					logger.error(e);
				}
			}

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
					+ arm.getTimeDeltaAsync() + 3);

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
					+ arm.getTimeDeltaAsync() + 3);
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

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean isReached() {
		return reached;
	}

	/**
	 * {@inheritDoc}
	 */
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
		if ((manipulator.getConfiguration().getArmName() == ArmName.SIMULATION)) {
			PlayerBridgeCloseGripperCommand cmd = new PlayerBridgeCloseGripperCommand();
			cmd.status = ManipulationCommandStatus.NEW;
			cmd.comp = ManipulationCompletion.COMPINIT;

			cmd.graspStatus = GraspingStatus.GRASPINGSTATUSINIT;
			String id = ((CogXRunner) manipulator.getRunner()).newDataID();
			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						cmd);
			} catch (AlreadyExistsOnWMException e) {
				logger.error(e);
			}
		} else {
			KatanaSensorData[] threshold = new KatanaSensorData[4];

			if ((manipulator.getConfiguration().getArmName() == ArmName.KATANA450)) {
				for (int i = 0; i < threshold.length; i++) {
					threshold[i] = new KatanaSensorData(0, 0);
				}
			}

			if ((manipulator.getConfiguration().getArmName() == ArmName.KATANA300)) {
				HashMap<SensorPosition, Integer> sensorData = null;
				sensorData = receiveGripperSensorData();

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

				((KatanaArmPrx) arm).gripperClose(threshold, 8);
				closed = true;
			} catch (ExTinyKatanaArm e) {
				logger.debug(e);
				closed = true;
			}
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void openGripper() {
		if ((manipulator.getConfiguration().getArmName() == ArmName.SIMULATION)) {
			PlayerBridgeOpenGripperCommand cmd = new PlayerBridgeOpenGripperCommand();
			cmd.status = ManipulationCommandStatus.NEW;
			cmd.comp = ManipulationCompletion.COMPINIT;
			String id = ((CogXRunner) manipulator.getRunner()).newDataID();

			try {
				((CogXRunner) manipulator.getRunner()).addToWorkingMemory(id,
						cmd);
			} catch (AlreadyExistsOnWMException e) {
				logger.error(e);
			}
		} else {
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
		stopArm();

		GenWorkspaceState genPosition = new GenWorkspaceState();
		genPosition.pos = new Mat34();
		genPosition.pos.p = new Vec3(targetPosition.getX(),
				targetPosition.getY(), targetPosition.getZ());

		// genPosition.pos.R = CogXConverter.convMatrixToGolem(MathOperation
		// .multMatrixWithMatrix(rotation, initArmRotation));
		genPosition.pos.R = CogXConverter.convMatrixToGolem(targetRotation);

		genPosition.vel = new Twist(new Vec3(0, 0, 0), new Vec3(0, 0, 0));

		Vector3D positionError = null;
		double angularError;

		try {
			GenConfigspaceState cbegin = arm
					.recvGenConfigspaceState(manipulator
							.getVirtualSceneConnector().getTime());

			genPosition.t = cbegin.t + arm.getTimeDeltaAsync() + 3;

			GenConfigspaceState cend = arm.findTarget(cbegin, genPosition);
			Mat34[] forwardTransArray = arm.getForwardTransform(cend.pos);
			Mat34 forwardTrans = forwardTransArray[forwardTransArray.length - 1];

			Matrix forwardTransRot = CogXConverter
					.convGolemToMatrix(forwardTrans.R);
			Vector3D forwardTransVec = CogXConverter
					.convGolemToVec(forwardTrans.p);

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

			// TODO need to be implemented
			angularError = Double.MAX_VALUE;

		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
		}

		return new ArmError(positionError, angularError);
	}

	/**
	 * {@inheritDoc}
	 */
	public KatanaGripperEncoderData getEncoderData()
			throws ManipulatorException {
		try {
			return ((KatanaArmPrx) arm).gripperRecvEncoderData(5);
		} catch (ExTinyKatanaArm e) {
			throw new ManipulatorException("Cannot get Encoder Data: " + e.what);
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public boolean isGraspingObject() {

		if (!(manipulator.getConfiguration().getArmName() == ArmName.SIMULATION)) {
			KatanaGripperEncoderData data = null;

			try {
				data = getEncoderData();
			} catch (ManipulatorException e) {
				logger.debug(e);
			}

			logger.debug("Closed: " + data.closed);
			logger.debug("Open: " + data.open);
			logger.debug("Current: " + data.current);

			if (data.current > 8700 && closed) {
				logger.debug("Grasping object!");
				return true;
			} else {
				logger.debug("Not grasping object!");
				return false;
			}
		} else {
			logger.warn("Receiving encoder data not implemented for simulation");
			return false;
		}
	}

	public Pose simulateArmMovement(Pose target) throws ManipulatorException {
		stopArm();

		GenWorkspaceState genPosition = new GenWorkspaceState();
		genPosition.pos = new Mat34();
		genPosition.pos.p = new Vec3(target.getTranslation().getX(), target
				.getTranslation().getY(), target.getTranslation().getZ());

		genPosition.pos.R = CogXConverter.convMatrixToGolem(target
				.getRotation());

		genPosition.vel = new Twist(new Vec3(0, 0, 0), new Vec3(0, 0, 0));

		Matrix newRotation;
		Vector3D newTranslation;

		try {
			GenConfigspaceState cbegin = arm
					.recvGenConfigspaceState(manipulator
							.getVirtualSceneConnector().getTime());

			genPosition.t = cbegin.t + arm.getTimeDeltaAsync() + 3;

			GenConfigspaceState cend = arm.findTarget(cbegin, genPosition);
			Mat34[] forwardTransArray = arm.getForwardTransform(cend.pos);
			Mat34 forwardTrans = forwardTransArray[forwardTransArray.length - 1];

			Matrix forwardTransRot = CogXConverter
					.convGolemToMatrix(forwardTrans.R);
			Vector3D forwardTransVec = CogXConverter
					.convGolemToVec(forwardTrans.p);

			Mat34 refMatrix = arm.getReferencePose();
			Matrix refRotation = CogXConverter.convGolemToMatrix(refMatrix.R);
			Vector3D refPosition = CogXConverter.convGolemToVec(refMatrix.p);

			newRotation = MathOperation.getMatrixMatrixMultiplication(
					forwardTransRot, refRotation);

			newTranslation = MathOperation.getVectorAddition(
					MathOperation.getMatrixVectorMultiplication(
							forwardTransRot, refPosition), forwardTransVec);

		} catch (ExTinyArm e) {
			throw new ManipulatorException(e.what);
		}

		return new Pose(newRotation, newTranslation);

	}
}

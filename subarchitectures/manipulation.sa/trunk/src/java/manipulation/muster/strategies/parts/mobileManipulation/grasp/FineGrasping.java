package manipulation.muster.strategies.parts.mobileManipulation.grasp;

import java.util.HashMap;

import manipulation.muster.core.share.Manipulator;
import manipulation.muster.core.share.exceptions.ExternalMemoryException;
import manipulation.muster.core.share.exceptions.InternalMemoryException;
import manipulation.muster.core.share.exceptions.ItemException;
import manipulation.muster.core.share.exceptions.ManipulatorException;
import manipulation.muster.core.share.types.ArmError;
import manipulation.muster.core.share.types.Matrix;
import manipulation.muster.core.share.types.SensorData.SensorPosition;
import manipulation.muster.core.share.types.Vector3D;
import manipulation.muster.itemMemory.Item.PropertyName;
import manipulation.muster.math.MathOperation;
import manipulation.muster.strategies.Strategy;
import manipulation.muster.strategies.parts.StrategyPart;

import org.apache.log4j.Logger;

/**
 * defines a behaviour to move the arm forward as long as nothing is between the
 * fingers of the gripper
 * 
 * @author ttoenige
 * 
 */
public class FineGrasping extends StrategyPart {

	private Logger logger = Logger.getLogger(this.getClass());

	/**
	 * constructor for the fine grasp part
	 * 
	 * @param manipulator
	 *            corresponding manipulator
	 * @param globalStrategy
	 *            corresponding global strategy
	 */
	public FineGrasping(Manipulator manipulator, Strategy globalStrategy) {
		setManipulator(manipulator);
		setGlobalStrategy(globalStrategy);
		setPartName(PartName.FINE_GRASPING);

	}

	private void fineApproach() {

		Vector3D currentGoalPosition = null;

		try {
			currentGoalPosition = (Vector3D) getManipulator().getItemMemory()
					.getFirstGraspItem()
					.getAttribute(PropertyName.WORLD_POSITION);
		} catch (ItemException e4) {
			logger.error(e4);
		} catch (InternalMemoryException e4) {
			logger.error(e4);
		}

		Vector3D currentArmPos = null;
		HashMap<SensorPosition, Integer> sensors = null;
		Matrix currentArmRot = null;
		Vector3D direction = null;
		try {
			currentArmPos = getManipulator().getArmConnector()
					.getCurrentPosition();

			logger.error("ArmPos: " + currentArmPos);

			currentArmRot = getManipulator().getArmConnector()
					.getCurrentRotation();

			direction = MathOperation.getDirection(currentArmPos,
					currentGoalPosition);

			logger.error("Direction: " + direction);

			sensors = getManipulator().getArmConnector()
					.receiveGripperSensorData();
		} catch (ManipulatorException e3) {
			logger.error("ANFANG FEHLER");
		}

		double stepsize = 0.01;
		int i = 0;

		// while (!((sensors.get(SensorPosition.INFRARED_RIGHT_INSIDE_NEAR) >
		// 70)
		// || (sensors.get(SensorPosition.INFRARED_LEFT_INSIDE_NEAR) > 70) ||
		// (sensors
		// .get(SensorPosition.INFRARED_MIDDLE) > 70))) {

		try {
			currentGoalPosition = (Vector3D) getManipulator().getItemMemory()
					.getFirstGraspItem()
					.getAttribute(PropertyName.WORLD_POSITION);
		} catch (ItemException e2) {
			logger.error(e2);
		} catch (InternalMemoryException e2) {
			logger.error(e2);
		}

		double counter = 0;

		logger.error("Sensor middle = "
				+ sensors.get(SensorPosition.INFRARED_MIDDLE));

		while (((sensors.get(SensorPosition.INFRARED_MIDDLE) < 240))
				&& counter < 20) {
			counter++;
			i++;

			// TODO was machen wenn position zu stark ändert

			try {

				Vector3D goalVec = new Vector3D((currentArmPos.getX() + i
						* stepsize * direction.getX()), currentArmPos.getY()
						+ i * stepsize * direction.getY(),
						currentGoalPosition.getZ());

				ArmError armError = null;
				try {
					armError = getManipulator().getArmConnector().getPosError(
							goalVec, currentArmRot);
				} catch (ManipulatorException e) {
					logger.error("2");
					logger.error(e);
				}

				logger.error(MathOperation.getEuclDistance(armError
						.getPoseError()));

				getManipulator().getArmConnector()
						.reach(goalVec, currentArmRot);
			} catch (ManipulatorException e1) {
				logger.error("ERROR REACH");
			}

			while (!getManipulator().getArmConnector().isReached()) {
				try {
					Thread.sleep(50);
				} catch (InterruptedException e) {
					logger.error(e);
				}
			}

			sensors = getManipulator().getArmConnector()
					.receiveGripperSensorData();

		}

		getManipulator().getArmConnector().closeGripper(10);

		if (getManipulator().getArmConnector().isGraspingObject()) {
			logger.error("Object in the gripper");
			try {
				getManipulator().getArmConnector().goHome();
				logger.error("back home");

				while (!getManipulator().getArmConnector().isHome()) {
					logger.error("wait for go home");
					try {
						Thread.sleep(500);
					} catch (InterruptedException e) {
						logger.error(e);
					}
				}
				setNextPartName(PartName.GO_TO_START_POSITION);
				return;
			} catch (ManipulatorException e1) {
				logger.error(e1);
				logger.error("cannot go home");
			}

		} else {
			try {
				getManipulator().getArmConnector().goHome();

				while (!getManipulator().getArmConnector().isHome()) {
					logger.error("wait for go home");
					try {
						Thread.sleep(500);
					} catch (InterruptedException e) {
						logger.error(e);
					}
				}
				logger.error("Open Gripper");
				getManipulator().getArmConnector().openGripper();

				logger.error("Try to recognize again and go on");

				// TODO was tun
				return;

			} catch (ManipulatorException e) {
				logger.error("Cannot move the arm home");
			}
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute: " + this.getClass());

		fineApproach();

		logger.debug("we go on!");

		changeToNextPart();

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {
		logger.error("Aufräumen fertig - weiter gehts");

		try {
			getManipulator().getCamConnector().resetTracker();
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}
}
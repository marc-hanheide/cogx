package manipulation.strategies.parts.mobileManipulation;

import java.util.HashMap;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.exceptions.InternalMemoryException;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.exceptions.ManipulatorException;
import manipulation.core.share.types.Matrix;
import manipulation.core.share.types.Vector3D;
import manipulation.core.share.types.SensorData.SensorPosition;
import manipulation.itemMemory.Item.PropertyName;
import manipulation.math.MathOperation;
import manipulation.strategies.Strategy;
import manipulation.strategies.parts.StrategyPart;

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
					.getFirstGraspItem().getAttribute(
							PropertyName.WORLD_POSITION);
		} catch (ItemException e4) {
			logger.error(e4);
			;
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

		while (!((sensors.get(SensorPosition.INFRARED_RIGHT_INSIDE_NEAR) > 70)
				|| (sensors.get(SensorPosition.INFRARED_LEFT_INSIDE_NEAR) > 70) || (sensors
				.get(SensorPosition.INFRARED_MIDDLE) > 70))) {
			i++;

			try {
				currentGoalPosition = (Vector3D) getManipulator()
						.getItemMemory().getFirstGraspItem().getAttribute(
								PropertyName.WORLD_POSITION);
			} catch (ItemException e2) {
				logger.error(e2);
			} catch (InternalMemoryException e2) {
				logger.error(e2);
			}

			try {

				Vector3D goalVec = new Vector3D((currentArmPos.getX() + i
						* stepsize * direction.getX()), currentArmPos.getY()
						+ i * stepsize * direction.getY(), currentGoalPosition
						.getZ());

				logger.error("goal vec" + goalVec);

				getManipulator().getArmConnector()
						.reach(goalVec, currentArmRot);
			} catch (ManipulatorException e1) {
				logger.error("ERROR REACH");
			}

			while (!getManipulator().getArmConnector().isReached()) {
				try {
					Thread.sleep(200);
				} catch (InterruptedException e) {
					logger.error(e);
				}
			}

			sensors = getManipulator().getArmConnector()
					.receiveGripperSensorData();

		}

		setNextPartName(PartName.GO_TO_START_POSITION);

		getManipulator().getArmConnector().closeGripper(5);

		try {
			if (getManipulator().getArmConnector().isGraspingObject()) {
				logger.error("Object in the gripper");
				try {
					getManipulator().getArmConnector().goHome();
					logger.error("back home");
					setNextPartName(PartName.GO_TO_START_POSITION);
					synchronized (this) {
						notifyAll();
					}
				} catch (ManipulatorException e1) {
					logger.error(e1);
					logger.error("cannot go home");
				}

			} else {
				getManipulator().getArmConnector().goHome();
				logger.error("Try to recognize again");

				setNextPartName(PartName.NEAR_RECOGNIZE);
				synchronized (this) {
					notifyAll();
				}
			}
		} catch (ManipulatorException e1) {
			logger.error("Problems while moving home!");
			logger.error(e1);
		}

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void execute() {
		logger.debug("execute - fine grasping - ");

		fineApproach();

		synchronized (this) {
			try {
				wait();
			} catch (InterruptedException e) {
				logger.error(e);
			}
		}

		logger.debug("we go on!");

		changeToNextPart();

	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void changeToNextPart() {

		try {
			getManipulator().getCamConnector().resetTracker();
		} catch (ExternalMemoryException e) {
			logger.error(e);
		}

		getGlobalStrategy().setNextPart(
				getGlobalStrategy().getPart(getNextPartName()));

	}
}
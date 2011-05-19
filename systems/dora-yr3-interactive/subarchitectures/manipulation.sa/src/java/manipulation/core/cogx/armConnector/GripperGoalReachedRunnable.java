package manipulation.core.cogx.armConnector;

import org.apache.log4j.Logger;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ManipulatorException;
import golem.tinyice.GenConfigspaceState;

/**
 * Runnable to evaluate if and when the gripper is still moving
 * 
 * @author Torben Toeniges
 * 
 */
public class GripperGoalReachedRunnable implements Runnable {
	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	/**
	 * constructor of the runnable
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public GripperGoalReachedRunnable(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	private double calculateEncoderError(GenConfigspaceState value1,
			GenConfigspaceState value2) {
		double error = 0;
		for (int i = 0; i < value1.pos.c.length - 1; i++) {
			error += Math.abs(value1.pos.c[i] - value2.pos.c[i]);
		}

		return error;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while (!manipulator.getArmConnector().isReached()) {
			GenConfigspaceState currentPos;
			try {
				currentPos = ((CogXKatanaArmConnector) manipulator
						.getArmConnector()).getCurrentConfigState();

				Thread.sleep(500);

				GenConfigspaceState currentPos1 = ((CogXKatanaArmConnector) manipulator
						.getArmConnector()).getCurrentConfigState();

				double threshold = 0.000005;

				double errorValue = calculateEncoderError(currentPos,
						currentPos1);

				logger.debug("Manipulator goal waiter error value: "
						+ errorValue);

				if (errorValue <= threshold) {
					manipulator.getArmConnector().setReached(true);
				}
			} catch (ManipulatorException e) {
				logger.error(e);
			} catch (InterruptedException e) {
				logger.error(e);
			}

		}
	}
}

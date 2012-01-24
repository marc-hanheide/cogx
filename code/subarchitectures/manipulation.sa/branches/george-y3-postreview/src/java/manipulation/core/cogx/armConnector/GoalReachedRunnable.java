package manipulation.core.cogx.armConnector;

import org.apache.log4j.Logger;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ManipulatorException;
import golem.tinyice.GenConfigspaceState;

/**
 * Runnable to evaluate if and when the arm reached its goal
 * 
 * @author Torben Toeniges
 * 
 */
public class GoalReachedRunnable implements Runnable {
	private Logger logger = Logger.getLogger(this.getClass());

	private Manipulator manipulator;

	/**
	 * constructor of the runnable
	 * 
	 * @param manipulator
	 *            current manipulator
	 */
	public GoalReachedRunnable(Manipulator manipulator) {
		this.manipulator = manipulator;
	}

	/**
	 * calculates the GenConfigspaceState-error between two different
	 * GenConfigspaceState
	 * 
	 * @param value1
	 *            first GenConfigspaceState for the error calculation
	 * @param value2
	 *            second GenConfigspaceState for the error calculation
	 * @return error value between the two GenConfigspaceStates
	 */
	private double calculateConfigStateError(GenConfigspaceState value1,
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

				double errorValue = calculateConfigStateError(currentPos,
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

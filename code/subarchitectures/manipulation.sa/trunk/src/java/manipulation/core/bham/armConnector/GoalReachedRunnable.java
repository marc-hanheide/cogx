package manipulation.core.bham.armConnector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ManipulatorException;
import golem.tinyice.GenConfigspaceState;

/**
 * Runnable to evaluate if and when the arm reached its goal
 * 
 * @author ttoenige
 * 
 */
public class GoalReachedRunnable implements Runnable {
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
				currentPos = ((BhamKatanaArmConnector) manipulator
						.getArmConnector()).getCurrentConfigState();

				Thread.sleep(500);

				GenConfigspaceState currentPos1 = ((BhamKatanaArmConnector) manipulator
						.getArmConnector()).getCurrentConfigState();

				if (calculateConfigStateError(currentPos, currentPos1) == 0) {
					manipulator.getArmConnector().setReached(true);
				}
			} catch (ManipulatorException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

		}
	}
}

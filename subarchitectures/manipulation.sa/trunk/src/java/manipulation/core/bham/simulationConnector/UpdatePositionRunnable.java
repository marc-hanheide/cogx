package manipulation.core.bham.simulationConnector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.types.BasePositionData;

import org.apache.log4j.Logger;

/**
 * Runnable to continuously update the position of the robot in the virtual
 * scene
 * 
 * @author ttoenige
 * 
 */
public class UpdatePositionRunnable implements Runnable {

	private Logger logger = Logger.getLogger(this.getClass());
	
	private BhamSimulationConnector simCon;
	private Manipulator manipulator;

	/**
	 * constructor for the robot position update runnable
	 * 
	 * @param simCon
	 *            current virtual scene connector
	 * @param manipulator
	 *            current manipulator
	 */
	public UpdatePositionRunnable(BhamSimulationConnector simCon,
			Manipulator manipulator) {
		this.manipulator = manipulator;
		this.simCon = simCon;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while (!Thread.interrupted()) {
			try {
				BasePositionData currentPos = manipulator.getBaseConnector()
						.getCurrentPosition();

				if (simCon.getRobot() != null)
					try {
						simCon.getTinyInterface().releaseActor(
								simCon.getRobot());
					} catch (Exception e) {
						logger.error(e);
					}
				try {
					simCon.setRobot(simCon.updateRobot(currentPos));
				} catch (Exception e) {
					logger.error(e);
				}
				simCon.updateArm();
			} catch (ExternalMemoryException e) {
				// TODO was mit Error machen
				logger.warn(e);
			}

			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				return;
			}
		}
	}
}

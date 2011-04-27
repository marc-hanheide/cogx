package manipulation.core.cogx.virtualSceneConnector;

import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ExternalMemoryException;
import manipulation.core.share.types.BasePositionData;
import manipulation.core.share.virtualSceneConnector.VirtualSceneConnector;

import org.apache.log4j.Logger;

/**
 * Runnable to continuously update the position of the robot in the virtual
 * scene
 * 
 * @author Torben Toeniges
 * 
 */
public class UpdatePositionRunnable implements Runnable {

	private Logger logger = Logger.getLogger(this.getClass());

	private CogXVirtualSceneConnector vitualSceneCon;
	private Manipulator manipulator;

	/**
	 * constructor for the robot position update runnable
	 * 
	 * @param simCon
	 *            current virtual scene connector
	 * @param manipulator
	 *            current manipulator
	 */
	public UpdatePositionRunnable(VirtualSceneConnector simCon,
			Manipulator manipulator) {
		this.manipulator = manipulator;
		this.vitualSceneCon = (CogXVirtualSceneConnector) simCon;
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

				if (vitualSceneCon.getRobot() != null)
					try {
						vitualSceneCon.getTinyInterface().releaseActor(
								vitualSceneCon.getRobot());
					} catch (Exception e) {
						logger.error(e);
					}
				try {
					vitualSceneCon.setRobot(vitualSceneCon
							.updateRobot(currentPos));
				} catch (Exception e) {
					logger.error(e);
				}
				vitualSceneCon.updateArm();
			} catch (ExternalMemoryException e) {
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

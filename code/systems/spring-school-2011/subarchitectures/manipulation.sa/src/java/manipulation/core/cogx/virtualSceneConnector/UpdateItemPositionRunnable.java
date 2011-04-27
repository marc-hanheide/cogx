package manipulation.core.cogx.virtualSceneConnector;

import golem.tinyice.ExTiny;
import golem.tinyice.RigidBodyPrx;
import manipulation.core.share.Manipulator;
import manipulation.core.share.exceptions.ItemException;
import manipulation.core.share.virtualSceneConnector.VirtualSceneConnector;
import manipulation.itemMemory.Item;

import org.apache.log4j.Logger;

/**
 * Runnable to continuously update the item positions
 * 
 * @author Torben Toeniges
 * 
 */
public class UpdateItemPositionRunnable implements Runnable {
	private Logger logger = Logger.getLogger(this.getClass());

	private CogXVirtualSceneConnector virtualSceneCon;
	private Manipulator manipulator;

	/**
	 * constructor for the item update runnable
	 * 
	 * @param simCon
	 *            current virtual scene connector
	 * @param manipulator
	 *            current manipulator
	 */
	public UpdateItemPositionRunnable(VirtualSceneConnector simCon,
			Manipulator manipulator) {
		this.manipulator = manipulator;
		this.virtualSceneCon = (CogXVirtualSceneConnector) simCon;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void run() {
		while (!Thread.interrupted()) {
			for (RigidBodyPrx obstacle : virtualSceneCon.getObstacles()) {
				try {
					virtualSceneCon.getTinyInterface().releaseActor(obstacle);
				} catch (Exception e) {
					logger.error(e);
				}
			}
			virtualSceneCon.getObstacles().clear();

			for (Item item : manipulator.getItemMemory().getItemList()) {
				try {
					virtualSceneCon.getObstacles().add(
							virtualSceneCon.updateObstacle(item));
				} catch (ExTiny e) {
					logger.error(e);
				} catch (ItemException e) {
					logger.error(e);
				}
			}

			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				return;
			}
		}
	}
}
